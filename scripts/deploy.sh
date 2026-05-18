#!/usr/bin/env bash

# Deploy the project to the AUV computer with rsync.
#
# Usage:
#   ./scripts/deploy.sh
#   ./scripts/deploy.sh --dry-run
#   ./scripts/deploy.sh --delete
#   ./scripts/deploy.sh --checksum workspace_auv/src/uv_perception/uv_perception/vision.py
#   ./scripts/deploy.sh --git-changed --checksum
#
# The connection defaults can be overridden with environment variables:
#   DEPLOY_USER=nvidia DEPLOY_HOST=192.168.16.10 \
#   DEPLOY_PATH='~/YouLong_AUV_Control_System' ./scripts/deploy.sh

set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

DEPLOY_USER="${DEPLOY_USER:-nvidia}"
DEPLOY_HOST="${DEPLOY_HOST:-192.168.16.10}"
DEPLOY_PATH="${DEPLOY_PATH:-~/YouLong_AUV_Control_System}"
SSH_PORT="${SSH_PORT:-22}"
SSH_KEY="${SSH_KEY:-}"

dry_run=false
delete_remote=false
checksum=false
git_changed=false
git_range=''
selected_paths=()

usage() {
    cat <<'EOF'
用法：
  ./scripts/deploy.sh [选项] [文件或目录 ...]

选项：
  -n, --dry-run  只显示将要同步的文件，不修改远端
  -d, --delete   删除远端中本地不存在的文件（谨慎使用）
  -c, --checksum 按文件内容 checksum 比较，而不是只比较大小和修改时间
      --git-changed
                 部署 Git 工作区相对 HEAD 的变更文件（含未跟踪文件）
      --git-range RANGE
                 部署指定 Git 提交范围内变更的文件，例如 HEAD~1..HEAD
  -h, --help     显示帮助

指定路径：
  不指定路径时同步整个项目；指定文件或目录时只同步这些路径。
  路径必须位于项目目录内，可以重复指定多个路径。

环境变量：
  DEPLOY_USER   SSH 用户，默认 nvidia
  DEPLOY_HOST   SSH 主机，默认 192.168.16.10
  DEPLOY_PATH   远端目录，默认 ~/YouLong_AUV_Control_System
  SSH_PORT      SSH 端口，默认 22
  SSH_KEY       SSH 私钥路径，可选

示例：
  ./scripts/deploy.sh
  ./scripts/deploy.sh --dry-run
  ./scripts/deploy.sh --checksum --dry-run workspace_auv/src/uv_perception/uv_perception/vision.py
  ./scripts/deploy.sh --git-changed --checksum --dry-run
  ./scripts/deploy.sh --git-range HEAD~1..HEAD --checksum
  ./scripts/deploy.sh workspace_auv/src/uv_perception/uv_perception/vision.py
  ./scripts/deploy.sh workspace_auv/src/uv_perception/uv_perception/vision.py workspace_auv/src/uv_perception/config
  DEPLOY_PATH='~/deploy/YouLong_AUV_Control_System' ./scripts/deploy.sh
EOF
}

while (($# > 0)); do
    case "$1" in
        -n|--dry-run)
            dry_run=true
            ;;
        -d|--delete)
            delete_remote=true
            ;;
        -c|--checksum)
            checksum=true
            ;;
        --git-changed)
            git_changed=true
            ;;
        --git-range)
            if (($# < 2)); then
                printf '错误：--git-range 需要一个提交范围。\n' >&2
                exit 2
            fi
            git_range="$2"
            shift
            ;;
        --git-range=*)
            git_range="${1#*=}"
            if [[ -z "$git_range" ]]; then
                printf '错误：--git-range 不能为空。\n' >&2
                exit 2
            fi
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        --)
            shift
            while (($# > 0)); do
                selected_paths+=("$1")
                shift
            done
            break
            ;;
        *)
            if [[ "$1" == -* ]]; then
                printf '错误：未知参数 %s\n\n' "$1" >&2
                usage >&2
                exit 2
            fi
            selected_paths+=("$1")
            ;;
    esac
    shift
done

if ! command -v ssh >/dev/null 2>&1; then
    printf '错误：未找到 ssh 命令。\n' >&2
    exit 127
fi

if ! command -v rsync >/dev/null 2>&1; then
    printf '错误：未找到 rsync 命令，请先安装 rsync。\n' >&2
    exit 127
fi

if [[ "$git_changed" == true || -n "$git_range" ]] && ! command -v git >/dev/null 2>&1; then
    printf '错误：Git 模式需要 git 命令。\n' >&2
    exit 127
fi

if [[ -z "$DEPLOY_USER" || -z "$DEPLOY_HOST" || -z "$DEPLOY_PATH" ]]; then
    printf '错误：DEPLOY_USER、DEPLOY_HOST、DEPLOY_PATH 不能为空。\n' >&2
    exit 2
fi

if [[ "$git_changed" == true && -n "$git_range" ]]; then
    printf '错误：--git-changed 与 --git-range 不能同时使用。\n' >&2
    exit 2
fi

if [[ "$delete_remote" == true && ( ${#selected_paths[@]} -gt 0 || "$git_changed" == true || -n "$git_range" ) ]]; then
    printf '错误：指定路径部署不能与 --delete 同时使用。\n' >&2
    exit 2
fi

if [[ "$git_changed" == true || -n "$git_range" ]]; then
    git_root="$(git -C "$PROJECT_ROOT" rev-parse --show-toplevel 2>/dev/null || true)"
    if [[ "$(realpath -- "$git_root" 2>/dev/null || true)" != "$PROJECT_ROOT" ]]; then
        printf '错误：项目目录不是 Git 仓库根目录：%s\n' "$PROJECT_ROOT" >&2
        exit 2
    fi

    git_paths=()
    deleted_git_paths=()
    if [[ "$git_changed" == true ]]; then
        mapfile -t git_paths < <(
            {
                git -C "$PROJECT_ROOT" diff --name-only --diff-filter=ACMRTUXB HEAD --
                git -C "$PROJECT_ROOT" ls-files --others --exclude-standard
            } | sort -u
        )
        mapfile -t deleted_git_paths < <(
            git -C "$PROJECT_ROOT" diff --name-only --diff-filter=D HEAD --
        )
    else
        mapfile -t git_paths < <(
            git -C "$PROJECT_ROOT" diff --name-only --diff-filter=ACMRTUXB "$git_range" -- | sort -u
        )
        mapfile -t deleted_git_paths < <(
            git -C "$PROJECT_ROOT" diff --name-only --diff-filter=D "$git_range" --
        )
    fi

    if ((${#deleted_git_paths[@]} > 0)); then
        printf '提示：以下 Git 删除项不会被选择性部署：\n' >&2
        for path in "${deleted_git_paths[@]}"; do
            printf '  - %s\n' "$path" >&2
        done
        printf '如需删除远端文件，请单独确认后使用完整目录的 --delete。\n' >&2
    fi
    selected_paths+=("${git_paths[@]}")
fi

if [[ ( "$git_changed" == true || -n "$git_range" ) && ${#selected_paths[@]} -eq 0 ]]; then
    printf '没有检测到需要部署的 Git 文件，已退出。\n'
    exit 0
fi

normalized_paths=()
declare -A seen_paths=()
for path in "${selected_paths[@]}"; do
    if [[ -z "$path" ]]; then
        printf '错误：部署路径不能为空。\n' >&2
        exit 2
    fi
    resolved_path="$(realpath -- "$PROJECT_ROOT/$path" 2>/dev/null || true)"
    case "$resolved_path" in
        "$PROJECT_ROOT"|"$PROJECT_ROOT"/*)
            ;;
        *)
            printf '错误：部署路径必须位于项目目录内：%s\n' "$path" >&2
            exit 2
            ;;
    esac
    if [[ ! -e "$resolved_path" ]]; then
        printf '错误：部署路径不存在：%s\n' "$path" >&2
        exit 2
    fi
    relative_path="${resolved_path#${PROJECT_ROOT}/}"
    if [[ -n "${seen_paths[$relative_path]+x}" ]]; then
        continue
    fi
    seen_paths[$relative_path]=1
    normalized_paths+=("$relative_path")
done
selected_paths=("${normalized_paths[@]}")

if [[ "$DEPLOY_PATH" == *$'\n'* || "$DEPLOY_PATH" == *$'\r'* ]]; then
    printf '错误：DEPLOY_PATH 不能包含换行符。\n' >&2
    exit 2
fi

readonly REMOTE="${DEPLOY_USER}@${DEPLOY_HOST}"

ssh_command=(ssh)
if [[ "$SSH_PORT" != "22" ]]; then
    ssh_command+=(-p "$SSH_PORT")
fi
if [[ -n "$SSH_KEY" ]]; then
    ssh_command+=(-i "$SSH_KEY")
fi

# rsync -e takes a command string, while ssh itself is kept as an array above
# so that paths containing spaces are passed correctly to the preflight call.
rsync_ssh_command="$(printf '%q ' "${ssh_command[@]}")"
rsync_ssh_command="${rsync_ssh_command% }"

printf '部署源目录：%s/\n' "$PROJECT_ROOT"
printf '部署目标：%s:%s\n' "$REMOTE" "$DEPLOY_PATH"
if [[ ${#selected_paths[@]} -eq 0 ]]; then
    printf '部署范围：整个项目\n'
else
    printf '部署范围：指定路径\n'
    for path in "${selected_paths[@]}"; do
        printf '  - %s\n' "$path"
    done
fi

if [[ "$dry_run" == true ]]; then
    printf '模式：dry-run（不会修改远端）\n'
fi
if [[ "$checksum" == true ]]; then
    printf '比较方式：文件内容 checksum\n'
fi
if [[ "$git_changed" == true ]]; then
    printf 'Git 范围：工作区相对 HEAD\n'
elif [[ -n "$git_range" ]]; then
    printf 'Git 范围：%s\n' "$git_range"
fi
if [[ "$delete_remote" == true ]]; then
    printf '警告：已启用 --delete，远端多余文件将被删除。\n'
fi

printf '检查 SSH 连接...\n'
# Expand ~/... on the remote side before creating the destination directory.
if [[ "$DEPLOY_PATH" == '~/'* ]]; then
    remote_mkdir_path="\$HOME/${DEPLOY_PATH#~/}"
elif [[ "$DEPLOY_PATH" == '~' ]]; then
    remote_mkdir_path='$HOME'
else
    remote_mkdir_path="$DEPLOY_PATH"
fi
if [[ "$dry_run" == true ]]; then
    printf 'dry-run：跳过创建远端目录。\n'
else
    "${ssh_command[@]}" "$REMOTE" "mkdir -p -- \"$remote_mkdir_path\""
fi

rsync_options=(
    --archive
    --compress
    --human-readable
    --info=progress2
    --partial
    --exclude=.git/
    --exclude=.codex/
    --exclude=.agents/
    --exclude='**/__pycache__/'
    --exclude='*.py[cod]'
    --exclude='build/'
    --exclude='install/'
    --exclude='log/'
)

if [[ "$dry_run" == true ]]; then
    rsync_options+=(--dry-run)
fi
if [[ "$delete_remote" == true ]]; then
    rsync_options+=(--delete)
fi
if [[ "$checksum" == true ]]; then
    rsync_options+=(--checksum)
fi
if [[ "$dry_run" == true || ${#selected_paths[@]} -gt 0 ]]; then
    rsync_options+=(--itemize-changes)
fi

if [[ ${#selected_paths[@]} -eq 0 ]]; then
    rsync "${rsync_options[@]}" \
        -e "$rsync_ssh_command" \
        "${PROJECT_ROOT}/" \
        "${REMOTE}:${DEPLOY_PATH}/"
else
    (
        cd -- "$PROJECT_ROOT"
        rsync "${rsync_options[@]}" \
            --relative \
            -e "$rsync_ssh_command" \
            "${selected_paths[@]}" \
            "${REMOTE}:${DEPLOY_PATH}/"
    )
fi

printf '\n部署完成。\n'
