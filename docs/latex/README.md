# YouLong AUV 系统设计、实现与实验手册

这是项目的正式技术文档工程。它与仓库中的 Markdown 文档分工如下：

- `docs/*.md` 和 `docs/architecture/*.md`：开发过程中的快速查阅和接口备忘。
- `docs/latex/`：可归档、可打印、可引用的系统设计、实现和实验手册。
- `workspace_auv`、`workspace_sim` 以及各配置文件：代码和参数的事实来源。

## 构建

在仓库根目录执行：

```bash
cd docs/latex
make
```

PDF 输出为 `docs/latex/build/youlong-auv-manual.pdf`。构建前会自动生成
`build/git-info.tex`，在封面写入当前 commit、分支和构建日期；该文件属于构建产物，
不应提交到 Git。

只检查 LaTeX 是否能编译而不打开 PDF：

```bash
make check
```

清理构建产物：

```bash
make clean
```

## 维护规则

1. 参数数值、Topic 名称、消息字段和坐标系实现以源码/配置为准；手册解释其含义，
   不复制一份容易漂移的“第二参数真值”。
2. 每个章节区分“当前实现”“设计约束”和“后续工作”，不得把预留接口写成已完成能力。
3. 变更接口或坐标约定时，同一个提交中同时更新代码、快速查阅 Markdown 和本手册。
4. 构建发布版前记录 commit、实验参数和数据目录，保证图表可以追溯。

## 文档入口

- 主文件：`main.tex`
- 公共样式：`preamble.tex`
- 正文：`chapters/`
- 附录：`appendices/`
- 文献：`references.bib`
- 图表资源：`figures/`、`tables/`

当前正文从第 1 章扩展到第 20 章：第 17--20 章专门记录包级职责、运行时数据流、
安全状态模型以及构建/发布/实验数据闭环。新增模块时，优先更新第 17 章和附录 E；
新增 Topic 或运行模式时，优先更新第 18 章和附录 F。
