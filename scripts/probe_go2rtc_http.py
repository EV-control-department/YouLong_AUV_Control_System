#!/usr/bin/env python3
"""Check go2rtc and a configured stream without opening the media source."""

import json
import re
import sys
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


def fail(message):
    print(message, file=sys.stderr)
    return 1


def main():
    if len(sys.argv) != 4:
        return fail("usage: probe_go2rtc_http.py HOST PORT STREAM")

    host, port_text, stream = sys.argv[1:]
    try:
        port = int(port_text)
        if not 1 <= port <= 65535:
            raise ValueError
    except ValueError:
        return fail(f"无效的 HTTP 端口：{port_text}")

    if not re.fullmatch(r"[a-zA-Z0-9_-]+", stream):
        return fail(f"非法流名称：{stream}")

    bare_host = host.strip("[]")
    authority_host = f"[{bare_host}]" if ":" in bare_host else bare_host
    authority = f"{authority_host}:{port}"
    url = f"http://{authority}/api/streams"
    request = Request(url, headers={"User-Agent": "ZIT6-recorder/1.0"})

    try:
        with urlopen(request, timeout=8.0) as response:
            if response.status != 200:
                return fail(f"{url}: HTTP {response.status}")
            streams = json.loads(response.read().decode("utf-8"))
    except HTTPError as error:
        return fail(f"{url}: HTTP {error.code} {error.reason}")
    except (URLError, OSError, TimeoutError) as error:
        reason = getattr(error, "reason", error)
        return fail(f"{url}: {reason}")
    except (UnicodeDecodeError, json.JSONDecodeError) as error:
        return fail(f"{url}: 无法解析 go2rtc 流列表：{error}")

    if not isinstance(streams, dict):
        return fail(f"{url}: go2rtc 返回了无效的流列表")
    if stream not in streams:
        return fail(f"{url}: 未配置视频源 {stream}")
    producers = streams[stream].get("producers") or []
    if not producers:
        return fail(f"{url}: 视频源 {stream} 没有配置 producer")

    print(f"go2rtc source configured ({len(producers)} producer(s))")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
