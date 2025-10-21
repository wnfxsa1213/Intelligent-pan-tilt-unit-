# Usage: python3 tools/generate_docs_index.py
# 生成 docs/docs_index.md 供团队速览
from __future__ import annotations

import argparse
import datetime as dt
import re
from pathlib import Path
from typing import Iterable, List, Tuple


def _extract_title(lines: Iterable[str], fallback: str) -> str:
    for raw in lines:
        stripped = raw.strip()
        if stripped.startswith("#"):
            return stripped.lstrip("#").strip() or fallback
    return fallback


LINKED_LIST_PATTERN = re.compile(r"^\d+\.\s*\[.*\]\(.*\)")


def _extract_summary(lines: Iterable[str], fallback: str) -> str:
    for raw in lines:
        stripped = raw.strip()
        if not stripped:
            continue
        if stripped.startswith("#"):
            continue
        if stripped.startswith(">"):
            continue
        if stripped.startswith("|"):
            continue
        if LINKED_LIST_PATTERN.match(stripped):
            continue
        candidate = stripped.lstrip("-* ").strip()
        if candidate.startswith(("**", "__")):
            candidate = candidate.strip("*_ ").strip()
        candidate = candidate.replace("**", "").replace("__", "")
        if candidate:
            return candidate
    return fallback


def _format_timestamp(ts: float) -> str:
    return dt.datetime.fromtimestamp(ts).strftime("%Y-%m-%d %H:%M")


def _collect_docs(docs_dir: Path, ignore: List[str]) -> List[Tuple[str, str, str, str]]:
    entries: List[Tuple[str, str, str, str]] = []
    for path in sorted(docs_dir.glob("*.md")):
        if path.name in ignore:
            continue
        content = path.read_text(encoding="utf-8").splitlines()
        title = _extract_title(content, path.stem)
        summary = _extract_summary(content, fallback=title)
        timestamp = _format_timestamp(path.stat().st_mtime)
        entries.append((path.name, title, summary, timestamp))
    return entries


def _render_markdown(entries: List[Tuple[str, str, str, str]]) -> str:
    lines = [
        "# docs 目录索引",
        "",
        "| 文档 | 标题 | 摘要 | 最近修改 |",
        "|------|------|------|----------|",
    ]
    for filename, title, summary, timestamp in entries:
        summary = summary or "暂无摘要"
        truncated = summary[:80] + ("..." if len(summary) > 80 else "")
        description = truncated.replace("|", "\\|")
        lines.append(f"| [{filename}]({filename}) | {title} | {description} | {timestamp} |")
    lines.append("")
    lines.append(
        f"共收录 {len(entries)} 篇文档，运行 `python3 tools/generate_docs_index.py` 可重新生成。"
    )
    lines.append("")
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(description="生成 docs 目录索引")
    parser.add_argument(
        "--docs-dir",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "docs",
        help="文档目录路径",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "docs" / "docs_index.md",
        help="输出文件路径",
    )
    args = parser.parse_args()

    docs_dir = args.docs_dir
    if not docs_dir.is_dir():
        raise SystemExit(f"文档目录不存在：{docs_dir}")

    entries = _collect_docs(docs_dir, ignore=[args.output.name])
    markdown = _render_markdown(entries)
    args.output.write_text(markdown, encoding="utf-8")


if __name__ == "__main__":
    main()
