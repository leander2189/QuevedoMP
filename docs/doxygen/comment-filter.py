#!/usr/bin/env python3
r"""Doxygen INPUT_FILTER: present QuevedoMP's plain `//` header comments to Doxygen.

The public headers are written to be read as source, in ordinary `//` comments. Doxygen ignores
those entirely — it only recognises `///`, `//!`, `/** */` and `/*! */` — so without this filter the
generated API reference would be a bare list of signatures with no prose at all. Rather than churn
537 comment lines across 24 headers into Doxygen markup (and make every future header carry markup
it does not need), we rewrite the comments on the way into Doxygen and leave the source alone.

What it does, per file:

  * The **leading comment block** — the file-header paragraph before `#pragma once` — becomes a
    `@file` block. Without this it would attach to whatever entity comes next, which would silently
    label the first struct in the file with the file's description. That failure is quiet and
    wrong, which is why it is handled first.
  * Every other **run of full-line comments** becomes one `/** ... */` block. With
    JAVADOC_AUTOBRIEF these get a brief (first sentence) and a detailed description for free.
  * **Trailing comments are left alone** (`int x = 3; // why`). Converting them would produce a doc
    comment that attaches to the *next* declaration — again quietly wrong. 146 lines in the public
    headers are trailing comments, so this case matters more than it looks.
  * **Separator comments** (`// ---- section ----`) stay plain. They document nothing, and as doc
    comments they would attach themselves to the following member.

Deliberately NOT handled: angle-bracket placeholders in prose (`package://<pkg>/<rest>`). Doxygen
logs "Unsupported xml/html tag" for those, but it still escapes and renders them correctly — this
was checked against generated HTML, not assumed. Escaping them with `\<` makes the output strictly
worse: the backslashes survive verbatim into brief descriptions. The warnings are noise in a log;
the pages are right. Leave them alone.

Doxygen invokes this as `comment-filter.py <file>` and reads the result from stdout, so the source
tree is never modified. Run it by hand on any header to see exactly what Doxygen will parse.
"""

from __future__ import annotations

import re
import sys

# A line that is nothing but a comment: optional indent, `//`, optional text. The `(?!/)` keeps any
# pre-existing `///` block (there are none today) from being wrapped a second time.
FULL_LINE_COMMENT = re.compile(r"^(\s*)//(?!/)[ \t]?(.*)$")

# `// ---- ... ----`, `// ====`, and friends: visual separators, not documentation.
SEPARATOR = re.compile(r"^\s*//\s*[-=*_]{3,}")


def emit_block(out: list[str], indent: str, body: list[str], *, file_block: bool) -> None:
    """Write one `/** ... */` block, or drop it if it carries no text."""
    while body and not body[-1].strip():  # trailing blank comment lines add nothing
        body.pop()
    if not body and not file_block:
        return
    out.append(f"{indent}/**")
    if file_block:
        out.append(f"{indent} * @file")
    for line in body:
        out.append(f"{indent} *{(' ' + line).rstrip()}")
    out.append(f"{indent} */")


def convert(text: str) -> str:
    lines = text.splitlines()
    out: list[str] = []
    i = 0
    # The file-header block is only the run of comments the file *opens* with.
    first_block = bool(lines) and FULL_LINE_COMMENT.match(lines[0]) is not None

    while i < len(lines):
        m = FULL_LINE_COMMENT.match(lines[i])
        if m is None or SEPARATOR.match(lines[i]):
            out.append(lines[i])
            i += 1
            continue

        indent = m.group(1)
        body: list[str] = []
        while i < len(lines):
            m = FULL_LINE_COMMENT.match(lines[i])
            if m is None or SEPARATOR.match(lines[i]) or m.group(1) != indent:
                break
            body.append(m.group(2))
            i += 1

        emit_block(out, indent, body, file_block=first_block)
        first_block = False

    return "\n".join(out) + "\n"


def main() -> int:
    if len(sys.argv) != 2:
        print(f"usage: {sys.argv[0]} <file>", file=sys.stderr)
        return 2
    with open(sys.argv[1], encoding="utf-8") as f:
        sys.stdout.write(convert(f.read()))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
