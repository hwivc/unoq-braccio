"""MkDocs hook: show real code from the repository inside the guide.

Write a line like one of these in any page:

    @code path/to/file.py                     the whole file
    @code path/to/file.py SerialBridge.tick   one function, method or class
    @code path/to/file.py L10-L40             a line range
    @code path/to/file.ino handleLine         a C/C++/Arduino function

It becomes a code block titled with the file and line numbers, with the
original line numbers in the margin and a copy button. The code is read from
the repository each time the site builds, so it never goes stale. Paths are
relative to the repository root (the folder holding mkdocs.yml).
"""

from __future__ import annotations

import ast
import os
import re

LANG = {".py": "python", ".ino": "cpp", ".cpp": "cpp", ".h": "cpp", ".xacro": "xml",
        ".urdf": "xml", ".xml": "xml", ".yaml": "yaml", ".yml": "yaml", ".js": "javascript",
        ".html": "html", ".css": "css", ".sh": "bash", ".txt": "text", ".rules": "text"}
LINE = re.compile(r"^@code\s+(\S+)(?:\s+(\S+))?\s*$", re.M)
_root = "."


def on_config(config):
    global _root
    _root = os.path.dirname(config.config_file_path)


def _python_symbol(source, name):
    tree = ast.parse(source)
    node = tree
    for part in name.split("."):
        found = None
        for child in ast.iter_child_nodes(node):
            if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)) and child.name == part:
                found = child
                break
        if found is None:
            raise KeyError(name)
        node = found
    start = min([node.lineno] + [d.lineno for d in node.decorator_list])
    return start, node.end_lineno


def _brace_symbol(lines, name):
    pattern = re.compile(r"^[A-Za-z_][\w\s\*&<>:]*\b" + re.escape(name) + r"\s*\(")
    for i, line in enumerate(lines):
        if pattern.match(line):
            depth, opened = 0, False
            for j in range(i, len(lines)):
                depth += lines[j].count("{") - lines[j].count("}")
                opened = opened or "{" in lines[j]
                if opened and depth == 0:
                    return i + 1, j + 1
    raise KeyError(name)


def _block(match):
    path, what = match.group(1), match.group(2)
    full = os.path.join(_root, path)
    with open(full, encoding="utf-8") as handle:
        source = handle.read()
    lines = source.splitlines()
    if what is None:
        start, end = 1, len(lines)
    elif re.fullmatch(r"L\d+-L\d+", what):
        start, end = (int(x) for x in what[1:].split("-L"))
    elif path.endswith(".py"):
        start, end = _python_symbol(source, what)
    else:
        start, end = _brace_symbol(lines, what)
    ext = os.path.splitext(path)[1]
    lang = LANG.get(ext, "text")
    title = f"{path}  (lines {start}-{end})" if what else path
    code = "\n".join(lines[start - 1:end])
    return f'``` {lang} title="{title}" linenums="{start}"\n{code}\n```'


def on_page_markdown(markdown, page, config, files):
    return LINE.sub(_block, markdown)
