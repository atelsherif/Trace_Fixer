"""Server-side directory listing for the "browse to a folder" picker in the
Scan directory UI. A plain `<input type=file webkitdirectory>` can't hand
back an absolute filesystem path (browsers deliberately don't expose one),
and the app doesn't assume a native OS dialog toolkit is available on
whatever machine runs the server -- so instead the browser walks the
filesystem the *server* can see, one directory at a time, over a couple of
small JSON endpoints.
"""
from __future__ import annotations

from pathlib import Path


def list_subdirectories(path: str | None) -> dict:
    root = Path(path).expanduser().resolve() if path else Path.home()
    if not root.is_dir():
        raise NotADirectoryError(str(root))
    dirs = []
    try:
        for child in root.iterdir():
            if child.is_dir() and not child.name.startswith("."):
                dirs.append(child.name)
    except PermissionError:
        pass
    dirs.sort(key=str.lower)
    parent = str(root.parent) if root.parent != root else None
    return {"path": str(root), "parent": parent, "dirs": dirs}
