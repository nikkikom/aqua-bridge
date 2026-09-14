#!/usr/bin/env python3
"""Create or update a user of the HTTPS API (PROJECT.md sections 6 and 10).

Run on the Pi as root, from the install directory::

    sudo .venv/bin/python tools/http_user.py --config /etc/aqua-bridge/config.yaml \\
        --group <service user> <name>

The password is read from a prompt (twice), or with ``--stdin`` from the first
line of standard input; never from the command line, where ``ps`` and the shell
history would show it. The user's line in ``http.credentials_file`` is replaced
or appended (other lines and comments are kept) with a PBKDF2-HMAC-SHA256 hash
of ``http.hash_iterations`` iterations and a fresh per-user salt. The file is
written atomically with mode 0640; an existing file keeps its owner and group,
a new one gets ``--group`` (the service user, so the daemon can read it). The
daemon rereads the file on the next request; no restart is needed.

``--file`` writes another file than the config's ``http.credentials_file``.
Without ``--config`` the default config path is used when it exists, otherwise
the defaults of :class:`aqua_bridge.publishers.httpauth.HttpSettings`.
"""

from __future__ import annotations

import argparse
import contextlib
import getpass
import grp
import os
import sys
import tempfile
from collections.abc import Callable
from pathlib import Path
from typing import TextIO

from aqua_bridge.config import ConfigError, load_config
from aqua_bridge.publishers.httpauth import (
    HttpSettings,
    HttpSetupError,
    hash_password,
    update_credentials_text,
    valid_username,
)

__all__ = ["build_parser", "main", "read_password", "write_user"]

DEFAULT_CONFIG = Path("/etc/aqua-bridge/config.yaml")
FILE_MODE = 0o640


class ToolError(Exception):
    pass


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Create or update a user of the aqua-bridge HTTPS API "
        "(the password is prompted for, or read from stdin with --stdin)."
    )
    parser.add_argument("user", help="user name (A-Z a-z 0-9 . _ @ + -)")
    parser.add_argument(
        "--config",
        type=Path,
        default=None,
        help=f"config.yaml whose http: section names the file and iterations "
        f"(default {DEFAULT_CONFIG} when it exists)",
    )
    parser.add_argument(
        "--file", type=Path, default=None, help="credentials file (overrides the config)"
    )
    parser.add_argument(
        "--group",
        default=None,
        help="group of a newly created file (the service user); an existing file keeps its group",
    )
    parser.add_argument(
        "--stdin",
        action="store_true",
        help="read the password from the first line of standard input instead of a prompt",
    )
    return parser


def load_settings(config: Path | None) -> HttpSettings:
    path = config
    if path is None and DEFAULT_CONFIG.exists():
        path = DEFAULT_CONFIG
    if path is None:
        return HttpSettings()
    try:
        return HttpSettings.from_section(load_config(path).section("http"))
    except (ConfigError, HttpSetupError) as exc:
        raise ToolError(str(exc)) from exc


def read_password(
    *,
    stdin: bool,
    stream: TextIO | None = None,
    prompt: Callable[[str], str] | None = None,
) -> str:
    if stdin:
        line = (stream if stream is not None else sys.stdin).readline()
        password = line.removesuffix("\n").removesuffix("\r")
    else:
        if prompt is None:
            prompt = getpass.getpass
        password = prompt("Password: ")
        if prompt("Repeat password: ") != password:
            raise ToolError("passwords do not match")
    if not password:
        raise ToolError("empty password")
    try:
        password.encode("utf-8")
    except UnicodeEncodeError as exc:
        raise ToolError("password is not valid UTF-8") from exc
    return password


def write_user(
    path: Path, user: str, password: str, iterations: int, *, group: str | None = None
) -> bool:
    """Replace or append ``user``; ``True`` when the file was created."""
    if not valid_username(user):
        raise ToolError(f"invalid user name {user!r} (allowed: A-Z a-z 0-9 . _ @ + -)")
    created = not path.exists()
    text = ""
    if not created:
        try:
            text = path.read_text(encoding="utf-8")
        except (OSError, UnicodeDecodeError) as exc:
            raise ToolError(f"cannot read {path}: {exc}") from exc
    try:
        new_text = update_credentials_text(text, user, hash_password(password, iterations))
    except ValueError as exc:
        raise ToolError(f"{path}: {exc}; fix the file by hand first") from exc
    if created:
        uid, gid = os.getuid(), os.getgid()
        if group is not None:
            try:
                gid = grp.getgrnam(group).gr_gid
            except KeyError as exc:
                raise ToolError(f"unknown group {group!r}") from exc
    else:
        st = path.stat()
        uid, gid = st.st_uid, st.st_gid
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp_name = tempfile.mkstemp(prefix=".http-users.", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as fh:
            os.fchmod(fh.fileno(), FILE_MODE)
            if (uid, gid) != (os.getuid(), os.getgid()):
                os.fchown(fh.fileno(), uid, gid)
            fh.write(new_text)
            fh.flush()
            os.fsync(fh.fileno())
        os.replace(tmp_name, path)
    except BaseException:
        with contextlib.suppress(OSError):
            os.unlink(tmp_name)
        raise
    return created


def main(argv: list[str] | None = None, *, stdin: TextIO | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        settings = load_settings(args.config)
        path = args.file if args.file is not None else Path(settings.credentials_file)
        if not valid_username(args.user):
            raise ToolError(f"invalid user name {args.user!r} (allowed: A-Z a-z 0-9 . _ @ + -)")
        password = read_password(stdin=args.stdin, stream=stdin)
        created = write_user(path, args.user, password, settings.hash_iterations, group=args.group)
    except (ToolError, OSError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    action = "created" if created else "updated"
    print(f"{action} {path}: user {args.user} ({settings.hash_iterations} iterations)")
    if created and args.group is None:
        print(
            f"warning: {path} belongs to your own group; the service must be able to read "
            f"it: sudo chown root:<service user> {path}",
            file=sys.stderr,
        )
    print("the daemon rereads the file on the next request; no restart needed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
