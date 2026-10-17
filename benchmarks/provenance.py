"""Which code produced a benchmark result (written into every summary)."""

import subprocess
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent


def git_commit():
    """Short HEAD hash, flagged when tracked files have uncommitted changes.

    core.autocrlf=input: the repo is a Windows checkout (CRLF working files)
    read by Linux git in the container, which would otherwise report every
    CRLF file as modified.
    """
    def git(*args):
        return subprocess.run(["git", "-c", "safe.directory=*", "-c", "core.autocrlf=input",
                               *args], capture_output=True, text=True, cwd=REPO,
                              check=True).stdout.strip()
    try:
        head = git("rev-parse", "--short", "HEAD")
        dirty = git("status", "--porcelain", "--untracked-files=no")
    except (OSError, subprocess.CalledProcessError):
        return "unknown"
    return f"{head} (+ uncommitted changes)" if dirty else head
