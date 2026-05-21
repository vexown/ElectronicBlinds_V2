#!/usr/bin/env python3
"""Generate an FNV-1a hash -> source-path map for FaultHandler crash reports.

FaultHandler_RecordAssert() stores a 32-bit FNV-1a hash of __FILE__ in a
watchdog scratch register, because a full path will not fit in 32 bits. This
build passes absolute source paths to the compiler, so __FILE__ expands to the
absolute path - that is the exact string this script hashes.

Keep the generated map next to the .elf it was built with: the hash depends on
the build-time absolute path, so a map from a different checkout will not match.

Usage: gen_file_hashes.py <repo-root> <output-file>
"""
import os
import sys

SKIP = {"build", "output", ".git", "pico-sdk", "picotool", "openocd", "debugprobe"}


def fnv1a32(s):
    h = 0x811C9DC5
    for b in s.encode("utf-8"):
        h ^= b
        h = (h * 0x01000193) & 0xFFFFFFFF
    return h


repo_root = os.path.abspath(sys.argv[1])
out_path = sys.argv[2]

rows = []
for dirpath, dirnames, filenames in os.walk(repo_root):
    dirnames[:] = [d for d in dirnames if d not in SKIP]
    for name in filenames:
        if name.endswith((".c", ".h")):
            path = os.path.join(dirpath, name)
            rows.append((fnv1a32(path), path))

with open(out_path, "w") as fh:
    for h, path in sorted(rows):
        fh.write("0x%08x  %s\n" % (h, path))

print("  wrote %d file hashes to %s" % (len(rows), out_path))
