#!/usr/bin/env python3
"""Fetch and verify the pinned OpenRobotics Sydney Regatta terrain once."""

import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import stat
import sys
import tempfile
import time
import urllib.request
import zipfile


REPO = Path(__file__).resolve().parents[1]
MANIFEST = REPO / "usv_rov_sim/worlds/sydney_regatta.asset.json"


def digest(path):
    with path.open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def validate(model_dir, manifest):
    """Check every pinned file, including the meshes and textures, offline."""
    if not model_dir.is_dir() or model_dir.is_symlink():
        return False, "model directory is missing or is a symlink"
    for relative, expected in manifest["files"].items():
        path = model_dir / relative
        if path.is_symlink() or not path.is_file():
            return False, f"missing regular file: {relative}"
        if not path.resolve().is_relative_to(model_dir.resolve()):
            return False, f"resource escapes model directory: {relative}"
        if digest(path) != expected:
            return False, f"checksum mismatch: {relative}"
    return True, "all model, mesh and texture checksums match"


def download(url, archive, expected_bytes):
    request = urllib.request.Request(url, headers={"User-Agent": "usv-rov-ros2-world-preparer/1"})
    with urllib.request.urlopen(request, timeout=60) as response, archive.open("wb") as output:
        total = 0
        next_progress = 0
        while chunk := response.read(1024 * 1024):
            total += len(chunk)
            if total > expected_bytes:
                raise RuntimeError("Fuel archive exceeds the pinned size")
            output.write(chunk)
            progress = total * 100 // expected_bytes
            if progress >= next_progress:
                print(f"Downloading Sydney Regatta: {progress}%", file=sys.stderr, flush=True)
                next_progress = progress + 10
    if total != expected_bytes:
        raise RuntimeError(f"Fuel returned {total} bytes; expected {expected_bytes}. Retry when the network is available.")


def extract(archive, destination, manifest):
    """Extract only the exact known version, with path and symlink checks."""
    expected = set(manifest["files"])
    with zipfile.ZipFile(archive) as source:
        members = source.infolist()
        if sum(entry.file_size for entry in members) != manifest["uncompressed_bytes"]:
            raise RuntimeError("Archive uncompressed size differs from pinned version")
        seen = set()
        for entry in members:
            path = PurePosixPath(entry.filename)
            if path.is_absolute() or ".." in path.parts or "\\" in entry.filename:
                raise RuntimeError("Unsafe archive path")
            mode = entry.external_attr >> 16
            if stat.S_ISLNK(mode):
                raise RuntimeError("Symlinks are not allowed in the terrain archive")
            if entry.is_dir():
                continue
            if entry.filename not in expected or entry.filename in seen:
                raise RuntimeError("Unexpected or duplicate archive member")
            seen.add(entry.filename)
        if seen != expected:
            raise RuntimeError("The archive is missing pinned resources")
        # All paths have been validated, and the archive itself was SHA-256 checked.
        source.extractall(destination)
    valid, reason = validate(destination, manifest)
    if not valid:
        raise RuntimeError(reason)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cache-dir", type=Path, default=REPO / ".world_cache",
                        help="Parent of the sydney_regatta model directory")
    parser.add_argument("--check-only", action="store_true", help="Verify cached files without downloading")
    args = parser.parse_args()
    manifest = json.loads(MANIFEST.read_text())
    cache = args.cache_dir.expanduser().resolve()
    model_dir = cache / manifest["name"]
    valid, reason = validate(model_dir, manifest)
    if valid:
        print(f"Sydney Regatta v3 verified: {model_dir}")
        return 0
    if args.check_only:
        print(f"Sydney Regatta cache is not ready: {reason}", file=sys.stderr)
        return 1

    cache.mkdir(parents=True, exist_ok=True)
    print("Preparing OpenRobotics Sydney Regatta v3 (129 MB, downloaded once).", file=sys.stderr, flush=True)
    with tempfile.TemporaryDirectory(prefix=".sydney-download-", dir=cache) as temp:
        staging = Path(temp)
        archive = staging / "sydney_regatta.zip"
        download(manifest["archive_url"], archive, manifest["archive_bytes"])
        if digest(archive) != manifest["archive_sha256"]:
            raise RuntimeError("Fuel archive SHA-256 differs from pinned version; refusing to install")
        unpacked = staging / manifest["name"]
        unpacked.mkdir()
        extract(archive, unpacked, manifest)
        (unpacked / ".usv_rov_asset.json").write_text(json.dumps({
            "source": manifest["model_url"], "version": manifest["version"],
            "archive_sha256": manifest["archive_sha256"],
            "license": manifest["license"], "owner": manifest["owner"],
        }, indent=2) + "\n")
        if model_dir.exists() or model_dir.is_symlink():
            # Preserve an old or damaged cache instead of deleting user files.
            backup = cache / f"sydney_regatta.previous-{time.time_ns()}"
            model_dir.rename(backup)
            print(f"Preserved previous cache at {backup}", file=sys.stderr)
        os.replace(unpacked, model_dir)
    print(f"Sydney Regatta v3 verified: {model_dir}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, ValueError, zipfile.BadZipFile) as error:
        print(f"World preparation failed: {error}", file=sys.stderr)
        raise SystemExit(1)
