#!/usr/bin/env python3
"""
CamFlight — local server for the drone flight analysis tool.

Serves the web app and streams the video files with HTTP Range support,
which the browser needs in order to seek and scrub.

Run:    python3 server.py                     (port 8765)
        python3 server.py 9000                (different port)
        python3 server.py 8765 /path/to/media (explicit media folder)
Then:   open http://localhost:8765

The media folder (DJI MP4/LRF/SRT files) is looked up in this order:
  1. the second command-line argument
  2. the CAMFLIGHT_MEDIA environment variable
  3. a "media_path.txt" file next to this script, containing one path
  4. ./media   (created on first run if nothing else is found)
"""
import json
import os
import re
import socketserver
import sys
from http.server import BaseHTTPRequestHandler
from pathlib import Path
from urllib.parse import unquote, urlparse

TOOL_DIR = Path(__file__).resolve().parent
PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8765

CTYPES = {
    ".html": "text/html; charset=utf-8",
    ".js": "text/javascript; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".json": "application/json; charset=utf-8",
    ".srt": "text/plain; charset=utf-8",
    ".mp4": "video/mp4",
    ".lrf": "video/mp4",  # the DJI low-res proxy is an mp4 container
    ".png": "image/png",
    ".svg": "image/svg+xml",
    ".ico": "image/x-icon",
}

CHUNK = 1024 * 512


def resolve_media_dir() -> Path:
    """Find the folder holding the drone clips (see the module docstring)."""
    candidates = []
    if len(sys.argv) > 2:
        candidates.append(Path(sys.argv[2]).expanduser())
    if os.environ.get("CAMFLIGHT_MEDIA"):
        candidates.append(Path(os.environ["CAMFLIGHT_MEDIA"]).expanduser())
    cfg = TOOL_DIR / "media_path.txt"
    if cfg.is_file():
        line = cfg.read_text(encoding="utf-8").strip()
        if line:
            candidates.append(Path(line).expanduser())
    candidates.append(TOOL_DIR / "media")

    for c in candidates:
        if c.is_dir():
            return c.resolve()
    fallback = TOOL_DIR / "media"
    fallback.mkdir(exist_ok=True)
    return fallback


MEDIA_DIR = resolve_media_dir()


def clip_meta(stem: str):
    """DJI_20250704120000_0001_D -> recording date, time and clip number."""
    m = re.match(r"DJI_(\d{4})(\d{2})(\d{2})(\d{2})(\d{2})(\d{2})_(\d+)", stem)
    if not m:
        return {"date": None, "time": None, "num": stem}
    y, mo, d, h, mi, s, num = m.groups()
    return {"date": f"{y}-{mo}-{d}", "time": f"{h}:{mi}:{s}", "num": num}


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, format, *args):  # keep the console quiet
        pass

    def do_GET(self):
        try:
            self.route()
        except (BrokenPipeError, ConnectionResetError):
            pass  # the browser dropped the video stream (normal while scrubbing)

    def do_HEAD(self):
        self.do_GET()

    def route(self):
        path = unquote(urlparse(self.path).path)
        if path in ("/", "/index.html"):
            return self.send_file(TOOL_DIR / "index.html")
        if path == "/api/clips":
            return self.api_clips()
        if path.startswith("/media/"):
            name = Path(path[len("/media/"):]).name  # no path traversal
            return self.send_file(MEDIA_DIR / name)
        # static files from the tool folder
        return self.send_file(TOOL_DIR / Path(path.lstrip("/")).name)

    def api_clips(self):
        clips = []
        srt_files = sorted(MEDIA_DIR.glob("*.SRT")) + sorted(MEDIA_DIR.glob("*.srt"))
        for srt in srt_files:
            stem = srt.stem
            mp4 = next((p for p in (MEDIA_DIR / f"{stem}.MP4", MEDIA_DIR / f"{stem}.mp4") if p.exists()), None)
            lrf = next((p for p in (MEDIA_DIR / f"{stem}.LRF", MEDIA_DIR / f"{stem}.lrf") if p.exists()), None)
            if not mp4 and not lrf:
                continue
            clips.append({
                "id": stem,
                "srt": f"/media/{srt.name}",
                "mp4": f"/media/{mp4.name}" if mp4 else None,
                "lrf": f"/media/{lrf.name}" if lrf else None,
                "mp4Size": mp4.stat().st_size if mp4 else 0,
                "lrfSize": lrf.stat().st_size if lrf else 0,
                **clip_meta(stem),
            })
        body = json.dumps({"clips": clips, "mediaDir": str(MEDIA_DIR)}).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        if self.command != "HEAD":
            self.wfile.write(body)

    def send_file(self, fp: Path):
        if not fp.is_file():
            self.send_error(404, "Not found")
            return
        size = fp.stat().st_size
        ctype = CTYPES.get(fp.suffix.lower(), "application/octet-stream")
        start, end = 0, size - 1
        status = 200

        rng = self.headers.get("Range")
        if rng:
            m = re.match(r"bytes=(\d*)-(\d*)", rng)
            if m:
                if m.group(1):
                    start = int(m.group(1))
                    if m.group(2):
                        end = min(int(m.group(2)), size - 1)
                elif m.group(2):  # suffix range: the last N bytes
                    start = max(0, size - int(m.group(2)))
                if start > end or start >= size:
                    self.send_response(416)
                    self.send_header("Content-Range", f"bytes */{size}")
                    self.send_header("Content-Length", "0")
                    self.end_headers()
                    return
                status = 206

        length = end - start + 1
        self.send_response(status)
        self.send_header("Content-Type", ctype)
        self.send_header("Accept-Ranges", "bytes")
        self.send_header("Content-Length", str(length))
        if fp.suffix.lower() in (".html", ".js", ".css"):
            # never cache the app files: avoids confusing stale pages
            self.send_header("Cache-Control", "no-cache")
        if status == 206:
            self.send_header("Content-Range", f"bytes {start}-{end}/{size}")
        self.end_headers()
        if self.command == "HEAD":
            return
        with open(fp, "rb") as f:
            f.seek(start)
            remaining = length
            while remaining > 0:
                data = f.read(min(CHUNK, remaining))
                if not data:
                    break
                self.wfile.write(data)
                remaining -= len(data)


class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def handle_error(self, request, client_address):
        # the browser drops video connections constantly while scrubbing;
        # those are not errors worth printing a traceback for
        if isinstance(sys.exc_info()[1], (BrokenPipeError, ConnectionResetError)):
            return
        super().handle_error(request, client_address)


if __name__ == "__main__":
    print(f"CamFlight running at  http://localhost:{PORT}")
    print(f"Media folder: {MEDIA_DIR}")
    if not list(MEDIA_DIR.glob("*.SRT")) and not list(MEDIA_DIR.glob("*.srt")):
        print("  (no clips yet — put the drone MP4/LRF/SRT files in this folder)")
    Server(("127.0.0.1", PORT), Handler).serve_forever()
