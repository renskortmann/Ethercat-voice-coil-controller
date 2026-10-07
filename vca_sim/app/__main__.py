"""Serve the simulator app and open it in the browser: python -m vca_sim.app [--port 8050] [--no-browser]

Run from the repository root (run_simulator.bat does that for you). If the app is already running on
the port, this only opens the browser. Served by waitress when it is installed (it handles the
browser's parallel requests reliably on Windows); otherwise by Flask's development server.
"""

import argparse
import sys
import threading
import time
import urllib.request
import webbrowser
from pathlib import Path

ROOT = Path.cwd()
sys.path.insert(0, str(ROOT))   # controller scripts import vca_sim, the log replay imports vca_log


def is_up(url):
    try:
        urllib.request.urlopen(url, timeout=1)
        return True
    except OSError:
        return False


def open_when_up(url, timeout_s=60):
    """Open the browser as soon as the server answers (in a background thread)."""
    def wait_and_open():
        t_end = time.time() + timeout_s
        while time.time() < t_end:
            if is_up(url):
                webbrowser.open(url)
                return
            time.sleep(0.3)
        print(f"server did not answer within {timeout_s} s; open {url} by hand")
    threading.Thread(target=wait_and_open, daemon=True).start()


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--port", type=int, default=8050)
parser.add_argument("--no-browser", action="store_true", help="do not open the browser")
parser.add_argument("--controllers", default=str(ROOT / "controllers"), help="folder with controller scripts")
args = parser.parse_args()
url = f"http://127.0.0.1:{args.port}/"

if not (ROOT / "vca_sim").is_dir():
    sys.exit(f"run this from the repository root (no vca_sim folder in {ROOT})")

if is_up(url):
    print(f"VCA simulator is already running on {url}; opening it")
    if not args.no_browser:
        webbrowser.open(url)
    sys.exit(0)

from vca_sim.app.main import create_app  # noqa: E402  (imported after the checks: it takes a few seconds)

app = create_app(ROOT, args.controllers)
print(f"VCA simulator on {url}  (Ctrl+C or close this window to stop)", flush=True)
if not args.no_browser:
    open_when_up(url)
try:
    from waitress import serve
except ImportError:
    app.run(host="127.0.0.1", port=args.port, debug=False)
else:
    serve(app.server, host="127.0.0.1", port=args.port, threads=8)
