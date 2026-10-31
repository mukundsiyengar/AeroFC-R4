"""serve.py - the simulator's local web server (like `python -m http.server`, but the browser never keeps old copies:
after you rebuild the firmware or update the files, a page reload always gets the new ones).

    python tools/serve.py [port] [url]    (START_SIMULATOR.bat does this; default port 8766; url = page to open)
"""
import functools
import http.server
import pathlib
import subprocess
import sys
import webbrowser


class NoCacheHandler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def log_message(self, *args):
        pass


class Server(http.server.ThreadingHTTPServer):
    allow_reuse_address = False      # on Windows "reuse" would let a second copy share the port


def open_page(url):
    """Edge or Chrome is needed for "Connect Arduino" (Web Serial), so Edge first."""
    if url and subprocess.call(f'start "" msedge "{url}"', shell=True, stderr=subprocess.DEVNULL) != 0:
        webbrowser.open(url)


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8766
    url = sys.argv[2] if len(sys.argv) > 2 else ""
    root = pathlib.Path(__file__).resolve().parent.parent
    handler = functools.partial(NoCacheHandler, directory=str(root))
    try:
        srv = Server(("127.0.0.1", port), handler)
    except OSError:
        print(f"Port {port} is busy: the simulator server is probably already running (that is fine).", flush=True)
        open_page(url)
        return
    with srv:
        print(f"AeroFC simulator on http://localhost:{port}/simulator/index.html  (close this window to stop)", flush=True)
        open_page(url)      # the server already listens here, so the page can't arrive too early
        srv.serve_forever()


if __name__ == "__main__":
    main()
