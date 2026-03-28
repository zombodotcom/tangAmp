"""
server.py — API server for tangAmp dashboard.
Serves WAV files, runs synth, runs tests.
Run: python server.py
"""

import http.server
import json
import os
import subprocess
import sys
import threading
import time
from urllib.parse import urlparse, parse_qs

PORT = 8421
PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
RESULTS_FILE = os.path.join(PROJECT_DIR, "test_results.json")

ENV = os.environ.copy()
for p in [r"C:\iverilog\bin", "/c/iverilog/bin"]:
    if p not in ENV.get("PATH", ""):
        ENV["PATH"] = p + os.pathsep + ENV.get("PATH", "")

_lock = threading.Lock()

TESTS = [
    ("Quick Smoke Test", "python quick_test.py"),
    ("Python NR Simulation", "python wdf_triode_sim.py"),
    ("Python WDF Simulation", "python wdf_triode_sim_wdf.py"),
    ("LUT Generation", "python tube_lut_gen.py"),
    ("Verilog Compile + Sim", "iverilog -g2012 -o wdf_sim_v wdf_triode_wdf_tb.v wdf_triode_wdf.v && vvp wdf_sim_v"),
    ("Verilog Analysis", "python analyze_tb.py wdf_tb_output.txt"),
    ("Cross-Validation", "python validate_wdf.py"),
    ("Physics Validation", "python validate_physics.py"),
    ("SPICE Validation", "python validate_spice.py"),
]

IMAGES = [
    "validation_report.png", "validation_spice.png", "validation_datasheet.png",
    "validation_sweep.png", "validation_frequency.png",
    "wdf_sim_wdf_waveforms.png", "wdf_sim_wdf_harmonics.png",
    "waveforms.png", "harmonics.png", "tube_plot.png",
]

DEMOS = {}  # populated on startup


def save_results(results):
    results["timestamp"] = time.strftime("%Y-%m-%d %H:%M:%S")
    with _lock:
        with open(RESULTS_FILE, "w") as f:
            json.dump(results, f, indent=2)


def run_single_test(results, index):
    name, cmd = TESTS[index]
    results["tests"][index]["status"] = "running"
    results["tests"][index]["details"] = {}
    save_results(results)
    t0 = time.time()
    try:
        proc = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=180, cwd=PROJECT_DIR, env=ENV)
        elapsed = time.time() - t0
        results["tests"][index]["time"] = f"{elapsed:.1f}s"
        output = proc.stdout + proc.stderr
        details = {}
        for line in output.split("\n"):
            line = line.strip()
            if "Gain:" in line:
                details["Gain"] = line.split(":", 1)[-1].strip()[:50]
            elif "PASS" in line and "%" in line:
                details["Result"] = line.strip()[:80]
            elif "error" in line.lower() and "%" in line:
                details["Error"] = line.strip()[:80]
        results["tests"][index]["status"] = "pass" if proc.returncode == 0 else "fail"
        if proc.returncode != 0:
            results["tests"][index]["error"] = output.strip()[-300:]
        results["tests"][index]["details"] = details
    except subprocess.TimeoutExpired:
        results["tests"][index]["status"] = "fail"
        results["tests"][index]["error"] = "Timeout"
        results["tests"][index]["time"] = "180s+"
    except Exception as e:
        results["tests"][index]["status"] = "fail"
        results["tests"][index]["error"] = str(e)[:200]
    save_results(results)


def run_all_tests():
    results = {"tests": [{"name": n, "status": "pending", "details": {}} for n, _ in TESTS], "images": []}
    save_results(results)
    for stage in [[0, 1, 2, 3], [4], [5, 6, 7, 8]]:
        threads = [threading.Thread(target=run_single_test, args=(results, i)) for i in stage]
        for t in threads: t.start()
        for t in threads: t.join()
    results["images"] = [img for img in IMAGES if os.path.exists(os.path.join(PROJECT_DIR, img))]
    save_results(results)


def scan_demos():
    """Scan for demo and wav files."""
    demos = []
    # Demo pairs
    demo_dir = os.path.join(PROJECT_DIR, "demos")
    if os.path.isdir(demo_dir):
        for f in sorted(os.listdir(demo_dir)):
            if f.startswith("dry_") and f.endswith(".wav"):
                name = f[4:-4]
                wet_files = [w for w in os.listdir(demo_dir) if w.startswith("wet_" + name.split("_")[0]) and w.endswith(".wav")]
                demos.append({"name": name.replace("_", " ").title(), "dry": f"demos/{f}", "wet": [f"demos/{w}" for w in sorted(wet_files)]})
    # Root wav pairs
    pairs = {}
    for f in sorted(os.listdir(PROJECT_DIR)):
        if f.startswith("wav_dry_") and f.endswith(".wav"):
            key = f[8:-4]
            pairs.setdefault(key, {})["dry"] = f
        elif f.startswith("wav_wet_") and f.endswith(".wav"):
            key = f[8:-4]
            pairs.setdefault(key, {})["wet"] = f
    simple = [{"name": k.replace("_", " ").title(), "dry": v.get("dry", ""), "wet": [v["wet"]] if "wet" in v else []} for k, v in pairs.items()]
    # Tube comparison
    tubes = [{"name": f[9:-4], "file": f} for f in sorted(os.listdir(PROJECT_DIR)) if f.startswith("wav_tube_") and f.endswith(".wav")]
    return {"demos": demos, "simple": simple, "tubes": tubes}


class API(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *a, **kw):
        super().__init__(*a, directory=PROJECT_DIR, **kw)

    def do_GET(self):
        path = urlparse(self.path).path
        if path == "/api/results":
            self._json_file(RESULTS_FILE)
        elif path == "/api/demos":
            self._json(scan_demos())
        elif path == "/api/images":
            imgs = [img for img in IMAGES if os.path.exists(os.path.join(PROJECT_DIR, img))]
            self._json(imgs)
        elif path == "/api/synth":
            self._synth()
        else:
            # Serve static files (wav, png, etc.)
            super().do_GET()

    def do_POST(self):
        path = urlparse(self.path).path
        if path == "/api/run-tests":
            threading.Thread(target=run_all_tests, daemon=True).start()
            self._json({"ok": True})
        else:
            self.send_response(404)
            self.end_headers()

    def _json(self, data):
        body = json.dumps(data).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Content-Length", len(body))
        self.end_headers()
        self.wfile.write(body)

    def _json_file(self, path):
        if os.path.exists(path):
            with open(path, "rb") as f:
                body = f.read()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
        else:
            self._json({"tests": []})

    def _synth(self):
        qs = parse_qs(urlparse(self.path).query)
        tube = qs.get("tube", ["12AX7"])[0]
        freq = qs.get("freq", ["440"])[0]
        drive = qs.get("drive", ["0.5"])[0]
        dur = qs.get("dur", ["2"])[0]
        try:
            proc = subprocess.run(
                f'python synth_api.py "{tube}" {freq} {drive} {dur}',
                shell=True, capture_output=True, text=True, timeout=30, cwd=PROJECT_DIR, env=ENV
            )
            if proc.returncode == 0:
                self._json(json.loads(proc.stdout))
            else:
                self.send_response(500)
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(proc.stderr.encode()[:500])
        except Exception as e:
            self.send_response(500)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(str(e).encode()[:500])

    def end_headers(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        super().end_headers()

    def log_message(self, *a):
        pass


if __name__ == "__main__":
    if not os.path.exists(RESULTS_FILE):
        save_results({"tests": [{"name": n, "status": "pending", "details": {}} for n, _ in TESTS], "images": []})
    server = http.server.HTTPServer(("", PORT), API)
    print(f"API server: http://localhost:{PORT}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        server.shutdown()
