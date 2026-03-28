"""
dashboard.py — Live test dashboard for tangAmp.
Run: python dashboard.py
Opens http://localhost:8420 with auto-refreshing test results.
"""

import http.server
import json
import os
import subprocess
import sys
import threading
import time
import webbrowser

PORT = 8420
PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
RESULTS_FILE = os.path.join(PROJECT_DIR, "test_results.json")

# Path setup for iverilog
ENV = os.environ.copy()
for p in [r"C:\iverilog\bin", "/c/iverilog/bin"]:
    if p not in ENV.get("PATH", ""):
        ENV["PATH"] = p + os.pathsep + ENV.get("PATH", "")

HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>tangAmp Test Dashboard</title>
<style>
  * { margin: 0; padding: 0; box-sizing: border-box; }
  body { background: #0d1117; color: #c9d1d9; font-family: 'Consolas', 'Fira Code', monospace; padding: 24px; }
  h1 { color: #58a6ff; margin-bottom: 8px; font-size: 24px; }
  .subtitle { color: #8b949e; margin-bottom: 24px; font-size: 14px; }
  .grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(340px, 1fr)); gap: 16px; }
  .card {
    background: #161b22; border: 1px solid #30363d; border-radius: 8px;
    padding: 16px; position: relative; overflow: hidden;
  }
  .card.running { border-color: #d29922; }
  .card.pass { border-color: #3fb950; }
  .card.fail { border-color: #f85149; }
  .card.pending { border-color: #30363d; }
  .card-header { display: flex; justify-content: space-between; align-items: center; margin-bottom: 8px; }
  .card-title { font-size: 14px; font-weight: 600; color: #e6edf3; }
  .badge {
    font-size: 11px; padding: 2px 8px; border-radius: 12px; font-weight: 600; text-transform: uppercase;
  }
  .badge.running { background: #d29922; color: #0d1117; }
  .badge.pass { background: #238636; color: #fff; }
  .badge.fail { background: #da3633; color: #fff; }
  .badge.pending { background: #30363d; color: #8b949e; }
  .card-body { font-size: 12px; color: #8b949e; line-height: 1.6; }
  .card-body .value { color: #c9d1d9; font-weight: 600; }
  .card-body .highlight { color: #58a6ff; }
  .timer { font-size: 11px; color: #8b949e; }
  .progress-bar {
    position: absolute; bottom: 0; left: 0; height: 3px; background: #d29922;
    animation: indeterminate 1.5s infinite;
  }
  .card.pass .progress-bar, .card.fail .progress-bar, .card.pending .progress-bar { display: none; }
  @keyframes indeterminate {
    0% { width: 0%; left: 0; }
    50% { width: 40%; left: 30%; }
    100% { width: 0%; left: 100%; }
  }
  .summary {
    margin-top: 24px; padding: 16px; background: #161b22; border: 1px solid #30363d;
    border-radius: 8px; font-size: 14px; text-align: center;
  }
  .summary.all-pass { border-color: #3fb950; color: #3fb950; }
  .summary.has-fail { border-color: #f85149; color: #f85149; }
  .summary.in-progress { border-color: #d29922; color: #d29922; }
  .refresh-note { color: #484f58; font-size: 11px; margin-top: 16px; text-align: center; }
  .spinner { display: inline-block; animation: spin 1s linear infinite; }
  @keyframes spin { 100% { transform: rotate(360deg); } }

  /* Images */
  .images { margin-top: 24px; }
  .images h2 { color: #58a6ff; font-size: 18px; margin-bottom: 12px; }
  .img-grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(400px, 1fr)); gap: 12px; }
  .img-grid img {
    width: 100%; border-radius: 6px; border: 1px solid #30363d;
    cursor: pointer; transition: transform 0.2s;
  }
  .img-grid img:hover { transform: scale(1.02); }
</style>
</head>
<body>
<h1>tangAmp Test Dashboard</h1>
<div class="subtitle" id="status">Loading...</div>
<div class="grid" id="cards"></div>
<div class="summary in-progress" id="summary">Waiting for results...</div>
<div class="images" id="images-section" style="display:none">
  <h2>Generated Plots</h2>
  <div class="img-grid" id="img-grid"></div>
</div>
<div class="refresh-note">Auto-refreshes every 2s &middot; <a href="#" onclick="runTests()" style="color:#58a6ff">Re-run all tests</a></div>

<script>
function renderResults(data) {
  const cards = document.getElementById('cards');
  const summary = document.getElementById('summary');
  const status = document.getElementById('status');

  if (!data || !data.tests) {
    status.textContent = 'No results yet...';
    return;
  }

  status.textContent = 'Last updated: ' + (data.timestamp || 'unknown');

  let html = '';
  let pass = 0, fail = 0, running = 0, pending = 0;

  for (const test of data.tests) {
    const s = test.status || 'pending';
    if (s === 'pass') pass++;
    else if (s === 'fail') fail++;
    else if (s === 'running') running++;
    else pending++;

    html += `<div class="card ${s}">`;
    html += `<div class="card-header">`;
    html += `<span class="card-title">${test.name}</span>`;
    html += `<span class="badge ${s}">${s === 'running' ? '<span class="spinner">&#9881;</span> ' : ''}${s}</span>`;
    html += `</div>`;
    html += `<div class="card-body">`;
    if (test.details) {
      for (const [k, v] of Object.entries(test.details)) {
        html += `<div>${k}: <span class="value">${v}</span></div>`;
      }
    }
    if (test.time) html += `<div class="timer">${test.time}</div>`;
    if (test.error) html += `<div style="color:#f85149;margin-top:4px">${test.error}</div>`;
    html += `</div>`;
    if (s === 'running') html += `<div class="progress-bar"></div>`;
    html += `</div>`;
  }
  cards.innerHTML = html;

  if (running > 0) {
    summary.className = 'summary in-progress';
    summary.textContent = `Running... ${pass} passed, ${fail} failed, ${running} running, ${pending} pending`;
  } else if (fail > 0) {
    summary.className = 'summary has-fail';
    summary.textContent = `${fail} FAILED, ${pass} passed out of ${pass + fail + pending} tests`;
  } else if (pass > 0) {
    summary.className = 'summary all-pass';
    summary.textContent = `ALL ${pass} TESTS PASSED`;
  }

  // Show images if any
  const imgSection = document.getElementById('images-section');
  const imgGrid = document.getElementById('img-grid');
  if (data.images && data.images.length > 0) {
    imgSection.style.display = 'block';
    imgGrid.innerHTML = data.images.map(img =>
      `<img src="/${img}?t=${Date.now()}" alt="${img}" onclick="window.open('/${img}','_blank')">`
    ).join('');
  }
}

async function poll() {
  try {
    const resp = await fetch('/test_results.json?t=' + Date.now());
    if (resp.ok) {
      const data = await resp.json();
      renderResults(data);
    }
  } catch(e) {}
  setTimeout(poll, 2000);
}

async function runTests() {
  try { await fetch('/run_tests', {method: 'POST'}); } catch(e) {}
}

poll();
</script>
</body>
</html>"""


def run_single_test(name, cmd, results, index):
    """Run a single test and update results dict."""
    results["tests"][index]["status"] = "running"
    results["tests"][index]["details"] = {}
    save_results(results)

    t0 = time.time()
    try:
        proc = subprocess.run(
            cmd, shell=True, capture_output=True, text=True,
            timeout=120, cwd=PROJECT_DIR, env=ENV
        )
        elapsed = time.time() - t0
        results["tests"][index]["time"] = f"{elapsed:.1f}s"

        output = proc.stdout + proc.stderr
        # Parse key metrics from output
        details = {}
        for line in output.split("\n"):
            line = line.strip()
            if "Gain:" in line or "gain:" in line:
                details["Gain"] = line.split(":", 1)[-1].strip()[:40]
            elif "PASS" in line and "%" in line:
                details["Result"] = line.strip()[:60]
            elif "Vplate" in line and "=" in line:
                details["Vplate"] = line.strip()[:60]
            elif "Ip" in line and "mA" in line and "=" in line:
                details["Ip"] = line.strip()[:60]
            elif "error" in line.lower() and "%" in line:
                details["Error"] = line.strip()[:60]
            elif "rms" in line.lower() and "=" in line:
                details["RMS"] = line.strip()[:60]

        if proc.returncode == 0:
            results["tests"][index]["status"] = "pass"
        else:
            results["tests"][index]["status"] = "fail"
            results["tests"][index]["error"] = output[-200:] if output else "Exit code " + str(proc.returncode)

        results["tests"][index]["details"] = details

    except subprocess.TimeoutExpired:
        results["tests"][index]["status"] = "fail"
        results["tests"][index]["error"] = "Timeout (120s)"
        results["tests"][index]["time"] = "120s+"
    except Exception as e:
        results["tests"][index]["status"] = "fail"
        results["tests"][index]["error"] = str(e)[:200]

    save_results(results)


def save_results(results):
    results["timestamp"] = time.strftime("%H:%M:%S")
    with open(RESULTS_FILE, "w") as f:
        json.dump(results, f, indent=2)


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
    "validation_report.png",
    "validation_datasheet.png",
    "validation_sweep.png",
    "validation_frequency.png",
    "wdf_sim_wdf_waveforms.png",
    "wdf_sim_wdf_harmonics.png",
    "waveforms.png",
    "harmonics.png",
    "tube_plot.png",
    "validation_spice.png",
]


def run_all_tests():
    results = {
        "tests": [{"name": name, "status": "pending", "details": {}} for name, _ in TESTS],
        "images": [img for img in IMAGES if os.path.exists(os.path.join(PROJECT_DIR, img))],
        "timestamp": time.strftime("%H:%M:%S"),
    }
    save_results(results)

    # Stage 1: parallel independent tests
    stage1 = [0, 1, 2, 3]  # smoke, NR sim, WDF sim, LUT gen
    threads = []
    for i in stage1:
        t = threading.Thread(target=run_single_test, args=(TESTS[i][0], TESTS[i][1], results, i))
        t.start()
        threads.append(t)
    for t in threads:
        t.join()

    # Stage 2: Verilog (needs LUTs)
    run_single_test(TESTS[4][0], TESTS[4][1], results, 4)

    # Stage 3: parallel post-processing
    stage3 = [5, 6, 7, 8]  # analysis, validation, physics, spice
    threads = []
    for i in stage3:
        t = threading.Thread(target=run_single_test, args=(TESTS[i][0], TESTS[i][1], results, i))
        t.start()
        threads.append(t)
    for t in threads:
        t.join()

    # Update images list
    results["images"] = [img for img in IMAGES if os.path.exists(os.path.join(PROJECT_DIR, img))]
    save_results(results)


class DashboardHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=PROJECT_DIR, **kwargs)

    def do_GET(self):
        if self.path == "/" or self.path == "/index.html":
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(HTML.encode())
        elif self.path.startswith("/test_results.json"):
            if os.path.exists(RESULTS_FILE):
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                with open(RESULTS_FILE, "rb") as f:
                    self.wfile.write(f.read())
            else:
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(b'{"tests":[]}')
        else:
            super().do_GET()

    def do_POST(self):
        if self.path == "/run_tests":
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b"OK")
            threading.Thread(target=run_all_tests, daemon=True).start()
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        pass  # quiet


def main():
    # Run tests immediately in background
    threading.Thread(target=run_all_tests, daemon=True).start()

    server = http.server.HTTPServer(("", PORT), DashboardHandler)
    url = f"http://localhost:{PORT}"
    print(f"Dashboard running at {url}")
    print(f"Press Ctrl+C to stop\n")

    # Open browser
    webbrowser.open(url)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping dashboard.")
        server.shutdown()


if __name__ == "__main__":
    main()
