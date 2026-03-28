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
<title>tangAmp — WDF Triode Dashboard</title>
<style>
:root {
  --bg: #0d1117; --bg2: #161b22; --border: #30363d;
  --text: #c9d1d9; --text2: #8b949e; --text3: #484f58;
  --blue: #58a6ff; --green: #3fb950; --red: #f85149; --yellow: #d29922;
  --green-bg: #238636; --red-bg: #da3633;
}
* { margin: 0; padding: 0; box-sizing: border-box; }
body { background: var(--bg); color: var(--text); font-family: -apple-system, 'Segoe UI', Helvetica, Arial, sans-serif; }

/* Header */
.header {
  background: var(--bg2); border-bottom: 1px solid var(--border);
  padding: 20px 32px; display: flex; align-items: center; justify-content: space-between;
}
.header h1 { font-size: 20px; color: var(--text); font-weight: 600; }
.header h1 span { color: var(--blue); }
.header-right { display: flex; align-items: center; gap: 16px; }
.btn {
  background: var(--bg); border: 1px solid var(--border); color: var(--text);
  padding: 6px 16px; border-radius: 6px; font-size: 13px; cursor: pointer;
  font-family: inherit; transition: border-color 0.2s;
}
.btn:hover { border-color: var(--text2); }
.timestamp { font-size: 12px; color: var(--text3); font-family: 'Consolas', monospace; }

/* Main content */
.main { max-width: 1200px; margin: 0 auto; padding: 24px 32px; }

/* Summary bar */
.summary-bar {
  padding: 12px 20px; border-radius: 8px; margin-bottom: 24px;
  font-size: 14px; font-weight: 600; text-align: center;
  border: 1px solid var(--border); background: var(--bg2);
  transition: color 0.3s, border-color 0.3s;
}
.summary-bar.running { color: var(--yellow); border-color: var(--yellow); }
.summary-bar.pass { color: var(--green); border-color: var(--green); }
.summary-bar.fail { color: var(--red); border-color: var(--red); }

/* Test grid */
.test-grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(350px, 1fr)); gap: 12px; }

.test-card {
  background: var(--bg2); border: 1px solid var(--border); border-radius: 8px;
  padding: 14px 16px; transition: border-color 0.3s;
  position: relative; overflow: hidden; min-height: 70px;
}
.test-card[data-status="pass"]    { border-left: 3px solid var(--green); }
.test-card[data-status="fail"]    { border-left: 3px solid var(--red); }
.test-card[data-status="running"] { border-left: 3px solid var(--yellow); }
.test-card[data-status="pending"] { border-left: 3px solid var(--border); opacity: 0.6; }

.test-card .row { display: flex; justify-content: space-between; align-items: center; }
.test-card .name { font-size: 13px; font-weight: 600; color: var(--text); }
.test-card .badge {
  font-size: 10px; padding: 2px 8px; border-radius: 10px;
  font-weight: 700; text-transform: uppercase; letter-spacing: 0.5px;
}
.badge-pass    { background: var(--green-bg); color: #fff; }
.badge-fail    { background: var(--red-bg); color: #fff; }
.badge-running { background: var(--yellow); color: #000; }
.badge-pending { background: var(--border); color: var(--text3); }

.test-card .details { font-size: 11px; color: var(--text2); margin-top: 6px; font-family: 'Consolas', monospace; line-height: 1.5; }
.test-card .details .val { color: var(--text); }
.test-card .timer { font-size: 11px; color: var(--text3); margin-top: 4px; }
.test-card .err { font-size: 11px; color: var(--red); margin-top: 4px; word-break: break-all; }

/* Progress shimmer for running */
.test-card[data-status="running"]::after {
  content: ''; position: absolute; bottom: 0; left: 0; right: 0; height: 2px;
  background: linear-gradient(90deg, transparent, var(--yellow), transparent);
  animation: shimmer 1.5s ease-in-out infinite;
}
@keyframes shimmer { 0%{transform:translateX(-100%)} 100%{transform:translateX(100%)} }

/* Roadmap section */
.section-title { font-size: 16px; color: var(--blue); margin: 32px 0 12px; font-weight: 600; }

.roadmap { display: grid; grid-template-columns: repeat(auto-fill, minmax(250px, 1fr)); gap: 10px; }
.roadmap-item {
  background: var(--bg2); border: 1px solid var(--border); border-radius: 6px;
  padding: 12px 14px; font-size: 12px;
}
.roadmap-item .phase { color: var(--blue); font-weight: 600; font-size: 11px; text-transform: uppercase; margin-bottom: 4px; }
.roadmap-item .desc { color: var(--text2); }
.roadmap-item.current { border-color: var(--green); }
.roadmap-item.current .phase { color: var(--green); }

/* Tabs */
.tabs { display: flex; gap: 4px; margin-bottom: 24px; border-bottom: 1px solid var(--border); padding-bottom: 0; }
.tab {
  background: none; border: none; border-bottom: 2px solid transparent;
  color: var(--text2); padding: 10px 20px; font-size: 14px; font-weight: 500;
  cursor: pointer; font-family: inherit; transition: color 0.2s, border-color 0.2s;
}
.tab:hover { color: var(--text); }
.tab.active { color: var(--blue); border-bottom-color: var(--blue); }
.tab-content { animation: fadeIn 0.2s ease; }
@keyframes fadeIn { from{opacity:0} to{opacity:1} }

/* Audio section */
.audio-section { margin-bottom: 8px; }
.audio-row { display: grid; grid-template-columns: repeat(auto-fill, minmax(400px, 1fr)); gap: 12px; margin-bottom: 12px; }
.audio-card {
  background: var(--bg2); border: 1px solid var(--border); border-radius: 8px; padding: 14px 16px;
}
.audio-label { font-size: 13px; font-weight: 600; color: var(--text); margin-bottom: 10px; }
.ab-row { display: flex; flex-direction: column; gap: 8px; }
.ab-tag {
  display: inline-block; font-size: 10px; font-weight: 700; padding: 1px 6px;
  border-radius: 4px; margin-bottom: 4px; text-transform: uppercase; letter-spacing: 0.5px;
}
.ab-tag.dry { background: var(--border); color: var(--text2); }
.ab-tag.wet { background: #1a3a1a; color: var(--green); }
audio { width: 100%; height: 32px; }

/* Tube comparison */
.tube-grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(200px, 1fr)); gap: 10px; }
.tube-card {
  background: var(--bg2); border: 1px solid var(--border); border-radius: 8px; padding: 12px 14px;
}
.tube-name { font-size: 16px; font-weight: 700; color: var(--blue); }
.tube-desc { font-size: 11px; color: var(--text2); margin: 2px 0 8px; }
.tube-card audio { width: 100%; height: 32px; }

/* Synth panel */
.synth-panel {
  background: var(--bg2); border: 1px solid var(--border); border-radius: 8px;
  padding: 20px; display: grid; grid-template-columns: 1fr 1fr; gap: 20px;
}
.synth-controls { display: flex; flex-direction: column; gap: 12px; }
.ctrl label { font-size: 12px; color: var(--text2); display: block; margin-bottom: 4px; }
.ctrl select, .ctrl input[type=range] { width: 100%; }
.ctrl select {
  background: var(--bg); color: var(--text); border: 1px solid var(--border);
  padding: 6px 8px; border-radius: 4px; font-size: 13px; font-family: inherit;
}
.ctrl input[type=range] { accent-color: var(--blue); }
.synth-go { margin-top: 8px; background: var(--blue); color: #fff; border: none; padding: 8px 20px; font-weight: 600; }
.synth-go:hover { opacity: 0.9; }
.synth-go:disabled { opacity: 0.4; cursor: wait; }
.synth-output { display: flex; flex-direction: column; justify-content: center; align-items: center; }
.synth-placeholder { color: var(--text3); font-size: 13px; }

/* Images */
.plot-grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(380px, 1fr)); gap: 10px; margin-top: 12px; }
.plot-grid img {
  width: 100%; border-radius: 6px; border: 1px solid var(--border);
  cursor: pointer; transition: opacity 0.2s;
}
.plot-grid img:hover { opacity: 0.85; }
</style>
</head>
<body>

<div class="header">
  <h1><span>tangAmp</span> WDF Triode Dashboard</h1>
  <div class="header-right">
    <span class="timestamp" id="ts"></span>
    <button class="btn" onclick="rerun()">Re-run Tests</button>
  </div>
</div>

<div class="main">
  <!-- TAB NAV -->
  <div class="tabs">
    <button class="tab active" onclick="showTab('audio')">Audio Demos</button>
    <button class="tab" onclick="showTab('synth')">Live Synth</button>
    <button class="tab" onclick="showTab('tests')">Tests</button>
    <button class="tab" onclick="showTab('plots')">Plots</button>
  </div>

  <!-- AUDIO TAB -->
  <div class="tab-content" id="tab-audio">

  <!-- DEMOS -->
  <div class="section-title">Demos — Guitar-Like Signals Through Tubes</div>
  <div class="audio-section">
    <div class="audio-row">
      <div class="audio-card">
        <div class="audio-label">Blues Riff in E (12AX7)</div>
        <div class="ab-row">
          <div><div class="ab-tag dry">DRY</div><audio controls preload="none" src="/demos/dry_blues_riff.wav"></audio></div>
          <div><div class="ab-tag wet">WET 12AX7</div><audio controls preload="none" src="/demos/wet_blues_riff_12AX7.wav"></audio></div>
          <div><div class="ab-tag wet">WET EL34</div><audio controls preload="none" src="/demos/wet_blues_riff_EL34.wav"></audio></div>
        </div>
      </div>
      <div class="audio-card">
        <div class="audio-label">Chord Progression E-A-B-E</div>
        <div class="ab-row">
          <div><div class="ab-tag dry">DRY</div><audio controls preload="none" src="/demos/dry_chord_progression.wav"></audio></div>
          <div><div class="ab-tag wet">WET CLEAN</div><audio controls preload="none" src="/demos/wet_chord_progression_12AX7.wav"></audio></div>
          <div><div class="ab-tag wet">WET CRANKED</div><audio controls preload="none" src="/demos/wet_chord_progression_cranked.wav"></audio></div>
        </div>
      </div>
    </div>
    <div class="audio-row">
      <div class="audio-card">
        <div class="audio-label">Drive Sweep: Clean -> Full Crunch</div>
        <div class="ab-row">
          <div><div class="ab-tag dry">DRY</div><audio controls preload="none" src="/demos/dry_drive_sweep.wav"></audio></div>
          <div><div class="ab-tag wet">WET</div><audio controls preload="none" src="/demos/wet_drive_sweep.wav"></audio></div>
        </div>
      </div>
      <div class="audio-card">
        <div class="audio-label">Staccato Hits (Palm Mute Style)</div>
        <div class="ab-row">
          <div><div class="ab-tag dry">DRY</div><audio controls preload="none" src="/demos/dry_staccato.wav"></audio></div>
          <div><div class="ab-tag wet">WET</div><audio controls preload="none" src="/demos/wet_staccato.wav"></audio></div>
        </div>
      </div>
    </div>
    <div class="audio-row">
      <div class="audio-card">
        <div class="audio-label">Frequency Sweep 80Hz -> 2kHz</div>
        <div class="ab-row">
          <div><div class="ab-tag dry">DRY</div><audio controls preload="none" src="/demos/dry_freq_sweep.wav"></audio></div>
          <div><div class="ab-tag wet">WET</div><audio controls preload="none" src="/demos/wet_freq_sweep.wav"></audio></div>
        </div>
      </div>
      <div class="audio-card">
        <div class="audio-label">Double Stop with Bend</div>
        <div class="ab-row">
          <div><div class="ab-tag dry">DRY</div><audio controls preload="none" src="/demos/dry_doublestop.wav"></audio></div>
          <div><div class="ab-tag wet">WET</div><audio controls preload="none" src="/demos/wet_doublestop.wav"></audio></div>
        </div>
      </div>
    </div>
    <div class="section-title" style="font-size:14px;margin:12px 0 8px">Same Riff Through All 5 Tubes</div>
    <div class="tube-grid">
      <div class="tube-card">
        <div class="tube-name">DRY</div>
        <div class="tube-desc">Unprocessed input</div>
        <audio controls preload="none" src="/demos/dry_riff_comparison.wav"></audio>
      </div>
      <div class="tube-card">
        <div class="tube-name">12AX7</div>
        <div class="tube-desc">High gain — most distortion</div>
        <audio controls preload="none" src="/demos/wet_riff_12AX7.wav"></audio>
      </div>
      <div class="tube-card">
        <div class="tube-name">12AU7</div>
        <div class="tube-desc">Low gain — clean/warm</div>
        <audio controls preload="none" src="/demos/wet_riff_12AU7.wav"></audio>
      </div>
      <div class="tube-card">
        <div class="tube-name">6SL7</div>
        <div class="tube-desc">Medium — vintage character</div>
        <audio controls preload="none" src="/demos/wet_riff_6SL7.wav"></audio>
      </div>
      <div class="tube-card">
        <div class="tube-name">EL34</div>
        <div class="tube-desc">Power tube — Marshall crunch</div>
        <audio controls preload="none" src="/demos/wet_riff_EL34.wav"></audio>
      </div>
      <div class="tube-card">
        <div class="tube-name">6L6</div>
        <div class="tube-desc">Power tube — Fender clean</div>
        <audio controls preload="none" src="/demos/wet_riff_6L6.wav"></audio>
      </div>
    </div>
  </div>

  <!-- SIMPLE A/B SECTION -->
  <div class="section-title">Simple Tones — Dry vs Wet (12AX7)</div>
  <div class="audio-section">
    <div class="audio-row">
      <div class="audio-card">
        <div class="audio-label">Clean Sine (0.5V, 440Hz)</div>
        <div class="ab-row">
          <div><div class="ab-tag dry">DRY</div><audio controls preload="none" src="/wav_dry_clean_sine.wav"></audio></div>
          <div><div class="ab-tag wet">WET</div><audio controls preload="none" src="/wav_wet_clean_sine.wav"></audio></div>
        </div>
      </div>
      <div class="audio-card">
        <div class="audio-label">Dirty Sine (2V overdrive)</div>
        <div class="ab-row">
          <div><div class="ab-tag dry">DRY</div><audio controls preload="none" src="/wav_dry_dirty_sine.wav"></audio></div>
          <div><div class="ab-tag wet">WET</div><audio controls preload="none" src="/wav_wet_dirty_sine.wav"></audio></div>
        </div>
      </div>
    </div>
    <div class="audio-row">
      <div class="audio-card">
        <div class="audio-label">E Power Chord</div>
        <div class="ab-row">
          <div><div class="ab-tag dry">DRY</div><audio controls preload="none" src="/wav_dry_power_chord.wav"></audio></div>
          <div><div class="ab-tag wet">WET</div><audio controls preload="none" src="/wav_wet_power_chord.wav"></audio></div>
        </div>
      </div>
      <div class="audio-card">
        <div class="audio-label">Volume Swell (clean → crunch)</div>
        <div class="ab-row">
          <div><div class="ab-tag dry">DRY</div><audio controls preload="none" src="/wav_dry_volume_swell.wav"></audio></div>
          <div><div class="ab-tag wet">WET</div><audio controls preload="none" src="/wav_wet_volume_swell.wav"></audio></div>
        </div>
      </div>
    </div>
  </div>

  <!-- TUBE COMPARISON -->
  <div class="section-title">Tube Comparison — Same Input, Different Character</div>
  <div class="audio-section">
    <div class="tube-grid">
      <div class="tube-card">
        <div class="tube-name">12AX7</div>
        <div class="tube-desc">High gain preamp — Marshall/Fender</div>
        <audio controls preload="none" src="/wav_tube_12AX7.wav"></audio>
      </div>
      <div class="tube-card">
        <div class="tube-name">12AU7</div>
        <div class="tube-desc">Low gain — clean/hi-fi</div>
        <audio controls preload="none" src="/wav_tube_12AU7.wav"></audio>
      </div>
      <div class="tube-card">
        <div class="tube-name">6SL7</div>
        <div class="tube-desc">Medium gain — vintage</div>
        <audio controls preload="none" src="/wav_tube_6SL7.wav"></audio>
      </div>
      <div class="tube-card">
        <div class="tube-name">EL34</div>
        <div class="tube-desc">Power tube — Marshall crunch</div>
        <audio controls preload="none" src="/wav_tube_EL34.wav"></audio>
      </div>
      <div class="tube-card">
        <div class="tube-name">6L6</div>
        <div class="tube-desc">Power tube — Fender clean</div>
        <audio controls preload="none" src="/wav_tube_6L6.wav"></audio>
      </div>
    </div>
  </div>

  </div><!-- end audio tab -->

  <!-- SYNTH TAB -->
  <div class="tab-content" id="tab-synth" style="display:none">
  <div class="section-title">Live Synth — Generate Custom Audio</div>
  <div class="synth-panel">
    <div class="synth-controls">
      <div class="ctrl">
        <label>Tube</label>
        <select id="syn-tube">
          <option value="12AX7" selected>12AX7 (high gain)</option>
          <option value="12AU7">12AU7 (clean)</option>
          <option value="6SL7">6SL7 (vintage)</option>
          <option value="EL34">EL34 (power)</option>
          <option value="6L6">6L6 (Fender)</option>
        </select>
      </div>
      <div class="ctrl">
        <label>Frequency: <span id="syn-freq-val">440</span>Hz</label>
        <input type="range" id="syn-freq" min="80" max="2000" value="440" step="1">
      </div>
      <div class="ctrl">
        <label>Drive: <span id="syn-drive-val">0.5</span>V</label>
        <input type="range" id="syn-drive" min="0.05" max="5" value="0.5" step="0.05">
      </div>
      <div class="ctrl">
        <label>Duration: <span id="syn-dur-val">2</span>s</label>
        <input type="range" id="syn-dur" min="0.5" max="5" value="2" step="0.5">
      </div>
      <button class="btn synth-go" id="syn-btn" onclick="synthGo()">Generate</button>
    </div>
    <div class="synth-output" id="syn-out">
      <div class="synth-placeholder">Adjust parameters and click Generate</div>
    </div>
  </div>

  </div><!-- end synth tab -->

  <!-- TESTS TAB -->
  <div class="tab-content" id="tab-tests" style="display:none">
  <div class="summary-bar" id="summary">Loading...</div>
  <div class="test-grid" id="grid"></div>
  <div class="section-title" style="margin-top:24px">Roadmap</div>
  <div class="roadmap">
    <div class="roadmap-item"><div class="phase">Phase 1</div><div class="desc">Audio I/O (I2S ADC + DAC)</div></div>
    <div class="roadmap-item"><div class="phase">Phase 2</div><div class="desc">Simple nonlinearity (soft clip LUT)</div></div>
    <div class="roadmap-item current"><div class="phase">Phase 3 - current</div><div class="desc">Single WDF triode stage (29.3dB, 2.19% error)</div></div>
    <div class="roadmap-item"><div class="phase">Phase 4</div><div class="desc">Cascaded preamp stages + tone stack</div></div>
    <div class="roadmap-item"><div class="phase">Phase 5</div><div class="desc">Power amp (6L6/EL34/300B) + cabinet IR</div></div>
  </div>
  </div><!-- end tests tab -->

  <!-- PLOTS TAB -->
  <div class="tab-content" id="tab-plots" style="display:none">
  <div class="section-title" id="plots-title">Validation Plots</div>
  <div class="plot-grid" id="plots"></div>
  </div><!-- end plots tab -->

</div>

<script>
function showTab(name){
  document.querySelectorAll('.tab-content').forEach(function(el){el.style.display='none';});
  document.querySelectorAll('.tab').forEach(function(el){el.classList.remove('active');});
  document.getElementById('tab-'+name).style.display='block';
  event.target.classList.add('active');
  // Load plots on first view
  if(name==='plots'&&!imgOk){loadPlots();}
}

const C={};
let imgOk=false, sumTxt='';

function up(d) {
  if(!d||!d.tests) return;
  document.getElementById('ts').textContent=d.timestamp||'';
  const g=document.getElementById('grid');
  let p=0,f=0,r=0,pn=0;
  for(let i=0;i<d.tests.length;i++){
    const t=d.tests[i], s=t.status||'pending', k=JSON.stringify(t);
    if(s==='pass')p++; else if(s==='fail')f++; else if(s==='running')r++; else pn++;
    if(C[i]===k) continue;  // THIS card unchanged, skip DOM touch entirely
    C[i]=k;
    let el=document.getElementById('t'+i);
    if(!el){el=document.createElement('div');el.id='t'+i;el.className='test-card';g.appendChild(el);}
    el.setAttribute('data-status',s);
    let h='<div class="row"><span class="name">'+t.name+'</span><span class="badge badge-'+s+'">'+s+'</span></div>';
    if(t.details){h+='<div class="details">';for(const[a,b]of Object.entries(t.details))h+=a+': <span class="val">'+b+'</span><br>';h+='</div>';}
    if(t.time)h+='<div class="timer">'+t.time+'</div>';
    if(t.error)h+='<div class="err">'+t.error+'</div>';
    el.innerHTML=h;
  }
  const st=''+p+','+f+','+r+','+pn;
  if(st!==sumTxt){
    sumTxt=st;
    const sm=document.getElementById('summary');
    if(r>0){sm.className='summary-bar running';sm.textContent='Running: '+p+' passed, '+r+' running, '+pn+' pending';}
    else if(f>0){sm.className='summary-bar fail';sm.textContent=f+' FAILED, '+p+' passed';}
    else if(p>0){sm.className='summary-bar pass';sm.textContent='ALL '+p+' TESTS PASSED';}
  }
  // Store images list for lazy loading
  if(d.images&&d.images.length) window._plotImages=d.images;
}
async function poll(){try{const r=await fetch('/test_results.json?_='+Date.now());if(r.ok)up(await r.json());}catch(e){}setTimeout(poll,2000);}
function loadPlots(){
  if(imgOk||!window._plotImages) return;
  imgOk=true;
  document.getElementById('plots').innerHTML=window._plotImages.map(function(x){return '<img src="/'+x+'" alt="'+x+'" onclick="window.open(\'/'+x+'\',\'_blank\')">';}).join('');
}
function rerun(){for(const k in C)delete C[k];imgOk=false;sumTxt='';document.getElementById('plots').innerHTML='';document.getElementById('grid').innerHTML='';fetch('/run_tests',{method:'POST'});}

// Slider labels
['freq','drive','dur'].forEach(function(n){
  const el=document.getElementById('syn-'+n);
  if(el)el.oninput=function(){document.getElementById('syn-'+n+'-val').textContent=el.value;};
});

async function synthGo(){
  const btn=document.getElementById('syn-btn');
  const out=document.getElementById('syn-out');
  btn.disabled=true; btn.textContent='Generating...';
  out.innerHTML='<div class="synth-placeholder">Running simulation...</div>';
  const tube=document.getElementById('syn-tube').value;
  const freq=document.getElementById('syn-freq').value;
  const drive=document.getElementById('syn-drive').value;
  const dur=document.getElementById('syn-dur').value;
  try{
    const r=await fetch('/synth?tube='+tube+'&freq='+freq+'&drive='+drive+'&dur='+dur);
    if(!r.ok)throw new Error(await r.text());
    const d=await r.json();
    out.innerHTML=
      '<div style="width:100%">'+
      '<div style="margin-bottom:8px"><span class="ab-tag dry">DRY</span><audio controls src="/'+d.dry+'" style="width:100%;height:32px"></audio></div>'+
      '<div style="margin-bottom:8px"><span class="ab-tag wet">WET</span><audio controls src="/'+d.wet+'" style="width:100%;height:32px"></audio></div>'+
      '<div style="font-size:11px;color:#8b949e;margin-top:4px">Gain: '+d.gain+' | THD: '+d.thd+' | Tube: '+d.tube+'</div>'+
      '</div>';
  }catch(e){out.innerHTML='<div style="color:#f85149">Error: '+e.message+'</div>';}
  btn.disabled=false; btn.textContent='Generate';
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
            timeout=180, cwd=PROJECT_DIR, env=ENV
        )
        elapsed = time.time() - t0
        results["tests"][index]["time"] = f"{elapsed:.1f}s"

        output = proc.stdout + proc.stderr
        details = {}
        for line in output.split("\n"):
            line = line.strip()
            if not line:
                continue
            if "Gain:" in line or "gain:" in line:
                details["Gain"] = line.split(":", 1)[-1].strip()[:50]
            elif "PASS" in line and "%" in line:
                details["Result"] = line.strip()[:80]
            elif "Vplate" in line and "=" in line:
                details["Vplate"] = line.strip()[:60]
            elif "Ip" in line and "mA" in line and "=" in line:
                details["Ip"] = line.strip()[:60]
            elif "error" in line.lower() and "%" in line:
                details["Error"] = line.strip()[:80]

        if proc.returncode == 0:
            results["tests"][index]["status"] = "pass"
        else:
            results["tests"][index]["status"] = "fail"
            results["tests"][index]["error"] = output.strip()[-300:] if output else f"Exit code {proc.returncode}"

        results["tests"][index]["details"] = details

    except subprocess.TimeoutExpired:
        results["tests"][index]["status"] = "fail"
        results["tests"][index]["error"] = "Timeout (180s)"
        results["tests"][index]["time"] = "180s+"
    except Exception as e:
        results["tests"][index]["status"] = "fail"
        results["tests"][index]["error"] = str(e)[:300]

    save_results(results)


_save_lock = threading.Lock()

def save_results(results):
    results["timestamp"] = time.strftime("%Y-%m-%d %H:%M:%S")
    with _save_lock:
        with open(RESULTS_FILE, "w") as f:
            json.dump(results, f, indent=2)


TESTS = [
    ("Quick Smoke Test", "python quick_test.py"),
    ("Python NR Simulation", "python wdf_triode_sim.py"),
    ("Python WDF Simulation", "python wdf_triode_sim_wdf.py"),
    ("LUT Generation", "python tube_lut_gen.py"),
    ("Verilog Compile + Sim", "iverilog -g2012 -o wdf_sim_v ../fpga/wdf_triode_wdf_tb.v ../rtl/wdf_triode_wdf.v && vvp wdf_sim_v"),
    ("Verilog Analysis", "python analyze_tb.py wdf_tb_output.txt"),
    ("Cross-Validation", "python validate_wdf.py"),
    ("Physics Validation", "python validate_physics.py"),
    ("SPICE Validation", "python validate_spice.py"),
]

IMAGES = [
    "validation_report.png",
    "validation_spice.png",
    "validation_datasheet.png",
    "validation_sweep.png",
    "validation_frequency.png",
    "wdf_sim_wdf_waveforms.png",
    "wdf_sim_wdf_harmonics.png",
    "waveforms.png",
    "harmonics.png",
    "tube_plot.png",
]


def run_all_tests():
    results = {
        "tests": [{"name": name, "status": "pending", "details": {}} for name, _ in TESTS],
        "images": [img for img in IMAGES if os.path.exists(os.path.join(PROJECT_DIR, img))],
    }
    save_results(results)

    # Stage 1: parallel (smoke, NR sim, WDF sim, LUT gen)
    threads = []
    for i in [0, 1, 2, 3]:
        t = threading.Thread(target=run_single_test, args=(TESTS[i][0], TESTS[i][1], results, i))
        t.start()
        threads.append(t)
    for t in threads:
        t.join()

    # Stage 2: Verilog (needs LUTs from stage 1)
    run_single_test(TESTS[4][0], TESTS[4][1], results, 4)

    # Stage 3: parallel post-processing
    threads = []
    for i in [5, 6, 7, 8]:
        t = threading.Thread(target=run_single_test, args=(TESTS[i][0], TESTS[i][1], results, i))
        t.start()
        threads.append(t)
    for t in threads:
        t.join()

    results["images"] = [img for img in IMAGES if os.path.exists(os.path.join(PROJECT_DIR, img))]
    save_results(results)


class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=PROJECT_DIR, **kwargs)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            self.wfile.write(HTML.encode())
        elif self.path.startswith("/synth"):
            self._handle_synth()
        elif self.path.startswith("/test_results.json"):
            if os.path.exists(RESULTS_FILE):
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Cache-Control", "no-store")
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

    def _parse_qs(self):
        from urllib.parse import urlparse, parse_qs
        return parse_qs(urlparse(self.path).query)

    def _handle_synth(self):
        """Generate dry/wet WAV pair on the fly."""
        qs = self._parse_qs()
        tube = qs.get("tube", ["12AX7"])[0]
        freq = float(qs.get("freq", ["440"])[0])
        drive = float(qs.get("drive", ["0.5"])[0])
        dur = float(qs.get("dur", ["2"])[0])

        # Run the synth script
        cmd = f'python synth_api.py "{tube}" {freq} {drive} {dur}'
        try:
            proc = subprocess.run(
                cmd, shell=True, capture_output=True, text=True,
                timeout=30, cwd=PROJECT_DIR, env=ENV
            )
            if proc.returncode != 0:
                self.send_response(500)
                self.send_header("Content-Type", "text/plain")
                self.end_headers()
                self.wfile.write(proc.stderr.encode()[:500])
                return

            result = json.loads(proc.stdout)
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps(result).encode())
        except Exception as e:
            self.send_response(500)
            self.send_header("Content-Type", "text/plain")
            self.end_headers()
            self.wfile.write(str(e).encode()[:500])

    def log_message(self, format, *args):
        pass


def main():
    # Don't auto-run tests — user can click "Re-run Tests"
    # Just load existing results if available
    if not os.path.exists(RESULTS_FILE):
        save_results({"tests": [{"name": n, "status": "pending", "details": {}} for n, _ in TESTS], "images": []})

    server = http.server.HTTPServer(("", PORT), Handler)
    url = f"http://localhost:{PORT}"
    print(f"tangAmp Dashboard: {url}", flush=True)
    print("Press Ctrl+C to stop", flush=True)

    if "--no-browser" not in sys.argv:
        try:
            webbrowser.open(url)
        except Exception:
            pass

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.", flush=True)
        server.shutdown()


if __name__ == "__main__":
    main()
