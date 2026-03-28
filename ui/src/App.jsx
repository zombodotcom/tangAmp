import { useState } from 'react'
import Demos from './components/Demos'
import Synth from './components/Synth'
import Tests from './components/Tests'
import Plots from './components/Plots'
import './App.css'

const TABS = ['Demos', 'Synth', 'Tests', 'Plots']

export default function App() {
  const [tab, setTab] = useState('Demos')
  return (
    <div className="app">
      <header>
        <h1><span className="accent">tangAmp</span> WDF Triode</h1>
        <p className="subtitle">FPGA tube amp emulator on Tang Nano 20K — Phase 3: Single 12AX7, 29.3dB gain, 2.19% Verilog error vs Python reference</p>
      </header>
      <nav>
        {TABS.map(t => (
          <button key={t} className={tab === t ? 'active' : ''} onClick={() => setTab(t)}>{t}</button>
        ))}
      </nav>
      <main>
        {tab === 'Demos' && <Demos />}
        {tab === 'Synth' && <Synth />}
        {tab === 'Tests' && <Tests />}
        {tab === 'Plots' && <Plots />}
      </main>
    </div>
  )
}
