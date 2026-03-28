import { useState } from 'react'

const TUBES = [
  { value: '12AX7', label: '12AX7 — High gain (Marshall/Fender)' },
  { value: '12AU7', label: '12AU7 — Low gain (clean/hi-fi)' },
  { value: '6SL7', label: '6SL7 — Medium (vintage)' },
  { value: 'EL34', label: 'EL34 — Power tube (Marshall crunch)' },
  { value: '6L6', label: '6L6 — Power tube (Fender clean)' },
]

export default function Synth() {
  const [tube, setTube] = useState('12AX7')
  const [freq, setFreq] = useState(440)
  const [drive, setDrive] = useState(0.5)
  const [dur, setDur] = useState(2)
  const [loading, setLoading] = useState(false)
  const [result, setResult] = useState(null)
  const [error, setError] = useState(null)

  async function generate() {
    setLoading(true)
    setError(null)
    try {
      const r = await fetch(`/api/synth?tube=${tube}&freq=${freq}&drive=${drive}&dur=${dur}`)
      if (!r.ok) throw new Error(await r.text())
      setResult(await r.json())
    } catch (e) {
      setError(e.message)
    }
    setLoading(false)
  }

  return (
    <div className="section">
      <div className="section-title">Live Synth</div>
      <div className="section-sub">Pick a tube, set frequency and drive level, generate audio through the WDF simulation in real-time</div>
      <div className="synth">
        <div className="synth-controls">
          <div className="ctrl">
            <label>Tube Type</label>
            <select value={tube} onChange={e => setTube(e.target.value)}>
              {TUBES.map(t => <option key={t.value} value={t.value}>{t.label}</option>)}
            </select>
          </div>
          <div className="ctrl">
            <label>Frequency: <span>{freq}Hz</span></label>
            <input type="range" min="80" max="2000" step="1" value={freq} onChange={e => setFreq(+e.target.value)} />
          </div>
          <div className="ctrl">
            <label>Drive: <span>{drive}V</span> {drive > 2 ? '(overdriven)' : drive > 0.8 ? '(moderate)' : '(clean)'}</label>
            <input type="range" min="0.05" max="5" step="0.05" value={drive} onChange={e => setDrive(+e.target.value)} />
          </div>
          <div className="ctrl">
            <label>Duration: <span>{dur}s</span></label>
            <input type="range" min="0.5" max="5" step="0.5" value={dur} onChange={e => setDur(+e.target.value)} />
          </div>
          <button className="btn" disabled={loading} onClick={generate}>
            {loading ? 'Generating...' : 'Generate'}
          </button>
        </div>
        <div className="synth-output">
          {error && <div style={{ color: 'var(--red)', fontSize: 13 }}>{error}</div>}
          {result ? (
            <>
              <div>
                <span className="tag tag-dry">DRY</span>
                <audio controls src={`/${result.dry}`} style={{ width: '100%' }} />
              </div>
              <div>
                <span className="tag tag-wet">WET</span>
                <audio controls src={`/${result.wet}`} style={{ width: '100%' }} />
              </div>
              <div className="synth-info">
                Gain: {result.gain} | THD: {result.thd} | Tube: {result.tube}
              </div>
            </>
          ) : !loading && (
            <div style={{ color: 'var(--text3)', fontSize: 13 }}>Adjust parameters and click Generate</div>
          )}
        </div>
      </div>
    </div>
  )
}
