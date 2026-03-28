import { useState, useEffect } from 'react'
import AudioPair from './AudioPair'

export default function Demos() {
  const [data, setData] = useState(null)

  useEffect(() => {
    fetch('/api/demos').then(r => r.json()).then(setData)
  }, [])

  if (!data) return <p style={{ color: 'var(--text2)' }}>Loading demos...</p>

  return (
    <>
      {data.demos.length > 0 && (
        <div className="section">
          <div className="section-title">Guitar-Like Demos</div>
          <div className="section-sub">Blues riffs, chord progressions, drive sweeps, and staccato hits through the WDF triode simulation</div>
          <div className="card-grid">
            {data.demos.map((d, i) => (
              <AudioPair key={i} label={d.name} dry={d.dry} wet={d.wet} />
            ))}
          </div>
        </div>
      )}

      {data.tubes.length > 0 && (
        <div className="section">
          <div className="section-title">Tube Comparison</div>
          <div className="section-sub">Same input signal through 5 different tube types — hear the character difference</div>
          <div className="tube-grid">
            {data.tubes.map((t, i) => (
              <div key={i} className="tube-card">
                <div className="tube-name">{t.name}</div>
                <audio controls preload="none" src={`/${t.file}`} />
              </div>
            ))}
          </div>
        </div>
      )}

      {data.simple.length > 0 && (
        <div className="section">
          <div className="section-title">Simple Tones — Dry vs Wet</div>
          <div className="card-grid">
            {data.simple.map((d, i) => (
              <AudioPair key={i} label={d.name} dry={d.dry} wet={d.wet} />
            ))}
          </div>
        </div>
      )}
    </>
  )
}
