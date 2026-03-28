import { useState, useEffect } from 'react'

function AudioFile({ name, file }) {
  return (
    <div style={{ marginBottom: 6 }}>
      <div style={{ fontSize: 11, color: 'var(--text2)', marginBottom: 2 }}>{name}</div>
      <audio controls preload="none" src={`/${file}`} style={{ width: '100%', height: 32 }} />
    </div>
  )
}

function AudioCard({ label, children }) {
  return (
    <div className="card">
      <div className="card-label">{label}</div>
      {children}
    </div>
  )
}

function Section({ title, sub, children }) {
  return (
    <div className="section">
      <div className="section-title">{title}</div>
      {sub && <div className="section-sub">{sub}</div>}
      {children}
    </div>
  )
}

export default function Demos() {
  const [data, setData] = useState(null)
  const [expanded, setExpanded] = useState({})

  useEffect(() => {
    fetch('/api/demos').then(r => r.json()).then(setData)
  }, [])

  if (!data) return <p style={{ color: 'var(--text2)' }}>Loading demos...</p>

  const toggle = (key) => setExpanded(e => ({ ...e, [key]: !e[key] }))

  return (
    <>
      {/* Amp Presets */}
      {Object.keys(data.amp_presets || {}).length > 0 && (
        <Section title="Amp Presets" sub="Full signal chain: preamp + tone stack + power amp + cabinet IR">
          {Object.entries(data.amp_presets).map(([preset, files]) => (
            <div key={preset} style={{ marginBottom: 16 }}>
              <div
                onClick={() => toggle(preset)}
                style={{ cursor: 'pointer', fontSize: 14, fontWeight: 600, color: 'var(--blue)', marginBottom: 8, userSelect: 'none' }}
              >
                {expanded[preset] ? '[-]' : '[+]'} {preset.replace(/_/g, ' ').replace(/\b\w/g, c => c.toUpperCase())}
                <span style={{ fontSize: 11, color: 'var(--text3)', marginLeft: 8 }}>{files.length} clips</span>
              </div>
              {expanded[preset] && (
                <div className="card-grid">
                  {files.map((f, i) => (
                    <AudioCard key={i} label={f.name}>
                      <AudioFile name="" file={f.file} />
                    </AudioCard>
                  ))}
                </div>
              )}
            </div>
          ))}
        </Section>
      )}

      {/* Gain Sweep */}
      {(data.gain_sweep || []).length > 0 && (
        <Section title="Marshall Gain Sweep" sub="Same chord, gain knob from 1 (clean) to 10 (full overdrive)">
          <div className="tube-grid">
            {data.gain_sweep.map((f, i) => (
              <div key={i} className="tube-card">
                <div className="tube-name">{f.name}</div>
                <audio controls preload="none" src={`/${f.file}`} />
              </div>
            ))}
          </div>
        </Section>
      )}

      {/* Physics Demos */}
      {Object.keys(data.physics || {}).length > 0 && (
        <Section title="Physics Models" sub="Real tube amp phenomena — each modeled from circuit physics">
          {Object.entries(data.physics).map(([category, files]) => (
            <div key={category} style={{ marginBottom: 16 }}>
              <div
                onClick={() => toggle('phys_' + category)}
                style={{ cursor: 'pointer', fontSize: 13, fontWeight: 600, color: 'var(--text)', marginBottom: 8, userSelect: 'none' }}
              >
                {expanded['phys_' + category] ? '[-]' : '[+]'} {category}
                <span style={{ fontSize: 11, color: 'var(--text3)', marginLeft: 8 }}>{files.length} clips</span>
              </div>
              {expanded['phys_' + category] && (
                <div className="tube-grid">
                  {files.map((f, i) => (
                    <div key={i} className="tube-card">
                      <div className="tube-desc">{f.name}</div>
                      <audio controls preload="none" src={`/${f.file}`} />
                    </div>
                  ))}
                </div>
              )}
            </div>
          ))}
        </Section>
      )}

      {/* Guitar Demos */}
      {(data.guitar_demos || []).length > 0 && (
        <Section title="Guitar Demos" sub="Blues riffs, chord progressions, drive sweeps, staccato hits">
          <div className="card-grid">
            {data.guitar_demos.map((d, i) => (
              <AudioCard key={i} label={d.name}>
                <div style={{ marginBottom: 6 }}>
                  <span className="tag tag-dry">DRY</span>
                  <audio controls preload="none" src={`/${d.dry}`} />
                </div>
                {d.wet.map((w, j) => (
                  <div key={j} style={{ marginBottom: 4 }}>
                    <span className="tag tag-wet">WET</span>
                    <audio controls preload="none" src={`/${w}`} />
                  </div>
                ))}
              </AudioCard>
            ))}
          </div>
        </Section>
      )}

      {/* Tube Comparison */}
      {(data.tubes || []).length > 0 && (
        <Section title="Tube Comparison" sub="Same input through different tube types">
          <div className="tube-grid">
            {data.tubes.map((t, i) => (
              <div key={i} className="tube-card">
                <div className="tube-name">{t.name}</div>
                <audio controls preload="none" src={`/${t.file}`} />
              </div>
            ))}
          </div>
        </Section>
      )}

      {/* Tone & Cabinet */}
      {(data.tone_cab || []).length > 0 && (
        <Section title="Tone Stack & Cabinet" sub="EQ and speaker cabinet impulse response demos">
          <div className="tube-grid">
            {data.tone_cab.map((f, i) => (
              <div key={i} className="tube-card">
                <div className="tube-desc">{f.name}</div>
                <audio controls preload="none" src={`/${f.file}`} />
              </div>
            ))}
          </div>
        </Section>
      )}

      {/* Simple dry/wet */}
      {(data.simple || []).length > 0 && (
        <Section title="Simple Tones — Dry vs Wet">
          <div className="card-grid">
            {data.simple.map((d, i) => (
              <AudioCard key={i} label={d.name}>
                {d.dry && <div style={{ marginBottom: 6 }}><span className="tag tag-dry">DRY</span><audio controls preload="none" src={`/${d.dry}`} /></div>}
                {(d.wet || []).map((w, j) => (
                  <div key={j}><span className="tag tag-wet">WET</span><audio controls preload="none" src={`/${w}`} /></div>
                ))}
              </AudioCard>
            ))}
          </div>
        </Section>
      )}
    </>
  )
}
