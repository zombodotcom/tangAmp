import { useState, useEffect, useRef } from 'react'

export default function Tests() {
  const [data, setData] = useState(null)
  const intervalRef = useRef(null)

  useEffect(() => {
    const poll = () => fetch('/api/results').then(r => r.json()).then(setData).catch(() => {})
    poll()
    intervalRef.current = setInterval(poll, 3000)
    return () => clearInterval(intervalRef.current)
  }, [])

  function runTests() {
    fetch('/api/run-tests', { method: 'POST' })
    // Reset to pending
    setData(d => d ? { ...d, tests: d.tests.map(t => ({ ...t, status: 'pending', details: {}, error: undefined, time: undefined })) } : d)
  }

  const tests = data?.tests || []
  const pass = tests.filter(t => t.status === 'pass').length
  const fail = tests.filter(t => t.status === 'fail').length
  const running = tests.filter(t => t.status === 'running').length

  let sumClass = 'summary-bar'
  let sumText = 'No results'
  if (running > 0) { sumClass += ' summary-running'; sumText = `Running... ${pass} passed, ${running} running` }
  else if (fail > 0) { sumClass += ' summary-fail'; sumText = `${fail} FAILED, ${pass} passed` }
  else if (pass > 0) { sumClass += ' summary-pass'; sumText = `ALL ${pass} TESTS PASSED` }

  return (
    <>
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: 16 }}>
        <div className={sumClass} style={{ flex: 1, marginBottom: 0, marginRight: 12 }}>{sumText}</div>
        <button className="btn-outline" onClick={runTests}>Re-run Tests</button>
      </div>
      <div className="test-grid">
        {tests.map((t, i) => (
          <div key={i} className={`test-card ${t.status || 'pending'}`}>
            <div className="test-row">
              <span className="test-name">{t.name}</span>
              <span className={`badge badge-${t.status || 'pending'}`}>{t.status || 'pending'}</span>
            </div>
            {t.details && Object.keys(t.details).length > 0 && (
              <div className="test-details">
                {Object.entries(t.details).map(([k, v]) => <div key={k}>{k}: {v}</div>)}
              </div>
            )}
            {t.time && <div className="test-time">{t.time}</div>}
            {t.error && <div className="test-error">{t.error}</div>}
          </div>
        ))}
      </div>
      <div className="section" style={{ marginTop: 24 }}>
        <div className="section-title">Roadmap</div>
        <div className="roadmap">
          {[
            ['Phase 1', 'Audio I/O (I2S ADC + DAC)'],
            ['Phase 2', 'Simple nonlinearity (soft clip LUT)'],
            ['Phase 3 - current', 'Single WDF triode stage (29.3dB, 2.19% error)', true],
            ['Phase 4', 'Cascaded preamp stages + tone stack'],
            ['Phase 5', 'Power amp (6L6/EL34/300B) + cabinet IR'],
          ].map(([phase, desc, current], i) => (
            <div key={i} className={`roadmap-item${current ? ' current' : ''}`}>
              <div className="phase">{phase}</div>
              <div className="desc">{desc}</div>
            </div>
          ))}
        </div>
      </div>
    </>
  )
}
