import { useState, useEffect } from 'react'

export default function Plots() {
  const [images, setImages] = useState([])

  useEffect(() => {
    fetch('/api/demos').then(r => r.json()).then(d => setImages(d.plots || []))
  }, [])

  if (!images.length) return <p style={{ color: 'var(--text2)' }}>No plots found. Run validation scripts to generate them.</p>

  return (
    <div className="section">
      <div className="section-title">Validation & Analysis Plots</div>
      <div className="section-sub">{images.length} plots — click to open full size</div>
      <div className="plot-grid">
        {images.map((img, i) => (
          <div key={i} style={{ position: 'relative' }}>
            <div style={{ fontSize: 11, color: 'var(--text3)', marginBottom: 4 }}>
              {img.split('/').pop()}
            </div>
            <img src={`/${img}`} alt={img} onClick={() => window.open(`/${img}`, '_blank')} />
          </div>
        ))}
      </div>
    </div>
  )
}
