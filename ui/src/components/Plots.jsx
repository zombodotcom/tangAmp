import { useState, useEffect } from 'react'

export default function Plots() {
  const [images, setImages] = useState([])

  useEffect(() => {
    fetch('/api/images').then(r => r.json()).then(setImages)
  }, [])

  if (!images.length) return <p style={{ color: 'var(--text2)' }}>No plots yet. Run tests first to generate validation plots.</p>

  return (
    <div className="section">
      <div className="section-title">Validation Plots</div>
      <div className="section-sub">Click any plot to open full size</div>
      <div className="plot-grid">
        {images.map((img, i) => (
          <img key={i} src={`/${img}`} alt={img} onClick={() => window.open(`/${img}`, '_blank')} />
        ))}
      </div>
    </div>
  )
}
