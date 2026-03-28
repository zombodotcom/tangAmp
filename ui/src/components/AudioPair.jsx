export default function AudioPair({ label, dry, wet }) {
  return (
    <div className="card">
      <div className="card-label">{label}</div>
      {dry && (
        <div style={{ marginBottom: 8 }}>
          <span className="tag tag-dry">DRY</span>
          <audio controls preload="none" src={dry} />
        </div>
      )}
      {(Array.isArray(wet) ? wet : [wet]).filter(Boolean).map((w, i) => (
        <div key={i} style={{ marginBottom: 6 }}>
          <span className="tag tag-wet">{wet.length > 1 ? w.split('/').pop().replace('.wav','').replace('wet_','') : 'WET'}</span>
          <audio controls preload="none" src={w} />
        </div>
      ))}
    </div>
  )
}
