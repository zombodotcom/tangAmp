import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

export default defineConfig({
  plugins: [react()],
  server: {
    port: 8420,
    proxy: {
      '/api': 'http://localhost:8421',
      '/demos': 'http://localhost:8421',
      '/wav_': 'http://localhost:8421',
      '/synth_cache': 'http://localhost:8421',
      '/validation_': 'http://localhost:8421',
      '/wdf_sim': 'http://localhost:8421',
      '/waveforms': 'http://localhost:8421',
      '/harmonics': 'http://localhost:8421',
      '/tube_plot': 'http://localhost:8421',
    }
  }
})
