import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'
import { nodePolyfills } from 'vite-plugin-node-polyfills'

const serverConfig = {
  host: 'localhost',
  port: 8000
}

export default defineConfig({
  root: 'src/frontend',
  build: {
    outDir: '../frontend_compiled',
    rollupOptions: {
      input: [
        'src/frontend/index.html',
        'src/frontend/visualise.html'
      ]
    }
  },
  worker: {
    format: 'es'
  },
  server: {
    ...serverConfig
  },
  preview: {
    ...serverConfig
  },
  plugins: [
    react(),
    nodePolyfills({ include: ['module'] })
  ]
})