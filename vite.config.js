import { resolve } from 'path'
import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'
import { nodePolyfills } from 'vite-plugin-node-polyfills'

const serverConfig = {
  host: 'localhost',
  port: 8000
}

export default defineConfig({
  build: {
    outDir: 'frontend_compiled',
    rollupOptions: {
      input: [
        resolve(__dirname, 'src/frontend', 'index.html'),
        resolve(__dirname, 'src/frontend', 'visualise.html')
      ]
    }
  },
  worker: {
    format: 'es'
  },
  root: 'src',
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
