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
    outDir: '../frontend',
    rollupOptions: {
      input: [
        resolve(__dirname, 'src', 'index.html'),
        resolve(__dirname, 'src', 'visualise.html')
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
