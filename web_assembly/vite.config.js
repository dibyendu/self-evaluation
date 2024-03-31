import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

const serverConfig = {
  host: 'localhost',
  port: 8000
}

export default defineConfig({
  build: {
    outDir: '../build'
  },
  root: 'src',
  server: {
    ...serverConfig
  },
  preview: {
    ...serverConfig
  },
  plugins: [react()]
})  