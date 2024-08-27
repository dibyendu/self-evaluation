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
    ...serverConfig,
    proxy: {
      '/init': {
        target: 'https://dummyjson.com/c/aa0b-5443-44da-bc28',
        changeOrigin: true,
        rewrite: path => path.replace(/^\/init/, '')
      },
      '/start': {
        target: 'https://dummyjson.com/c/fb8b-a9f0-4b34-bd80',
        changeOrigin: true,
        rewrite: path => path.replace(/^\/start/, '')
      },
      '^/stop/.*': {
        target: 'https://dummyjson.com/c/0c99-8c1e-4279-800d',
        changeOrigin: true,
        rewrite: path => path.replace(/^\/stop\/.*/, ''),
      },
      '/pose': {
        target: 'https://dummyjson.com/c/8bc5-f188-44bd-9c12',
        changeOrigin: true,
        rewrite: path => path.replace(/^\/pose/, '')
      }
    }
  },
  preview: {
    ...serverConfig
  },
  plugins: [
    react(),
    nodePolyfills({ include: ['module'] })
  ]
})
