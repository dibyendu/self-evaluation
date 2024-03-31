import { createRoot } from 'react-dom/client'
import { createBrowserRouter, RouterProvider } from 'react-router-dom'

import App from './App'
import Visualize from './Visualize'


const router = createBrowserRouter([
  { path: '/', element: <App /> },
  { path: 'visualise', element: <Visualize /> }
])

createRoot(document.getElementById('root')).render(
  <RouterProvider router={router} />
)