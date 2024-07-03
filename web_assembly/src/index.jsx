import { createRoot } from 'react-dom/client'
import { createBrowserRouter, RouterProvider } from 'react-router-dom'

import App from './App'
import Visualise from './Visualise'


const router = createBrowserRouter([
  { path: '/', element: <App /> },
  { path: '/visualise.html', element: <Visualise /> }
])

createRoot(document.getElementById('root')).render(
  <RouterProvider router={router} />
)
