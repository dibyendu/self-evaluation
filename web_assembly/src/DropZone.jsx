import { useState, useRef } from 'react'


function directoryWalk(item, path='') {
  return new Promise(resolve => {
    if (item.isFile) {
      item.file(file => {
        file.filepath = path + file.name
        resolve(file)
      })
      return
    }
    const dirReader = item.createReader()
    dirReader.readEntries(entries => {
      Promise.all(entries.map(entry => directoryWalk(entry, `${path}${item.name}/`)))
      .then(fileList => resolve(fileList.flat()))
    })
  })
}


const getFileKey = file => file.webkitRelativePath || file.filepath || file.name


export function DropZoneWrapper({ getFiles, children, style }) {

  const inputRef = useRef(null)

  const [dropEnabled, setDropEnabled] = useState(false)

  return (
    <div
      style={{ display: 'flex', flexDirection: 'column', backgroundColor: dropEnabled ? '#cdcdcd70' : '', margin: '0 auto', justifyContent: 'center', alignItems: 'center', ...style }}
      onDrop={event => {
        event.preventDefault()
        Promise.all([...event.dataTransfer.items].map(item => directoryWalk(item.webkitGetAsEntry())))
        .then(fileList => fileList.flat())
        .then(fileList => getFiles(Object.assign(...fileList.map(file => ({ [getFileKey(file)]: file })))))
        setDropEnabled(false)
      }}
      onDragOver={event => {
        event.preventDefault()
        setDropEnabled(true)
      }}
      onDragEnter={() => setDropEnabled(true)}
      onDragLeave={() => setDropEnabled(false)}
    >
      {children}
    </div>
  )
}


export default function App({ getFiles, icon, text, onlyDirectory }) {

  const inputRef = useRef(null)

  const [dropEnabled, setDropEnabled] = useState(false)

  return (
    <>
      <div
        style={{ width: '50%', height: 200, backgroundColor: dropEnabled ? '#cdcdcd70' : '#fdfbcc80', margin: '0 auto', border: '1px dashed #666', borderRadius: 30, display: 'flex', flexDirection: 'column', justifyContent: 'center', alignItems: 'center', cursor: 'grab' }}
        onDrop={event => {
          event.preventDefault()
          Promise.all([...event.dataTransfer.items].map(item => directoryWalk(item.webkitGetAsEntry())))
          .then(fileList => fileList.flat())
          .then(fileList => getFiles(Object.assign(...fileList.map(file => ({ [getFileKey(file)]: file })))))
          setDropEnabled(false)
        }}
        onDragOver={event => {
          event.preventDefault()
          setDropEnabled(true)
        }}
        onDragEnter={() => setDropEnabled(true)}
        onDragLeave={() => setDropEnabled(false)}
        onClick={event => {
          event.preventDefault()
          inputRef.current.click()
        }}
      >
        {icon && <span className='material-symbols-rounded' style={{ fontSize: 60 }}>{icon}</span>}
        <p>{text}</p>
        <span className='material-symbols-rounded'>place_item</span>
      </div>
      <input type='file' ref={inputRef} {...(onlyDirectory ? { webkitdirectory: 'true' } : {})} multiple style={{ display: 'none' }} onChange={() => {
        let fileList = event.target.files
        fileList = new Array(fileList.length).fill().map((_, index) => fileList.item(index))
        getFiles(Object.assign(...fileList.map(file => ({ [getFileKey(file)]: file }))))
      }}/>
    </>
  )
}