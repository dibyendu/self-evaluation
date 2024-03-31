onmessage = ({ data: fileMap }) => {
  for (const key in fileMap) {
    fileMap[key] = { file: fileMap[key], data: null }

    const reader = new FileReader()
    reader.onerror = () => console.error('Error reading the file:', fileMap[key].file.name)
    reader.onload = ({ target: { result: data }}) => {
      fileMap[key] = data
      if (Object.values(fileMap).every(({ data }) => data !== null))
        postMessage({ result: Object.entries(fileMap) })
    }
    reader.readAsText(fileMap[key].file, 'UTF-8')
  }
}