[![Open in StackBlitz](https://developer.stackblitz.com/img/open_in_stackblitz_small.svg)](https://stackblitz.com/github/dibyendu/self-evaluation/tree/main/web_assembly)


## Run in client mode (web-assembly)

`cd web_assembly`

### Compile server code

`emcc -std=c++11 -O2 ../kinlib/*.cpp server.cpp -o src/server.mjs -s ALLOW_MEMORY_GROWTH -lembind`

### Start application
`npm start`




## Run in client-server mode

`cd web_assembly`

### Build the files
`npm i`
`cd backend && pip install -r requirements.txt && cd ..`
`npm run build`

### Start server
`python backend/app.py`




## Run in command-line (CLI) mode

### Compile server code

`g++ -std=c++11 -O2 kinlib/*.cpp server.cpp -o server -lpthread`

### Start server
`./server`

### Start client
`python client.py`
