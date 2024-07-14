[![Open in StackBlitz](https://developer.stackblitz.com/img/open_in_stackblitz_small.svg)](https://stackblitz.com/github/dibyendu/self-evaluation/tree/main/web_assembly)


# Run in browser (using web-assembly)

`cd web_assembly`

### Compile the server code

`emcc -std=c++11 -O2 ../kinlib/*.cpp server.cpp -o src/server.mjs -s ALLOW_MEMORY_GROWTH -lembind`

### Start application
`npm start`


# Run in client-server mode

### Compile the server code

`g++ -std=c++11 -O2 kinlib/*.cpp server.cpp -o server -lpthread`

### Start server
`./server`

### Start client
`python client.py`
