[![Open in StackBlitz](https://developer.stackblitz.com/img/open_in_stackblitz_small.svg)](https://stackblitz.com/github/dibyendu/self-evaluation/tree/main/web_assembly)


![belief](https://github.com/user-attachments/assets/77b3ee97-9630-4d39-97b8-f65cd83fdccb)


# Run in browser (using web-assembly)

`cd web_assembly`

### Start application
`npm start`

### (Optionally) Compile the planner code

`emcc -std=c++11 -O2 ../kinlib/*.cpp planner.cpp -o src/planner.mjs -s ALLOW_MEMORY_GROWTH -lembind`


# Run in client-server mode

### Compile the planner code

`g++ -std=c++11 -O2 kinlib/*.cpp planner.cpp -o server -lpthread`

### Start server
`./server`

### Start client
`python client.py`
