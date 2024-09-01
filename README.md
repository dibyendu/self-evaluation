[![Open in StackBlitz](https://developer.stackblitz.com/img/open_in_stackblitz_small.svg)](https://stackblitz.com/~/github/dibyendu/self-evaluation/tree/backend) &nbsp;&nbsp;&nbsp;&nbsp; <a alt='YouTube Video' href='https://youtu.be/R-qICICdEos' target='_blank'><img alt='YouTube Video' src='https://img.shields.io/badge/YouTube-red?style=flat&logo=youtube&logoColor=red&label=Watch%20on&labelColor=black'></a>




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
