[![Open in StackBlitz](https://developer.stackblitz.com/img/open_in_stackblitz_small.svg)](https://stackblitz.com/~/github/dibyendu/self-evaluation/tree/backend) &nbsp;&nbsp;&nbsp;&nbsp; <a alt='YouTube Video' href='https://youtu.be/R-qICICdEos'><img alt='YouTube Video' src='https://img.shields.io/badge/YouTube-red?style=flat&logo=youtube&logoColor=red&label=Watch%20on&labelColor=black'></a>


| Robot's belief after acquiring `2` kinesthetic demonstrations                               | Robot acheived `≥ 95%` success probability with high (`95%`) confidence       |
| ------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------- |
| ![belief](https://github.com/user-attachments/assets/76f0283e-d7d8-467c-871e-e24ec3c70004)  | ![final](https://github.com/user-attachments/assets/3f83a0dc-bf2b-422a-9e69-137b4ed2ae37) |


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
