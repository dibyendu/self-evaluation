## Run on the Robot's server

`cd ~/self-evaluation/web_assembly`

### Activate virtualenv
`source ~/miniconda3/bin/activate py37`

### Install dependencies and build the files (only if the code changes)
`npm i`

`pip install -r backend/requirements.txt`

`npm run build`

### Start the server
`python backend/app.py`

### (Optionally) Deactivate virtualenv
`source ~/miniconda3/bin/deactivate`
