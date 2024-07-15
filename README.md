## Run on the Robot's server

### Activate virtualenv
`source ~/miniconda3/bin/activate py37`

`cd ~/self-evaluation/web_assembly`

### Install dependencies and build the files (one time action)
`npm i`

`pip install -r backend/requirements.txt`

`npm run build`

### Start the server
`python backend/app.py`

### Deactivate virtualenv
`source ~/miniconda3/bin/deactivate`
