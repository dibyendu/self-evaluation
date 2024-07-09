## Run on the robot's server

### Activate virtualenv
`source ~/miniconda3/bin/activate py37`

`cd ~/self-evaluation/web_assembly`

### Build the files (one time action)
`npm i`
`pip install -r backend/requirements.txt`
`npm run build`

### Start server
`python backend/app.py`

### Deactivate virtualenv
`source ~/miniconda3/bin/deactivate`
