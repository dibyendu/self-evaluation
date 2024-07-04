from flask import Flask, jsonify
from flask_cors import CORS

import time


# make sure to update it in self-evaluation/web_assembly/src/Config.js
PORT = 8080


app = Flask(__name__)
CORS(app)


@app.route('/init')
def init():
  time.sleep(4)
  return jsonify({ 'status': True })


@app.route('/start')
def start():
  time.sleep(4)
  return jsonify({ 'pid': 1234 })


@app.route('/stop/<pid>')
def stop(pid):
  time.sleep(4)
  
  print(f'kill -9 {pid}')
  
  file = open('../../demonstrations/Scoop_Interactive/Demo1/joint_angles.csv', 'r')

  data = file.read()
  file.close()
  return jsonify({ 'demonstration': data })


if __name__ == '__main__':
  app.run(host='0.0.0.0', port=PORT)
