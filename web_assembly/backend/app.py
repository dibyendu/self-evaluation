import os
from flask_cors import CORS
from flask import Flask, jsonify, send_from_directory

import time


# if changed, make sure to update it in self-evaluation/web_assembly/src/Config.js
PORT = 8000

demonstration_file = os.path.abspath(os.path.realpath(os.path.join(
  os.path.dirname(os.path.realpath(__file__)),
  '../../demonstrations/Scoop_Interactive/Demo1/joint_angles.csv'
)))

frontend_folder = os.path.abspath(os.path.realpath(os.path.join(
  os.path.dirname(os.path.realpath(__file__)),
  '../frontend'
)))




app = Flask(__name__)
CORS(app)


@app.route('/')
def index():
  return send_from_directory(frontend_folder, 'index.html')


@app.route('/visualise')
def visualise():
  return send_from_directory(frontend_folder, 'visualise.html')


@app.route('/assets/<path:filename>')
def static_files(filename):
  return send_from_directory(f'{frontend_folder}/assets', filename)




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
  
  file = open(demonstration_file, 'r')

  data = file.read()
  file.close()
  return jsonify({ 'demonstration': data })


if __name__ == '__main__':
  app.run(host='0.0.0.0', port=PORT)
