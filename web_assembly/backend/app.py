# add this line to baxter.sh
# if [ $# -ne 0 ]; then tf2=$(mktemp); cp $tf $tf2; echo $tf2; fi

import os
import time
import signal
import subprocess
from flask_cors import CORS
from flask import Flask, jsonify, send_from_directory


# if changed, make sure to update it in self-evaluation/web_assembly/src/Config.js
PORT = 8000

workspace_path = '/home/shubham/baxter_ws'




frontend_folder = os.path.abspath(os.path.realpath(os.path.join(os.path.dirname(os.path.realpath(__file__)), '../frontend')))

env = os.environ.copy()

app = Flask(__name__)
CORS(app)


def generate_rcfile():
  process = subprocess.Popen('./baxter.sh 011502P0023.local', cwd=workspace_path, stdout=subprocess.PIPE, stdin=subprocess.DEVNULL, stderr=subprocess.DEVNULL, shell=True, env=env)
  rcfile = process.stdout.read(1024).decode().strip()
  process.terminate()
  return rcfile


@app.route('/')
def index():
  return send_from_directory(frontend_folder, 'index.html')


@app.route('/visualise')
def visualise():
  return send_from_directory(frontend_folder, 'visualise.html')


@app.route('/assets/<path:filename>')
def static_files(filename):
  return send_from_directory('%s/assets' % frontend_folder, filename)




@app.route('/init')
def init():
  rcfile = generate_rcfile()
  process = subprocess.Popen(['bash', '--rcfile', rcfile, '-i'], cwd=workspace_path, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, env=env, universal_newlines=True)
  output, error = process.communicate('rosrun baxter_tools enable_robot.py -e\nexit\n')
  process.terminate()
  os.system('rm -f %s' % rcfile)
  return jsonify({ 'status': True if error is None else False })


@app.route('/start')
def start():
  rcfile = generate_rcfile()
  file = open('%s.sh' % rcfile, 'w')
  file.write('''#!/bin/bash
source ~/miniconda3/bin/deactivate
source %s
rosrun baxter_examples joint_recorder.py --file %s.csv &
echo $! > /tmp/$$.pid
echo %s > /tmp/$!.pid''' % (rcfile, rcfile, rcfile))
  file.close()
  process = subprocess.Popen(['bash', '%s.sh' % rcfile], cwd=workspace_path, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, env=env, universal_newlines=True)
  time.sleep(2)
  process.terminate()
  return jsonify({ 'pid': process.pid })


@app.route('/stop/<pid>')
def stop(pid):
  file = open('/tmp/%s.pid' % pid, 'r')
  pid1 = file.read().strip()
  file.close()

  file = open('/tmp/%s.pid' % pid1, 'r')
  rcfile = file.read().strip()
  file.close()

  os.kill(int(pid1), signal.SIGTERM)

  time.sleep(2)

  file = open('%s.csv' % rcfile, 'r')
  data = file.read()
  file.close()

  os.system('rm -f /tmp/%s.pid /tmp/%s.pid %s*' % (pid, pid1, rcfile))
  return jsonify({ 'demonstration': data })



if __name__ == '__main__':
  app.run(host='0.0.0.0', port=PORT)
