import sys
import struct
import socket


# X.ABCDEFG           '!f'   4 bytes
# X.ABCDEFGIJKLMNOP   '!d'   8 bytes
format_str = '!d'


def send_int(sock, value):
  sock.sendall(socket.htonl(value).to_bytes(4, byteorder=sys.byteorder))


def recv_int(sock):
  bytes = sock.recv(4)
  return socket.ntohl(int.from_bytes(bytes, byteorder='big')) if bytes else None


def send_float(sock, value):
  sock.sendall(struct.pack(format_str, value))


def recv_float(sock):
  bytes = sock.recv(struct.calcsize(format_str))
  return struct.unpack(format_str, bytes)[0] if bytes else None


def send_str(sock, string):
  send_int(sock, len(string))
  sock.sendall(str.encode(string))


def recv_bool(sock):
  bytes = sock.recv(1)
  return struct.unpack('?', bytes)[0] if bytes else None


def remote_planner(remote_address, demontrations, init_joint_config, task_instances):

  if type(remote_address) == type(''):
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
  else:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  
  sock.connect(remote_address)
  
  send_int(sock, len(demontrations))
  for demo in demontrations:
    send_str(sock, demo['recorded_demo_file'])
    send_str(sock, demo['object_poses_file'])
    send_float(sock, demo['region_of_interest'])
    send_float(sock, demo['score'])
  
  send_int(sock, len(task_instances))
  for task_instance in task_instances:
    send_int(sock, len(task_instance))
    for pose in task_instance:
      pose = pose.flatten().tolist()
      send_int(sock, len(pose))
      for val in pose:
        send_float(sock, val)
  
  send_int(sock, len(init_joint_config))
  for val in init_joint_config:
    send_float(sock, val)
  
  motion_plans = []
  for _ in task_instances:
    plans = []
    for _ in demontrations:
      n_screw_segments = recv_int(sock)
      n_joints, plan = recv_int(sock), []
      for _ in range(n_joints):
        joint_vector_length, joint_vector = recv_int(sock), []
        for _ in range(joint_vector_length):
          joint_vector.append(recv_float(sock))
        plan.append(joint_vector)
  
      is_successful = recv_bool(sock)
  
      failed_screw_segment, failed_joint_angle = None, None
      if not is_successful:
        failed_screw_segment = recv_int(sock)
        failed_joint_angle = recv_int(sock)
  
      plans.append({
        'is_successful': is_successful,
        'n_screw_segments': n_screw_segments,
        'plan': plan,
        'failed_screw_segment': failed_screw_segment,
        'failed_joint_angle': failed_joint_angle
      })
  
      if is_successful: break
  
    motion_plans.append(plans)
  
  return motion_plans
