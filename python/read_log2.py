import lcm
import time
import sys
import math
import csv
import pickle
sys.path.append("lcmtypes")

from lcmtypes import mbot_encoder_t
from lcmtypes import mbot_imu_t
from lcmtypes import mbot_motor_command_t
from lcmtypes import odometry_t
from lcmtypes import pose_xyt_t
from lcmtypes import robot_path_t
from lcmtypes import timestamp_t

if len(sys.argv) < 2:
    sys.stderr.write("usage: read-log <logfile>\n")
    sys.exit(1)

log = lcm.EventLog(sys.argv[1], "r")

data = {'slam_pose': {'x': [], 'y':[], 'th': [], 'time': []}, 
        'odometry': {'x': [], 'y':[], 'th': [], 'time': []}}

for event in log:
    if event.channel == "SLAM_POSE":
        msg = pose_xyt_t.decode(event.data)
        data['slam_pose']['x'].append(msg.x)
        data['slam_pose']['y'].append(msg.y)
        data['slam_pose']['th'].append(msg.theta)
        data['slam_pose']['time'].append(msg.utime)
    if event.channel == "ODOMETRY":
        msg = pose_xyt_t.decode(event.data)
        data['odometry']['x'].append(msg.x)
        data['odometry']['y'].append(msg.y)
        data['odometry']['th'].append(msg.theta)
        data['odometry']['time'].append(msg.utime)

pickle.dump( data, open( "team_5_data.p", "wb" ) )
