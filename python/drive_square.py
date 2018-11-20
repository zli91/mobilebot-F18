#! /usr/bin/python
import lcm
from time import sleep
import sys
import math
# import numpy as np
sys.path.append("lcmtypes")

# from future import print_function
from lcmtypes import mbot_encoder_t
from lcmtypes import mbot_imu_t
from lcmtypes import mbot_motor_command_t
from lcmtypes import odometry_t
from lcmtypes import pose_xyt_t
from lcmtypes import robot_path_t
from lcmtypes import timestamp_t
ang_v = 3.1415/4.0
trans_v = 0.1
pi = 3.1415926



class WaypointFollower():
    def __init__(self):
        """ Setup LCM and subscribe """
        self.lc = lcm.LCM()
        self.lcm_sub = self.lc.subscribe("ODOMETRY", self.odometry_handler)
        self.waypoints = [[0.0,0.0],[1.0,0.0],[1.0,1.0],[0.0,1.0],[0.0,0.0]]
        self.wpt_num = 0
        self.wpt_thresh = 0.01
        self.odo_x = 0;
        self.odo_y = 0;
        self.odo_theta = 0;
   
    def odometry_handler(self, channel, data):
        msg = odometry_t().decode(data)
        self.odo_x = msg.x
        self.odo_y = msg.y
        self.odo_theta = msg.theta
        # print(msg.x)
        # print(msg.y)
        # print(msg.theta)

        

    def motor_cmd_publish(self,transv,angularv):
        msg = mbot_motor_command_t()
        msg.utime = 0.0
        msg.trans_v = transv
        msg.angular_v = angularv
        self.lc.publish("MBOT_MOTOR_COMMAND",msg.encode())

def squaretask(counter,direction,trans_v,ang_v,distance):
    sys.stdout.write("\tCounter %.1f" % counter)
    tol1 = 0.8
    tol2 = 1.2
    if(counter == 0):
        sys.stdout.write("\tentered c1")
        # sys.stdout.flush()
        while not(tol1*distance< tempsquares.odo_x < tol2*distance):
            sys.stdout.write("\tentered c1_trans")
            tempsquares.motor_cmd_publish(trans_v,0)
            return
        while not(tol1*(pi/2)< math.fabs(tempsquares.odo_theta) < tol2*(pi/2)):
            sys.stdout.write("\tentered c1_turns")
            tempsquares.motor_cmd_publish(0,direction*ang_v)
            return
        if (tol1*(pi/2)< math.fabs(tempsquares.odo_theta) < tol2*(pi/2)):
            counter = 1
            return

    if(counter == 1):
        sys.stdout.write("\tentered c2")
        # sys.stdout.flush()
        while not (tol1*distance< math.fabs(tempsquares.odo_y) < tol2*distance):
            sys.stdout.write("\tentered c2_trans")
            tempsquares.motor_cmd_publish(trans_v,0)
        while not(tol1*(pi)< math.fabs(tempsquares.odo_theta) < tol2*(pi)):
            sys.stdout.write("\tentered c2_turns")
            tempsquares.motor_cmd_publish(0,direction*ang_v)
        counter = 2

    if(counter == 2):
        sys.stdout.write("\tentered c3")
        while not(math.fabs(tempsquares.odo_x) < (tol2-1)*distance):
            tempsquares.motor_cmd_publish(trans_v,0)
        while not(tol1*(pi/2)< math.fabs(tempsquares.odo_theta) < tol2*(pi/2)):
            tempsquares.motor_cmd_publish(0,direction*ang_v)
        counter = 3

    if(counter == 3):
        sys.stdout.write("\tentered c4")
        while not(math.fabs(tempsquares.odo_y) < (tol2-1)*distance):
            tempsquares.motor_cmd_publish(trans_v,0)
        while not(math.fabs(tempsquares.odo_theta) < (tol2-1)*pi):
            tempsquares.motor_cmd_publish(0,direction*ang_v)
        counter =0 


tempsquares = WaypointFollower()
distance = tempsquares.waypoints[1][0];
try:
    while True:
        tempsquares.lc.handle()
        squaretask(0,1,trans_v,ang_v,distance)
        sys.stdout.write("Odometry x %.6f" % tempsquares.odo_x)
        # sys.stdout.write("\tOdometry y %.6f" % tempsquares.odo_y)
        # sys.stdout.write("\tOdometry theta %.6f" % tempsquares.odo_theta)
        sys.stdout.flush()
        # print (tempsquares.odo_x,end = " ")
        # print (tempsquares.odo_y,end = " ")
        # print(tempsquares.odo_theta)
except KeyboardInterrupt:
    pass
# tempsquares.lc.


# tempsquares.motor_cmd_publish(0,ang_v)











