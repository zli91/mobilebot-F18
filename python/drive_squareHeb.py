#! /usr/bin/python
import lcm
from time import sleep
import sys
import math
# import numpy as np
sys.path.append("lcmtypes")

from lcmtypes import mbot_encoder_t
from lcmtypes import mbot_imu_t
from lcmtypes import mbot_motor_command_t
from lcmtypes import odometry_t
from lcmtypes import pose_xyt_t
from lcmtypes import robot_path_t
from lcmtypes import timestamp_t


pi = math.pi



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


    def motor_cmd_publish(self,transv,angularv):
        msg = mbot_motor_command_t()
        msg.utime = 0.0
        msg.trans_v = transv
        msg.angular_v = angularv
        self.lc.publish("MBOT_MOTOR_COMMAND",msg.encode())

    def move2pose(self, x_goal, y_goal, theta_goal, linearVelocity, angularVelocity):
        self.lc.handle()
        x_diff = x_goal - self.odo_x
        y_diff = y_goal - self.odo_y

        rho = math.sqrt(x_diff**2 + y_diff**2)

        while rho > 0.4:
            self.lc.handle()
            x_diff = x_goal - self.odo_x
            y_diff = y_goal - self.odo_y


            rho = math.sqrt(x_diff**2 + y_diff**2)
            alpha = (math.atan2(y_diff, x_diff) - self.odo_theta + pi) % (2 * pi) - pi
            beta = (theta_goal - self.odo_theta - alpha + pi) % (2 * pi) - pi

            if alpha > pi / 2 or alpha < -pi / 2:
                self.motor_cmd_publish(-linearVelocity, math.copysign(1, alpha)*angularVelocity)
            else:
                self.motor_cmd_publish(linearVelocity, math.copysign(1, alpha)*angularVelocity)

    def moveDistance(self, Distance_goal, linearVelocity):
        self.lc.handle()
        rho = Distance_goal
        theta_goal = self.odo_theta

        x_goal = self.odo_x + Distance_goal*math.cos(theta_goal)
        y_goal = self.odo_y + Distance_goal*math.sin(theta_goal)

        inFront = math.cos(theta_goal)*math.cos(theta_goal) + math.sin(theta_goal)*math.sin(theta_goal)       

        while rho > 0.01*Distance_goal and inFront >= 0:
            self.lc.handle()
            x_diff = x_goal - self.odo_x
            y_diff = y_goal - self.odo_y
            rho = math.sqrt(x_diff**2 + y_diff**2)
            inFront = x_diff*math.cos(self.odo_theta) + y_diff*math.sin(self.odo_theta)
            theta_diff = theta_goal - self.odo_theta

            Kp = 0.950

            print(rho, "     ", inFront)
            # print(theta_diff)

            self.motor_cmd_publish(linearVelocity, Kp*theta_diff)
        
        self.motor_cmd_publish(0.0, 0.0)        


    def turnAngle(self, AngularDistance_goal, angularVelocity):
        self.lc.handle()
        alpha = AngularDistance_goal
        theta_goal = self.odo_theta + alpha
        
        if theta_goal > pi:
            theta_goal = theta_goal%(2*pi) - 2*pi
        elif theta_goal <= -pi:
            theta_goal = theta_goal%(2*pi)

        # self.motor_cmd_publish(0, math.copysign(1, alpha)*angularVelocity)

        while math.fabs(alpha) > 0.1:
            self.lc.handle()
            alpha_old = alpha
            alpha = theta_goal - self.odo_theta

            if math.fabs(alpha) <= pi:
                finalAngularVelocity = math.copysign(1, alpha)*angularVelocity
            else:
                finalAngularVelocity = - math.copysign(1, alpha)*angularVelocity

            # print(alpha)

            if math.fabs(alpha) < math.fabs(AngularDistance_goal)/2:
                print alpha
                self.motor_cmd_publish(0.0, .5*alpha)
            else:
                # print angularVelocity
                self.motor_cmd_publish(0.0, finalAngularVelocity)


        print("I am about to stop the motors\n\n\n")
        self.motor_cmd_publish(0.0, 0.0)


# def squareTask(WPFollower, squareSide, linearVelocity, angularVelocity):
#     x_goal = WPFollower.waypoints[1][0]*squareSide
#     y_goal = WPFollower.waypoints[1][1]*squareSide
#     theta_goal = 0.0

#     WPFollower.move2pose(x_goal, y_goal, theta_goal, linearVelocity, angularVelocity)

#     x_goal = WPFollower.waypoints[2][0]*squareSide
#     y_goal = WPFollower.waypoints[2][1]*squareSide
#     theta_goal = pi/2

#     WPFollower.move2pose(x_goal, y_goal, theta_goal, linearVelocity, angularVelocity)

#     x_goal = WPFollower.waypoints[3][0]*squareSide
#     y_goal = WPFollower.waypoints[3][1]*squareSide
#     theta_goal = pi

#     WPFollower.move2pose(x_goal, y_goal, theta_goal, linearVelocity, angularVelocity)

#     x_goal = WPFollower.waypoints[4][0]*squareSide
#     y_goal = WPFollower.waypoints[4][1]*squareSide
#     theta_goal = -pi/2

#     WPFollower.move2pose(x_goal, y_goal, theta_goal, linearVelocity, angularVelocity)

def squareTask(WPFollower, squareSide, linearVelocity, angularVelocity, direction):
    print "starting first moveDistance"
    WPFollower.moveDistance(squareSide, linearVelocity)
    print "starting first turnAngle"
    WPFollower.turnAngle(direction*pi/2, angularVelocity)
    print "starting sencond moveDistance"
    WPFollower.moveDistance(squareSide, linearVelocity)
    print "starting second turnAngle"
    WPFollower.turnAngle(direction*pi/2, angularVelocity)
    print "starting third moveDistance"
    WPFollower.moveDistance(squareSide, linearVelocity)
    print "starting third turnAngle"
    WPFollower.turnAngle(direction*pi/2, angularVelocity)
    print "starting fourth moveDistance"
    WPFollower.moveDistance(squareSide, linearVelocity)
    print "starting fouth turnAngle"
    WPFollower.turnAngle(direction*pi/2, angularVelocity)

    return


tempsquares = WaypointFollower()
squareSide = 1.0
ang_v = 3.1415926/4.0
# ang_v = 0
# trans_v = 0.0

trans_v = 0.2

# try:
#     while True:
#         tempsquares.lc.handle()
#         squareTask(tempsquares, squareSide, trans_v, ang_v)
# except KeyboardInterrupt:
#     pass


try:
    
    tempsquares.lc.handle()
    squareTask(tempsquares, squareSide, trans_v, ang_v, -1)

except KeyboardInterrupt:
    pass
