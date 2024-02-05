import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool
import math
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv2 import aruco
import cv2 as cv
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from geometry_msgs.msg import Wrench
hex_pts = [[200, 150], [175, 200], [125, 200], [100, 150], [125, 100], [175, 100],[200, 150]]
tri_pts = [[300, 100], [400, 100], [300, 200], [400, 100]]
rect_pts = [[200, 300], [400, 300], [400, 400], [200, 400], [200, 300]]
bot1_index = 0
bot2_index = 0
bot3_index = 0
bot1_flag = 0


class Controller(Node):
    def __init__(self):
        super().__init__('Controller')
        self.pen1_subcriber = self.create_subscription(Pose2D, "/pen2_pose", self.bot1_callback, 10)
        self.pen2_subcriber = self.create_subscription(Pose2D, "/pen1_pose", self.bot2_callback, 10)
        self.pen3_subcriber = self.create_subscription(Pose2D, "/pen3_pose", self.bot3_callback, 10)

        self.bot1_pub = self.create_publisher(Twist, "/bot2_vel", 10)
        self.bot1_pen_pub = self.create_publisher(Bool, "/pen2_down", 10)
        self.prev_error_x = 0
        self.prev_error_y = 0
        self.integral_x = 0
        self.integral_y = 0

        

    def bot1_callback(self, msg):
        kp = 0.000000000000000000000001
        ki = 0.00000000000000000000000
        kd = 0.000000000000000000000000000000000000000
        pose1_x = msg.x
        pose1_y = msg.y
        pose1_theta = msg.theta
        global bot1_index
        global bot1_flag
        goal_x = (hex_pts[bot1_index])[0]
        goal_y = (hex_pts[bot1_index])[1]
        if bot1_index == (len(hex_pts) - 1):
            goal_xn = (hex_pts[0])[0]
            goal_yn = (hex_pts[0])[1]
        else:
            goal_xn = (hex_pts[bot1_index + 1])[0]
            goal_yn = (hex_pts[bot1_index + 1])[1]
        # goal_x = 100
        # goal_y = 150
        vel_x = goal_x - pose1_x
        vel_y = goal_y - pose1_y

        line = self.line_from_two_points(goal_xn, goal_yn, goal_x, goal_y)
        px, py = self.perpendicular_projection(pose1_x, pose1_y, line)
        correction_x = px - pose1_x
        correction_y = py - pose1_y

        self.integral_x += correction_x
        self.integral_y += correction_y
        derivative_x = correction_x - self.prev_error_x 
        derivative_y = correction_y - self.prev_error_y

        # vel_x = vel_x + correction_x*kp + self.integral_x*ki + derivative_x*kd
        # vel_y = vel_y + correction_y*kp + self.integral_y*ki + derivative_y*kd


        # vel_x = -100
        # vel_y = -100
        new_vel_x = (self.rot_pts(vel_x, vel_y, pose1_theta))[0]
        new_vel_y = (self.rot_pts(vel_x, vel_y, pose1_theta))[1]

        # new_vel_x = new_vel_xo + correction_x*kp + self.integral_x*ki + derivative_x*kd
        # new_vel_y = new_vel_yo + correction_y*kp + self.integral_y*ki + derivative_y*kd

        # print(new_vel_x, new_vel_y)
        v11 = (((self.inverse_kinematics(new_vel_x, new_vel_y, pose1_theta)[0])[0]))*0.18
        v22 = (((self.inverse_kinematics(new_vel_x, new_vel_y, pose1_theta)[1])[0]))*0.18
        v33 = (((self.inverse_kinematics(new_vel_x, new_vel_y, pose1_theta)[2])[0]))*0.18
        v1 = v11*10 + 90
        v2 = v22*10 + 90 
        v3 = v33*10 + 90 
        print(v1, v2, v3)

        if v1 >= 180:
            v1 = 180
        elif v1 <= 0:
            v1 = 0

        if v2 >= 180:
            v2 = 180
        elif v2 <= 0:
            v2 = 0

        if v3 >= 180:
            v3 = 180
        elif v3 <= 0:
            v3 = 0
      

        distance = math.sqrt(vel_x**2 + vel_y**2) 
        stop_threshold = 20.0

        if distance < stop_threshold and (pose1_theta > 350 or pose1_theta < 10):
            bot1 = Twist()
            bot1.linear.x = 90.0
            bot1.linear.y = 90.0
            bot1.linear.z = 90.0
            self.bot1_pub.publish(bot1)
        else:
            bot1 = Twist()
            bot1.linear.x = float(v1)
            bot1.linear.y = float(v2)
            bot1.linear.z = float(v3)
            self.bot1_pub.publish(bot1)

        if distance < stop_threshold:
            if bot1_index < (len(hex_pts)-1):
                bot1_index += 1
            else:
                bot1_index = 0
        print(bot1_index)

        if bot1_index == 0 and bot1_flag == 0 and distance < stop_threshold:
            bul = Bool()
            bul.data = True
            self.bot1_pen_pub.publish(bul)
            bot1_flag = 1
        elif bot1_index == 0 and bot1_flag == 1 and distance < stop_threshold:
            bul = Bool()
            bul.data = False
            self.bot1_pen_pub.publish(bul)
            bot1_flag = 0

        self.prev_error_x = correction_x
        self.prev_error_y = correction_y

    def bot2_callback(self, msg):
        pass

    def bot3_callback(self, msg):
        pass

    def inverse_kinematics(self, rotated_x, rotated_y, theta_r):
        r = 4.318 #change it nigga
        d = 26.136 #change it nigga
        theta =  (theta_r*math.pi)/180
        vel = np.array([[theta],
                        [rotated_x],
                        [-rotated_y]])
        cal_mat = np.array([[-d, 1.0, 0],
                            [-d, -0.5, -np.sin(np.pi/3) + 0.0],
                            [-d, -0.5, np.sin(np.pi/3) + 0.0]])
                
        inv_vel = (np.dot(cal_mat, vel))/r
        # v1 = inv_vel[0]
        # v2 = inv_vel[1]
        # v3 = inv_vel[2]
        return inv_vel


    def rot_pts(self,x,y,angle):
        angle_r = (angle*math.pi)/180
        c = np.cos(angle_r)
        s = np.sin(angle_r)

        rot_mat = np.array([[c, s], 
                            [-s, c]])
        pts_to_be_rot = np.array([[x],
                                  [y]])
        pts_rotated = np.dot(rot_mat, pts_to_be_rot)
        rotated_x = (pts_rotated[0])[0]
        rotated_y = (pts_rotated[1])[0]

        return rotated_x, rotated_y
    
    def line_from_two_points(self, x1, y1, x2, y2):
        A = y2 - y1
        B = x1 - x2
        C = x2 * y1 - x1 * y2

        return A, B, C

    def distance_from_point_to_line(self, x0, y0, A, B, C):
        # x0, y0 = point
        # A, B, C = line

        # Calculate the distance
        distance = abs(A * x0 + B * y0 + C) / math.sqrt(A**2 + B**2)

        return distance
    
    def perpendicular_projection(self, x0, y0, line):
        # x0, y0 = point
        A, B, C = line

        # Calculate the perpendicular projection
        if A == 0 or B == 0:
            A = 0.00000000000000000001
            B = 0.00000000000000000001
        projection_x = (B * (B * x0 - A * y0) - A * C) / (A**2 + B**2)
        projection_y = (A * (A * y0 - B * x0) - B * C) / (A**2 + B**2)

        return projection_x, projection_y

def main(args = None):
    rclpy.init(args=args)
    Controller_node = Controller()
    rclpy.spin(Controller_node)
    Controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()