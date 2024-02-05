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
tri_pts = [[300, 100], [400, 100], [300, 200], [300, 100]]
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
        self.bot2_pub = self.create_publisher(Twist, "/bot1_vel", 10)
        self.bot3_pub = self.create_publisher(Twist, "/bot3_vel", 10)
        self.bot1_pen_pub = self.create_publisher(Bool, "/pen2_down", 10)
        self.prev_error_x = 0
        self.prev_error_y = 0
        self.integral_x = 0
        self.integral_y = 0

        

    def bot1_callback(self, msg):
        # kp = 1.5
        # global bot1_index
        # goal_x = (hex_pts[bot1_index])[0]
        # goal_y = (hex_pts[bot1_index])[1]
        # goal_theta = 0
        # pose1_x = msg.x
        # pose1_y = msg.y
        # pose1_theta = msg.theta
        # if pose1_theta > 180:
        #     pose1_theta = pose1_theta - 360
        # vel_x = goal_x - pose1_x
        # vel_y = goal_y - pose1_y
        # err_theta = (goal_theta - pose1_theta)
        # print(err_theta)

        # new_vel_x = (self.rot_pts(vel_x, vel_y, pose1_theta))[0]
        # new_vel_y = (self.rot_pts(vel_x, vel_y, pose1_theta))[1]

        # v11 = (((self.inverse_kinematics(new_vel_x, new_vel_y, -err_theta)[0])[0]))*0.18
        # v22 = (((self.inverse_kinematics(new_vel_x, new_vel_y, -err_theta)[1])[0]))*0.18
        # v33 = (((self.inverse_kinematics(new_vel_x, new_vel_y, -err_theta)[2])[0]))*0.18
        # v1 = v11*15 + 90
        # v2 = v22*15 + 90 
        # v3 = v33*15 + 90 

        # if v1 >= 180:
        #     v1 = 180
        # elif v1 <= 0:
        #     v1 = 0

        # if v2 >= 180:
        #     v2 = 180
        # elif v2 <= 0:
        #     v2 = 0

        # if v3 >= 180:
        #     v3 = 180
        # elif v3 <= 0:
        #     v3 = 0

        
        # distance = math.sqrt(vel_x**2 + vel_y**2) 
        # stop_threshold = 20.0
        # if distance < stop_threshold and (pose1_theta > 350 or pose1_theta < 10):
        #     bot1 = Twist()
        #     bot1.linear.x = 90.0
        #     bot1.linear.y = 90.0
        #     bot1.linear.z = 90.0
        #     self.bot1_pub.publish(bot1)
        # else:
        #     bot1 = Twist()
        #     bot1.linear.x = float(v1)
        #     bot1.linear.y = float(v2)
        #     bot1.linear.z = float(v3)
        #     self.bot1_pub.publish(bot1)

        # if distance < stop_threshold:
        #     if bot1_index < (len(hex_pts)-1):
        #         bot1_index += 1
        #     else:
        #         bot1_index = 0
        # print("hex_index:", bot1_index)
        pass


    def bot2_callback(self, msg):
        # kp = 1.5
        # global bot2_index
        # goal_x = (tri_pts[bot2_index])[0]
        # goal_y = (tri_pts[bot2_index])[1]
        # goal_theta = 0
        # pose1_x = msg.x
        # pose1_y = msg.y
        # pose1_theta = msg.theta
        # if pose1_theta > 180:
        #     pose1_theta = pose1_theta - 360
        # vel_x = goal_x - pose1_x
        # vel_y = goal_y - pose1_y
        # err_theta = (goal_theta - pose1_theta)
        # print(err_theta)

        # new_vel_x = (self.rot_pts(vel_x, vel_y, pose1_theta))[0]
        # new_vel_y = (self.rot_pts(vel_x, vel_y, pose1_theta))[1]

        # v11 = (((self.inverse_kinematics(new_vel_x, new_vel_y, -err_theta)[0])[0]))*0.18
        # v22 = (((self.inverse_kinematics(new_vel_x, new_vel_y, -err_theta)[1])[0]))*0.18
        # v33 = (((self.inverse_kinematics(new_vel_x, new_vel_y, -err_theta)[2])[0]))*0.18
        # v1 = v11*15 + 90
        # v2 = v22*15 + 90 
        # v3 = v33*15 + 90 

        # if v1 >= 180:
        #     v1 = 180
        # elif v1 <= 0:
        #     v1 = 0

        # if v2 >= 180:
        #     v2 = 180
        # elif v2 <= 0:
        #     v2 = 0

        # if v3 >= 180:
        #     v3 = 180
        # elif v3 <= 0:
        #     v3 = 0

        
        # distance = math.sqrt(vel_x**2 + vel_y**2) 
        # stop_threshold = 20.0
        # if distance < stop_threshold and (pose1_theta > 350 or pose1_theta < 10):
        #     bot1 = Twist()
        #     bot1.linear.x = 90.0
        #     bot1.linear.y = 90.0
        #     bot1.linear.z = 90.0
        #     self.bot2_pub.publish(bot1)
        # else:
        #     bot1 = Twist()
        #     bot1.linear.x = float(v1)
        #     bot1.linear.y = float(v2)
        #     bot1.linear.z = float(v3)
        #     self.bot2_pub.publish(bot1)

        # if distance < stop_threshold:
        #     if bot2_index < (len(tri_pts)-1):
        #         bot2_index += 1
        #     else:
        #         bot2_index = 0
        # print("tri_index:", bot2_index)
        pass

    def bot3_callback(self, msg):
        kp = 1.5
        global bot3_index
        # goal_x = (rect_pts[bot3_index])[0]
        # goal_y = (rect_pts[bot3_index])[1]
        goal_x = 100
        goal_y = 400
        goal_theta = 0
        pose1_x = msg.x
        pose1_y = msg.y
        pose1_theta = msg.theta
        if pose1_theta > 180:
            pose1_theta = pose1_theta - 360
        vel_x = goal_x - pose1_x
        vel_y = goal_y - pose1_y
        err_theta = (goal_theta - pose1_theta)
        print(err_theta)

        new_vel_x = (self.rot_pts(vel_x, vel_y, pose1_theta))[0]
        new_vel_y = (self.rot_pts(vel_x, vel_y, pose1_theta))[1]

        v11 = (((self.inverse_kinematics(new_vel_x, new_vel_y, -err_theta)[0])[0]))*0.18
        v22 = (((self.inverse_kinematics(new_vel_x, new_vel_y, -err_theta)[1])[0]))*0.18
        v33 = (((self.inverse_kinematics(new_vel_x, new_vel_y, -err_theta)[2])[0]))*0.18
        v1 = v11*20 + 90
        v2 = v22*20 + 90
        v3 = v33*20 + 90 

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
            self.bot3_pub.publish(bot1)
        else:
            bot1 = Twist()
            bot1.linear.x = float(v1)
            bot1.linear.y = float(v2)
            bot1.linear.z = float(v3)
            self.bot3_pub.publish(bot1)

        if distance < stop_threshold:
            if bot3_index < (len(rect_pts)-1):
                bot3_index += 1
            else:
                bot3_index = 0
        print("rect_index:", bot3_index)
        

    def inverse_kinematics(self, rotated_x, rotated_y, theta_r):
        r = 4.318 
        d = 26.136 
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
    
def main(args = None):
    rclpy.init(args=args)
    Controller_node = Controller()
    rclpy.spin(Controller_node)
    Controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()