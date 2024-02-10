import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool
import math
import numpy as np
from std_srvs.srv import Empty
t1 = np.linspace((0*np.pi)/3, (6*np.pi)/3, 150)
t2 = np.linspace((2*np.pi)/3, (4*np.pi)/3, 50)
t3 = np.linspace((4*np.pi)/3, (6*np.pi)/3, 50)
x1 = (200*np.cos(t1)) + 250 + 2.5
y1 = (-150*np.sin(4*t1)) + 250 + 25.45
x2 = (200*np.cos(t2)) + 250
y2 = (-150*np.sin(4*t2)) + 250
x3 = (200*np.cos(t3)) + 250
y3 = (-150*np.sin(4*t3)) + 250
bot1_index = 0
bot2_index = 0
bot3_index = 0
bot1_prev_index = 0
bot2_prev_index = 0
bot3_prev_index = 0
bot1_prev_vel_x = 0
bot1_prev_vel_y = 0
bot1_prev_vel_z = 0
stop1_flag = 0
stop2_flag = 0
stop3_flag = 0
kp_ang = 1.0
kp_l = 1.0


class Controller(Node):
    def __init__(self):
        super().__init__('Controller')
        self.pen1_subcriber = self.create_subscription(Pose2D, "/pen11_pose", self.bot1_callback, 10)
        self.pen2_subcriber = self.create_subscription(Pose2D, "/pen22_pose", self.bot2_callback, 10)
        self.pen3_subcriber = self.create_subscription(Pose2D, "/pen33_pose", self.bot3_callback, 10)

        self.bot1_pub = self.create_publisher(Twist, "/bot1_vel", 10)
        self.bot2_pub = self.create_publisher(Twist, "/bot2_vel", 10)
        self.bot3_pub = self.create_publisher(Twist, "/bot3_vel", 10)
        self.bot1_pen_pub = self.create_publisher(Bool, "/pen1_down", 10)
        self.bot2_pen_pub = self.create_publisher(Bool, "/pen2_down", 10)
        self.bot3_pen_pub = self.create_publisher(Bool, "/pen3_down", 10)
        self.prev_error_x = 0
        self.prev_error_y = 0
        self.integral_x = 0
        self.integral_y = 0

        

    def bot1_callback(self, msg):
        
        global bot1_index
        global bot1_prev_index
        global bot1_prev_vel_x
        global bot1_prev_vel_y
        global bot1_prev_vel_z
        global stop1_flag
        global kp_l
        global kp_ang
        goal_x = (x1[bot1_index])
        goal_y = (y1[bot1_index])
        goal_theta = 0
        pose1_x = msg.x
        pose1_y = msg.y
        pose1_theta = msg.theta
        if pose1_theta > 180:
            pose1_theta = pose1_theta - 360
        # goal_x = (self.anti_pen_pose_point(goal_xp, goal_yp, 2.5, 25.45, pose1_theta))[0]
        # goal_y = (self.anti_pen_pose_point(goal_xp, goal_yp, 2.5, 25.45, pose1_theta))[1]
        vel_x = goal_x - pose1_x
        vel_y = goal_y - pose1_y
        err_theta = (goal_theta - pose1_theta)
        print(err_theta)

        new_vel_x = (self.rot_pts(vel_x, vel_y, pose1_theta))[0] 
        new_vel_y = (self.rot_pts(vel_x, vel_y, pose1_theta))[1] 

        v11 = (((self.inverse_kinematics(new_vel_x, new_vel_y, (-err_theta-10))[0])[0]))*0.18
        v22 = (((self.inverse_kinematics(new_vel_x, new_vel_y, (-err_theta-10))[1])[0]))*0.18
        v33 = (((self.inverse_kinematics(new_vel_x, new_vel_y, (-err_theta-10))[2])[0]))*0.18
        v1 = v11*15 + 90
        v2 = v22*15 + 90 
        v3 = v33*15 + 90 

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
        if distance > stop_threshold and (pose1_theta < 350 or pose1_theta > 10):
            bot1 = Twist()
            bot1.linear.x = float(v1)
            bot1.linear.y = float(v2)
            bot1.linear.z = float(v3)
            self.bot1_pub.publish(bot1)
        else:
            bot1 = Twist()
            # bot1.linear.x = bot1_prev_vel_x
            # bot1.linear.y = bot1_prev_vel_y
            # bot1.linear.z = bot1_prev_vel_z
            bot1.linear.x = 90.0
            bot1.linear.y = 90.0
            bot1.linear.z = 90.0
            self.bot1_pub.publish(bot1)


        if bot1_prev_index == 0 and bot1_index == 0:
            bul = Bool()
            bul.data = False
            self.bot1_pen_pub.publish(bul)
        else:
            bul = Bool()
            bul.data = True
            self.bot1_pen_pub.publish(bul)

        bot1_prev_index = bot1_index
        bot1_prev_vel_x = float(v1)
        bot1_prev_vel_y = float(v2)
        bot1_prev_vel_z = float(v3)

        if distance < stop_threshold:
            if bot1_index < (len(x1)-1):
                bot1_index += 1
            elif bot1_index == (len(x1)-1):
                bot1 = Twist()
                bot1.linear.x = 90.0
                bot1.linear.y = 90.0
                bot1.linear.z = 90.0
                self.bot1_pub.publish(bot1)
                stop1_flag = 1
            else:
                bot1_index = 0

            if (stop1_flag == 1 and stop2_flag == 1 and stop3_flag == 1):
                self.reset_serv()
        print("x1_index:", bot1_index)
        
        # print(y1)

    def bot2_callback(self, msg):
        
        global bot2_index
        global bot2_prev_index
        global bot2_prev_vel_x
        global bot2_prev_vel_y
        global bot2_prev_vel_z
        global stop2_flag
        global kp_l
        global kp_ang
        goal_xp = (x2[bot2_index])
        goal_yp = (y2[bot2_index])
        goal_theta = 0
        pose1_x = msg.x
        pose1_y = msg.y
        pose1_theta = msg.theta
        if pose1_theta > 180:
            pose1_theta = pose1_theta - 360
        goal_x = (self.anti_pen_pose_point(goal_xp, goal_yp, 2.5, 25.45, pose1_theta))[0]
        goal_y = (self.anti_pen_pose_point(goal_xp, goal_yp, 2.5, 25.45, pose1_theta))[1]
        vel_x = goal_x - pose1_x
        vel_y = goal_y - pose1_y
        err_theta = (goal_theta - pose1_theta)
        print(err_theta)

        new_vel_x = (self.rot_pts(vel_x, vel_y, pose1_theta))[0] * kp_l
        new_vel_y = (self.rot_pts(vel_x, vel_y, pose1_theta))[1] * kp_l

        v11 = (((self.inverse_kinematics(new_vel_x, new_vel_y, (-err_theta-10)*kp_ang)[0])[0]))*0.18
        v22 = (((self.inverse_kinematics(new_vel_x, new_vel_y, (-err_theta-10)*kp_ang)[1])[0]))*0.18
        v33 = (((self.inverse_kinematics(new_vel_x, new_vel_y, (-err_theta-10)*kp_ang)[2])[0]))*0.18
        v1 = v11*15 + 90
        v2 = v22*15 + 90 
        v3 = v33*15 + 90 

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
        stop_threshold = 10.0
        if distance > stop_threshold and (pose1_theta < 350 or pose1_theta > 10):
            bot1 = Twist()
            bot1.linear.x = float(v1)
            bot1.linear.y = float(v2)
            bot1.linear.z = float(v3)
            self.bot2_pub.publish(bot1)
        else:
            bot1 = Twist()
            # bot1.linear.x = bot1_prev_vel_x
            # bot1.linear.y = bot1_prev_vel_y
            # bot1.linear.z = bot1_prev_vel_z
            bot1.linear.x = 90.0
            bot1.linear.y = 90.0
            bot1.linear.z = 90.0
            self.bot2_pub.publish(bot1)


        if bot2_prev_index == 0 and bot2_index == 0:
            bul = Bool()
            bul.data = False
            self.bot2_pen_pub.publish(bul)
        else:
            bul = Bool()
            bul.data = True
            self.bot2_pen_pub.publish(bul)

        bot2_prev_index = bot2_index
        bot2_prev_vel_x = float(v1)
        bot2_prev_vel_y = float(v2)
        bot2_prev_vel_z = float(v3)

        if distance < stop_threshold:
            if bot2_index < (len(x2)-1):
                bot2_index += 1
            elif bot2_index == (len(x2)-1):
                bot1 = Twist()
                bot1.linear.x = 90.0
                bot1.linear.y = 90.0
                bot1.linear.z = 90.0
                self.bot2_pub.publish(bot1)
                stop2_flag = 1
            else:
                bot2_index = 0

            if (stop1_flag == 1 and stop2_flag == 1 and stop3_flag == 1):
                self.reset_serv()
        print("x2_index:", bot2_index)
        
        
        

    def bot3_callback(self, msg):
        
        # global bot3_index
        # global bot3_prev_index
        # global bot3_prev_vel_x
        # global bot3_prev_vel_y
        # global bot3_prev_vel_z
        # global stop3_flag
        # global kp_l
        # global kp_ang
        # goal_xp = (x3[bot3_index])
        # goal_yp = (y3[bot3_index])
        # goal_theta = 0
        # pose1_x = msg.x
        # pose1_y = msg.y
        # pose1_theta = msg.theta
        # if pose1_theta > 180:
        #     pose1_theta = pose1_theta - 360
        # goal_x = (self.anti_pen_pose_point(goal_xp, goal_yp, 2.5, 25.45, pose1_theta))[0] 
        # goal_y = (self.anti_pen_pose_point(goal_xp, goal_yp, 2.5, 25.45, pose1_theta))[1] 
        # vel_x = goal_x - pose1_x
        # vel_y = goal_y - pose1_y
        # err_theta = (goal_theta - pose1_theta)
        # print(err_theta)

        # new_vel_x = (self.rot_pts(vel_x, vel_y, pose1_theta))[0] * kp_l
        # new_vel_y = (self.rot_pts(vel_x, vel_y, pose1_theta))[1] * kp_l

        # v11 = (((self.inverse_kinematics(new_vel_x, new_vel_y, (-err_theta-10)*kp_ang)[0])[0]))*0.18
        # v22 = (((self.inverse_kinematics(new_vel_x, new_vel_y, (-err_theta-10)*kp_ang)[1])[0]))*0.18
        # v33 = (((self.inverse_kinematics(new_vel_x, new_vel_y, (-err_theta-10)*kp_ang)[2])[0]))*0.18
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
        # stop_threshold = 10.0
        # if distance > stop_threshold and (pose1_theta < 350 or pose1_theta > 10):
        #     bot1 = Twist()
        #     bot1.linear.x = float(v1)
        #     bot1.linear.y = float(v2)
        #     bot1.linear.z = float(v3)
        #     self.bot3_pub.publish(bot1)
        # else:
        #     bot1 = Twist()
        #     # bot1.linear.x = bot1_prev_vel_x
        #     # bot1.linear.y = bot1_prev_vel_y
        #     # bot1.linear.z = bot1_prev_vel_z
        #     bot1.linear.x = 90.0
        #     bot1.linear.y = 90.0
        #     bot1.linear.z = 90.0
        #     self.bot3_pub.publish(bot1)


        # if bot3_prev_index == 0 and bot3_index == 0:
        #     bul = Bool()
        #     bul.data = False
        #     self.bot3_pen_pub.publish(bul)
        # else:
        #     bul = Bool()
        #     bul.data = True
        #     self.bot3_pen_pub.publish(bul)

        # bot3_prev_index = bot3_index
        # bot1_prev_vel_x = float(v1)
        # bot1_prev_vel_y = float(v2)
        # bot1_prev_vel_z = float(v3)

        # if distance < stop_threshold:
        #     if bot3_index < (len(x3)-1):
        #         bot3_index += 1
        #     elif bot3_index == (len(x3)-1):
        #         bot1 = Twist()
        #         bot1.linear.x = 90.0
        #         bot1.linear.y = 90.0
        #         bot1.linear.z = 90.0
        #         self.bot3_pub.publish(bot1)
        #         stop3_flag = 1
        #     else:
        #         bot3_index = 0
            
        #     if (stop1_flag == 1 and stop2_flag == 1 and stop3_flag == 1):
        #         self.reset_serv()
        # print("x3_index:", bot3_index)
        pass
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
    
    def anti_pen_pose_point(self,xc,yc,x,y,angle_d):
        angle = (angle_d*math.pi)/180
        c = np.cos(angle)
        s = np.sin(angle)

        rot_mat = np.array([[c, s], 
                            [-s, c]])
        pts_to_be_rot = np.array([[x],
                                  [y]])
        pts_rotated = np.dot(rot_mat, pts_to_be_rot)
        rotated_x = (pts_rotated[0])[0]
        rotated_y = (pts_rotated[1])[0]
        
        penpose_x = xc + rotated_x
        penpose_y = yc + rotated_y

        return penpose_x, penpose_y
    
    def reset_serv(self):
       client2 = self.create_client(Empty, "Stop_Flag")
       while not client2.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service")
       request = Empty.Request()
       client2.call_async(request)
    
    
    
def main(args = None):
    rclpy.init(args=args)
    Controller_node = Controller()
    rclpy.spin(Controller_node)
    Controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()