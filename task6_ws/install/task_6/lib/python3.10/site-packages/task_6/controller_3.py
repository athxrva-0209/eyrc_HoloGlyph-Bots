import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool
import math
import numpy as np
from std_srvs.srv import Empty
t1 = np.linspace((4*np.pi)/3, (6*np.pi)/3, 50)
x1 = (220*np.cos(4*t1)*np.cos(t1)) + (250 - 2.5)
y1 = (-220*np.cos(4*t1)*np.sin(t1)) + (250 - 25.45) + 15
# x1 = 100*np.cos(t1) + (250 - 2.5)
# y1 = 100*np.sin(t1) + (250 - 25.45)
# pose1_x = 0 
# pose1_y = 0
# pose1_theta = 0
bot1_index = 0
bot2_index = 0
bot3_index = 0
bot1_prev_index = 0
bot2_prev_index = 0
bot3_prev_index = 0
stop1_flag = 0
stop2_flag = 0
stop3_flag = 0
last_err_x = 0
last_err_y = 0


class Controller(Node):
    def __init__(self):
        super().__init__('Controller')
        self.pen1_subcriber = self.create_subscription(Pose2D, "/pen33_pose", self.bot1_callback, 10)
        self.pen2_subcriber = self.create_subscription(Pose2D, "/pen22_pose", self.bot2_callback, 10)
        self.pen3_subcriber = self.create_subscription(Pose2D, "/pen11_pose", self.bot3_callback, 10)

        self.bot1_pen_sub = self.create_subscription(Bool, "/pen1_down", self.pen1down_cb, 10)
        self.bot2_pen_sub = self.create_subscription(Bool, "/pen2_down", self.pen2down_cb, 10)
        self.bot3_pen_sub = self.create_subscription(Bool, "/pen3_down", self.pen3down_cb, 10)

        # self.bot1_flag_sub = self.create_subscription(Bool, "/bot1_flag", self.bot1_flag_callback, 10)
        # self.bot2_flag_sub = self.create_subscription(Bool, "/bot2_flag", self.bot2_flag_callback, 10)
        # self.bot3_flag_sub = self.create_subscription(Bool, "/bot3_flag", self.bot3_flag_callback, 10)

        self.bot1_pub = self.create_publisher(Twist, "/bot3_vel", 10)
        # self.bot3_flag_pub = self.create_publisher(Bool, "/bot3_flag", 10)
        # self.bot2_pub = self.create_publisher(Twist, "/bot2_vel", 10)
        # self.bot3_pub = self.create_publisher(Twist, "/bot1_vel", 10)
        self.bot1_pen_pub = self.create_publisher(Bool, "/pen3_down", 10)
        # self.bot2_pen_pub = self.create_publisher(Bool, "/pen2_down", 10)
        # self.bot3_pen_pub = self.create_publisher(Bool, "/pen3_down", 10)
        self.pose1_theta = 0
        self.pose1_x = 0
        self.pose1_y = 0
        self.pose3_theta = 0
        self.pose3_x = 0
        self.pose3_y = 0
        self.pose2_theta = 0
        self.pose2_x = 0
        self.pose2_y = 0
        self.bot1_flag = 0
        self.bot2_flag = 0
        self.bot3_flag = 0
        self.pen1 = 0
        self.pen2 = 0
        self.pen3 = 0


    def bot1_callback(self, msg):
        # global pose1_x, pose1_y, pose1_theta
        self.pose1_x = msg.x
        self.pose1_y = msg.y
        self.pose1_theta = msg.theta

    def bot2_callback(self, msg):
        # global pose1_x, pose1_y, pose1_theta
        self.pose2_x = msg.x
        self.pose2_y = msg.y
        self.pose2_theta = msg.theta

    def bot3_callback(self, msg):
        # global pose1_x, pose1_y, pose1_theta
        self.pose3_x = msg.x
        self.pose3_y = msg.y
        self.pose3_theta = msg.theta

    def pen1down_cb(self, data):
        self.pen1 = data.data

    def pen2down_cb(self, data):
        self.pen2 = data.data

    def pen3down_cb(self, data):
        self.pen3 = data.data

    

    def reset_serv(self):
        client = self.create_client(Empty, "/Stop_Flag")
        if not client.service_is_ready():
            self.get_logger().warn("Service 'Stop_Flag' is not ready")
            return

        request = Empty.Request()
        future = client.call_async(request)
        future.add_done_callback(self.reset_serv_callback)

    def reset_serv_callback(self, future):
        if future.result() is not None:
            self.get_logger().info("Service 'Stop_Flag' called successfully")
        else:
            self.get_logger().error("Failed to call service 'Stop_Flag'")

def rot_pts(x,y,angle):
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

def inverse_kinematics(rotated_x, rotated_y, theta_r):
        r = 4.318 
        d = 25.00 
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


def main(args=None):
    rclpy.init(args=args)

    # Create an instance of the EbotController class
    con = Controller()
   
    # Send an initial request with the index from ebot_controller.index
    # con.send_request(con.index)
    # Main loop
    while (1):

        cx2 = con.pose2_x - con.pose1_x
        cy2 = con.pose2_y - con.pose1_y
        co_dist_2 = math.sqrt(cx2**2 + cy2**2)
        if co_dist_2 < 110:
            bot1 = Twist()
            bot1.linear.x = 90.0
            bot1.linear.y = 90.0
            bot1.linear.z = 90.0
            con.bot1_pub.publish(bot1)
        else:
        
        #########################################################################################################################
            kd = 0.00006
            kp_ang = 1.5
            global bot1_index, bot1_prev_index, bot1_index, stop1_flag, stop2_flag, stop3_flag, last_err_x, last_err_y
            goal_x = (x1[bot1_index])
            goal_y = (y1[bot1_index])
            goal_theta = 0
            pose1_theta = con.pose1_theta
            if pose1_theta > 180:
                pose1_theta = pose1_theta - 360
            # goal_x = pen_pose_point(goal_x, goal_y, 2.5, 25.45, pose1_theta)[0]
            # goal_y = pen_pose_point(goal_x, goal_y, 2.5, 25.45, pose1_theta)[1]
            vel_x = goal_x - con.pose1_x
            vel_y = goal_y - con.pose1_y
            err_theta = (goal_theta - pose1_theta)
            err_x = goal_x - con.pose1_x
            err_y = goal_y - con.pose1_y
            diff_x = err_x - last_err_x
            diff_y = err_y - last_err_y
            # print(err_theta)

            new_vel_x = (rot_pts(vel_x, vel_y, pose1_theta))[0] +kd*diff_x
            new_vel_y = (rot_pts(vel_x, vel_y, pose1_theta))[1] +kd*diff_y

            v11 = (((inverse_kinematics(new_vel_x, new_vel_y, -(err_theta)*kp_ang)[0])[0]))
            v22 = (((inverse_kinematics(new_vel_x, new_vel_y, -(err_theta)*kp_ang)[1])[0]))
            v33 = (((inverse_kinematics(new_vel_x, new_vel_y, -(err_theta)*kp_ang)[2])[0]))
            v1 = v11*1.5 + 90
            v2 = v22*1.5 + 90
            v3 = v33*1.5 + 90 
            # print (v1, v2, v3)

            # if v1 < 98 and v1 > 91:
            #     v1 = 98
            # elif v1 > 87 and v1 < 89:
            #     v1 = 87

            # if v2 < 97 and v2 > 91:
            #     v2 = 98
            # elif v2 > 86 and v2 < 89:
            #     v2 = 87

            # if v3 < 98 and v3 > 91:
            #     v3 = 98
            # elif v3 > 87 and v3 < 89:
            #     v3 = 87

            distance = math.sqrt(vel_x**2 + vel_y**2) 
            stop_threshold = 20.0
            if distance > stop_threshold:
                bot1 = Twist()
                bot1.linear.x = float(v1)
                bot1.linear.y = float(v2)
                bot1.linear.z = float(v3)
                con.bot1_pub.publish(bot1)
            else:
                bot1 = Twist()
                bot1.linear.x = 90.0
                bot1.linear.y = 90.0
                bot1.linear.z = 90.0
                con.bot1_pub.publish(bot1)

            if bot1_prev_index == 0 and bot1_index == 0:
                bul = Bool()
                bul.data = False
                con.bot1_pen_pub.publish(bul)
                
            else:
                bul = Bool()
                bul.data = True
                con.bot1_pen_pub.publish(bul)
                

            bot1_prev_index = bot1_index

            if distance < stop_threshold and con.pen1 and con.pen2 and con.pen3:
                if bot1_index < (len(x1)-1):
                    bot1_index += 1
                elif bot1_index == (len(x1)-1):
                    bot1 = Twist()
                    bot1.linear.x = 90.0
                    bot1.linear.y = 90.0
                    bot1.linear.z = 90.0
                    con.bot1_pub.publish(bot1)
                    stop1_flag = 1
                    con.reset_serv()
                else:
                    bot1_index = 0
            elif distance < stop_threshold and bot1_index == 0:
                bot1_index += 1

                # if (stop1_flag == 1 and stop2_flag == 1 and stop3_flag == 1):
                #     con.reset_serv()
            print("x1_index:", bot1_index)
            last_err_x = err_x
            last_err_y = err_y
            ############################################################################################################################
        

        # Spin once to process callbacks
        rclpy.spin_once(con)
            
    
    # Destroy the node and shut down ROS
    con.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()