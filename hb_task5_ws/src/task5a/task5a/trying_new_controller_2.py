import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool
import math
import numpy as np
from std_srvs.srv import Empty
hex_pts = [[200, 150], [175, 200], [125, 200], [100, 150], [125, 100], [175, 100],[200, 150]]
# hex_pts = [[400, 250], [100, 250]]
tri_pts = [[300, 100], [400, 100], [300, 200], [300, 100]]
rect_pts = [[200, 300], [400, 300], [400, 400], [200, 400], [200, 300]]
t1 = np.linspace((2*np.pi)/3, (4*np.pi)/3, 100)
x1 = (200*np.cos(t1)) + 250 - 2.5
y1 = (-150*np.sin(4*t1)) + 250 - 25.45

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
        self.pen1_subcriber = self.create_subscription(Pose2D, "/pen22_pose", self.bot1_callback, 100)
        # self.pen2_subcriber = self.create_subscription(Pose2D, "/pen22_pose", self.bot2_callback, 10)
        # self.pen3_subcriber = self.create_subscription(Pose2D, "/pen33_pose", self.bot3_callback, 10)

        self.bot1_pub = self.create_publisher(Twist, "/bot2_vel", 10)
        # self.bot2_pub = self.create_publisher(Twist, "/bot2_vel", 10)
        # self.bot3_pub = self.create_publisher(Twist, "/bot1_vel", 10)
        self.bot1_pen_pub = self.create_publisher(Bool, "/pen2_down", 10)
        # self.bot2_pen_pub = self.create_publisher(Bool, "/pen2_down", 10)
        # self.bot3_pen_pub = self.create_publisher(Bool, "/pen3_down", 10)
        self.pose1_theta = 0
        self.pose1_x = 0
        self.pose1_y = 0


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

    def reset_serv(self):
       client2 = self.create_client(Empty, "/Stop_Flag")
       while not client2.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service")
       request = Empty.Request()
       client2.call_async(request)

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

def pen_pose_point(xc,yc,x,y,angle_d):
  # angle = (angle_d*math.pi)/180
  angle = angle_d
  c = np.cos(angle)
  s = np.sin(angle)

  rot_mat = np.array([[c, -s], 
                      [s, c]])
  pts_to_be_rot = np.array([[x],
                            [-y]])
  pts_rotated = np.dot(rot_mat, pts_to_be_rot)
  rotated_x = (pts_rotated[0])[0]
  rotated_y = (pts_rotated[1])[0]

  penpose_x = xc + rotated_x
  penpose_y = yc - rotated_y

  return penpose_x, penpose_y


def main(args=None):
    rclpy.init(args=args)

    # Create an instance of the EbotController class
    con = Controller()
   
    # Send an initial request with the index from ebot_controller.index
    # con.send_request(con.index)
    
    # Main loop
    while (1):
        #########################################################################################################################
        kd = 0.0006
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

        new_vel_x = (rot_pts(vel_x, vel_y, pose1_theta))[0] + kd*diff_x
        new_vel_y = (rot_pts(vel_x, vel_y, pose1_theta))[1] + kd*diff_y

        v11 = (((inverse_kinematics(new_vel_x, new_vel_y, -err_theta-10)[0])[0]))*0.18
        v22 = (((inverse_kinematics(new_vel_x, new_vel_y, -err_theta-10)[1])[0]))*0.18
        v33 = (((inverse_kinematics(new_vel_x, new_vel_y, -err_theta-10)[2])[0]))*0.18
        v1 = v11*20 + 92.5
        v2 = v22*20 + 91.5
        v3 = v33*20 + 92.5 

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
            con.bot1_pub.publish(bot1)
        else:
            bot1 = Twist()
            bot1.linear.x = float(v1)
            bot1.linear.y = float(v2)
            bot1.linear.z = float(v3)
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

        if distance < stop_threshold:
            if bot1_index < (len(x1)-1):
                bot1_index += 1
            elif bot1_index == (len(x1)-1):
                bot1 = Twist()
                bot1.linear.x = 90.0
                bot1.linear.y = 90.0
                bot1.linear.z = 90.0
                con.bot1_pub.publish(bot1)
                stop1_flag = 1
            else:
                bot1_index = 0

            if (stop1_flag == 1 and stop2_flag == 1 and stop3_flag == 1):
                con.reset_serv()
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