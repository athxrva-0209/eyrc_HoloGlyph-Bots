import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
import math
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv2 import aruco
import cv2 as cv
from std_msgs.msg import Int32
from sensor_msgs.msg import Image

bot1_prev = [[[[0],[0]],[[0],[0]],[[0],[0]],[[0],[0]]]]
bot2_prev = [[[[0],[0]],[[0],[0]],[[0],[0]],[[0],[0]]]]
bot3_prev = [[[[0],[0]],[[0],[0]],[[0],[0]],[[0],[0]]]]
nbot1_prev = [[[[0.1],[0.2]],[[0.3],[0.4]],[[0.5],[0.6]],[[0.7],[0.8]]]]
nbot2_prev = [[[[0.17],[0.16]],[[0.15],[0.14]],[[0.13],[0.12]],[[0.11],[0.9]]]]
nbot3_prev = [[[[0.18],[0.19]],[[0.20],[0.21]],[[0.22],[0.23]],[[0.24],[0.25]]]]
ref1_prev = [[[[0],[0]],[[0],[0]],[[0],[0]],[[0],[0]]]]
ref2_prev = [[[[0],[0]],[[0],[0]],[[0],[0]],[[0],[0]]]]
ref3_prev = [[[[0],[0]],[[0],[0]],[[0],[0]],[[0],[0]]]]
ref4_prev = [[[[0],[0]],[[0],[0]],[[0],[0]],[[0],[0]]]]
max_length = 100

class Publisher(Node):
    def __init__(self):
        super().__init__('Penposepub')
        self.image_subcriber = self.create_subscription(Image, "/image_rect_color", self.image_callback, 10)
        self.aruco_pose_pub_1 = self.create_publisher(Pose2D, "/pen11_pose", 10)
        self.aruco_pose_pub_2 = self.create_publisher(Pose2D, "/pen22_pose", 10)
        self.aruco_pose_pub_3 = self.create_publisher(Pose2D, "/pen33_pose", 10)
        self.aruco_pose_pub_1_eval = self.create_publisher(Pose2D, "/pen1_pose", 10)
        self.aruco_pose_pub_2_eval = self.create_publisher(Pose2D, "/pen2_pose", 10)
        self.aruco_pose_pub_3_eval = self.create_publisher(Pose2D, "/pen3_pose", 10)
        self.cv_bridge = CvBridge()
        self.marker_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_1000)
        self.param_markers = aruco.DetectorParameters()
        self.detector = cv.aruco.ArucoDetector(self.marker_dict, self.param_markers)
        self.detector1 = cv.aruco.ArucoDetector(self.marker_dict, self.param_markers)
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        

    def image_callback(self, msg):
        
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.corners, self.ids, rejected = self.detector.detectMarkers(cv_image)
        # cv.aruco.drawDetectedMarkers(cv_image, self.corners, self.ids)
        self.display_image(cv_image, "org_img")
        # print(self.ids)

        bot1_marker_id = 1
        bot2_marker_id = 2
        bot3_marker_id = 3  

        ref1_marker_id = 8
        ref2_marker_id = 10
        ref3_marker_id = 12
        ref4_marker_id = 4 

        bot1_corners = [[[0.1,0],[0.1,0],[0.11,0],[0.1,0]]]
        bot2_corners = [[[0.2,0],[0.2,0],[0.21,0],[0.2,0]]]
        bot3_corners = [[[0.3,0],[0.3,0],[0.31,0],[0.3,0]]]
        
        ref1_corners = [[[0,0],[0.4,0],[0.41,0],[0.4,0]]]
        ref3_corners = [[[500,500],[0.5,0],[0.51,0],[0.5,0]]]
        ref2_corners = [[[0,0],[500,0],[0.41,0],[0.4,0]]]
        ref4_corners = [[[0.3,0.3],[0.5,0],[0.51,0],[0,500]]]



        if self.ids is not None:
            for i in range(len(self.ids)):
                if self.ids[i] == bot1_marker_id:
                    bot1_corners = self.corners[i]
                    bot1_prev.append(bot1_corners)

            for i in range(len(self.ids)):
                if self.ids[i] == bot2_marker_id:
                    bot2_corners = self.corners[i]
                    bot2_prev.append(bot2_corners)

            for i in range(len(self.ids)):
                if self.ids[i] == bot3_marker_id:
                    bot3_corners = self.corners[i]
                    bot3_prev.append(bot3_corners)

            for i in range(len(self.ids)):
                if self.ids[i] == ref1_marker_id:
                    ref1_corners = self.corners[i]
                    ref1_prev.append(ref1_corners)

            for i in range(len(self.ids)):
                if self.ids[i] == ref2_marker_id:
                    ref2_corners = self.corners[i]
                    ref2_prev.append(ref2_corners)

            for i in range(len(self.ids)):
                if self.ids[i] == ref3_marker_id:
                    ref3_corners = self.corners[i]
                    ref3_prev.append(ref3_corners)
            
            for i in range(len(self.ids)):
                if self.ids[i] == ref4_marker_id:
                    ref4_corners = self.corners[i]
                    ref4_prev.append(ref4_corners)

            if bot1_corners is not None:
                
                x11 = ((bot1_corners[0])[0])[0]
                y11 = ((bot1_corners[0])[0])[1]
                x13 = ((bot1_corners[0])[2])[0]
                y13 = ((bot1_corners[0])[2])[1]
                
            else:
                for element in bot1_prev:
                    if element is not None:
                        bot1_corners = element
                x11 = ((bot1_corners[0])[0])[0]
                y11 = ((bot1_corners[0])[0])[1]
                x13 = ((bot1_corners[0])[2])[0]
                y13 = ((bot1_corners[0])[2])[1]



            if bot2_corners is not None:
                
                x21 = ((bot2_corners[0])[0])[0]
                y21 = ((bot2_corners[0])[0])[1]
                x23 = ((bot2_corners[0])[2])[0]
                y23 = ((bot2_corners[0])[2])[1]
                
            else:
                for element in bot2_prev:
                    if element is not None:
                        bot2_corners = element
                x21 = ((bot2_corners[0])[0])[0]
                y21 = ((bot2_corners[0])[0])[1]
                x23 = ((bot2_corners[0])[2])[0]
                y23 = ((bot2_corners[0])[2])[1]

            

            if bot3_corners is not None:
                
                x31 = ((bot3_corners[0])[0])[0]
                y31 = ((bot3_corners[0])[0])[1]
                x33 = ((bot3_corners[0])[2])[0]
                y33 = ((bot3_corners[0])[2])[1]
                
            else:
                for element in bot3_prev:
                    if element is not None:
                        bot3_corners = element
                x31 = ((bot3_corners[0])[0])[0]
                y31 = ((bot3_corners[0])[0])[1]
                x33 = ((bot3_corners[0])[2])[0]
                y33 = ((bot3_corners[0])[2])[1]



            if ref1_corners is not None:
                
                xr11 = ((ref1_corners[0])[0])[0]
                yr11 = ((ref1_corners[0])[0])[1]
            
                
            else:
                for element in ref1_prev:
                    if element is not None:
                        ref1_corners = element
                xr11 = ((ref1_corners[0])[0])[0]
                yr11 = ((ref1_corners[0])[0])[1]




            if ref2_corners is not None:
                
                xr22 = ((ref2_corners[0])[1])[0]
                yr22 = ((ref2_corners[0])[1])[1]
            
                
            else:
                for element in ref2_prev:
                    if element is not None:
                        ref2_corners = element
                xr22 = ((ref2_corners[0])[1])[0]
                yr22 = ((ref2_corners[0])[1])[1]

            
            if ref3_corners is not None:
                
                xr33 = ((ref3_corners[0])[2])[0]
                yr33 = ((ref3_corners[0])[2])[1]
            
                
            else:
                for element in ref3_prev:
                    if element is not None:
                        ref3_corners = element
                xr33 = ((ref3_corners[0])[2])[0]
                yr33 = ((ref3_corners[0])[2])[1]

            if ref4_corners is not None:
                
                xr44 = ((ref4_corners[0])[3])[0]
                yr44 = ((ref4_corners[0])[3])[1]
            
                
            else:
                for element in ref4_prev:
                    if element is not None:
                        ref4_corners = element
                xr44 = ((ref4_corners[0])[3])[0]
                yr44 = ((ref4_corners[0])[3])[1]
            
            # cornerr1 = np.float32([[xr11, yr11], [xr22, yr22],
            #                     [xr44, yr44], [xr33, yr33]])
                
            cornerr1 = np.float32([[99, 23], [480, 40],
                                [95, 412], [479, 402]])
            transformed_pers_img = self.transform_img(cv_image, cornerr1)
            print(xr11, yr11)

            self.corners1, self.ids1, rejected1 = self.detector1.detectMarkers(transformed_pers_img)
            cv.aruco.drawDetectedMarkers(transformed_pers_img, self.corners1, self.ids1)
            self.display_image(transformed_pers_img, "transformed_image")
            # print(self.ids1)

            nbot1_corners = [[[0.1,0.2],[0.3,0.4],[0.5,0.6],[0.7,0.8]]]
            nbot2_corners = [[[0.2,0.3],[0.4,0.5],[0.21,0.6],[0.7,0.8]]]
            nbot3_corners = [[[0.3,0.4],[0.5,0.6],[0.7,0.8],[0.9,0.123]]]
            nx11 = 1
            ny11 = 2
            nx12 = 2
            ny12 = 3
            nx13 = 0.3
            ny13 = 0.4
            nx21 = 0.5
            ny21 = 0.11
            nx22 = 0.51
            ny22 = 0.111
            nx23 = 0.22
            ny23 = 0.13
            nx31 = 0.5
            ny31 = 0.11
            nx32 = 0.51
            ny32 = 0.111
            nx33 = 0.22
            ny33 = 0.13



            if self.ids1 is not None:
                for i in range(len(self.ids1)):
                    if self.ids1[i] == bot1_marker_id:
                        nbot1_corners = self.corners1[i]
                        nbot1_prev.append(nbot1_corners)

                for i in range(len(self.ids1)):
                    if self.ids1[i] == bot2_marker_id:
                        nbot2_corners = self.corners1[i]
                        nbot2_prev.append(nbot2_corners)

                for i in range(len(self.ids1)):
                    if self.ids1[i] == bot3_marker_id:
                        nbot3_corners = self.corners1[i]
                        nbot3_prev.append(nbot3_corners)


                if nbot1_corners is not None:
                
                    nx11 = ((nbot1_corners[0])[0])[0]
                    ny11 = ((nbot1_corners[0])[0])[1]
                    nx12 = ((nbot1_corners[0])[1])[0]
                    ny12 = ((nbot1_corners[0])[1])[1]
                    nx13 = ((nbot1_corners[0])[2])[0]
                    ny13 = ((nbot1_corners[0])[2])[1]                    
                else:
                    for element in nbot1_prev:
                        if element is not None:
                            nbot1_corners = element
                    nx11 = ((nbot1_corners[0])[0])[0]
                    ny11 = ((nbot1_corners[0])[0])[1]
                    nx12 = ((nbot1_corners[0])[1])[0]
                    ny12 = ((nbot1_corners[0])[1])[1]
                    nx13 = ((nbot1_corners[0])[2])[0]
                    ny13 = ((nbot1_corners[0])[2])[1] 


                if nbot2_corners is not None:
                
                    nx21 = ((nbot2_corners[0])[0])[0]
                    ny21 = ((nbot2_corners[0])[0])[1]
                    nx22 = ((nbot2_corners[0])[1])[0]
                    ny22 = ((nbot2_corners[0])[1])[1]
                    nx23 = ((nbot2_corners[0])[2])[0]
                    ny23 = ((nbot2_corners[0])[2])[1] 
                    
                else:
                    for element in nbot2_prev:
                        if element is not None:
                            nbot2_corners = element
                    nx21 = ((nbot2_corners[0])[0])[0]
                    ny21 = ((nbot2_corners[0])[0])[1]
                    nx22 = ((nbot2_corners[0])[1])[0]
                    ny22 = ((nbot2_corners[0])[1])[1]
                    nx23 = ((nbot2_corners[0])[2])[0]
                    ny23 = ((nbot2_corners[0])[2])[1] 

                if nbot3_corners is not None:
                
                    nx31 = ((nbot3_corners[0])[0])[0]
                    ny31 = ((nbot3_corners[0])[0])[1]
                    nx32 = ((nbot3_corners[0])[1])[0]
                    ny32 = ((nbot3_corners[0])[1])[1]
                    nx33 = ((nbot3_corners[0])[2])[0]
                    ny33 = ((nbot3_corners[0])[2])[1] 
                    
                else:
                    for element in nbot3_prev:
                        if element is not None:
                            nbot3_corners = element
                    nx31 = ((nbot3_corners[0])[0])[0]
                    ny31 = ((nbot3_corners[0])[0])[1]
                    nx32 = ((nbot3_corners[0])[1])[0]
                    ny32 = ((nbot3_corners[0])[1])[1]
                    nx33 = ((nbot3_corners[0])[2])[0]
                    ny33 = ((nbot3_corners[0])[2])[1] 

            bot1_x = (nx11+nx13)/2
            bot1_y = (ny11+ny13)/2

            bot2_x = (nx21+nx23)/2
            bot2_y = (ny21+ny23)/2

            # print(bot2_x)

            bot3_x = (nx31+nx33)/2
            bot3_y = (ny31+ny33)/2

            m_ref_i = 500
            m_ref_j = 0
            m_bot1_i = (nx12-nx11)
            m_bot1_j = (ny12-ny11)
            m_bot2_i = (nx22-nx21)
            m_bot2_j = (ny22-ny21)
            m_bot3_i = (nx32-nx31)
            m_bot3_j = (ny32-ny31)
            # print(m_bot3_i)
            # m_bot2 = (ny23-ny21)/(nx23-nx21)
            # m_bot3 = (ny33-ny31)/(nx33-nx31)

            # bot1_theta = math.acos(((m_ref_i*m_bot1_i)+(m_ref_j*m_bot1_j))/(math.sqrt((m_ref_i**2 + m_ref_j**2)*(m_bot1_i**2 + m_bot1_j**2))))
            bot1_theta = math.degrees(math.acos(((m_ref_i*m_bot1_i)+(m_ref_j*m_bot1_j))/(math.sqrt((m_ref_i**2 + m_ref_j**2)*(m_bot1_i**2 + m_bot1_j**2)))))
            if m_bot1_j < 0:
                bot1_theta = 360-bot1_theta

            bot2_theta = math.degrees(math.acos(((m_ref_i*m_bot2_i)+(m_ref_j*m_bot2_j))/(math.sqrt((m_ref_i**2 + m_ref_j**2)*(m_bot2_i**2 + m_bot2_j**2)))))
            if m_bot2_j < 0:
                bot2_theta = 360-bot2_theta

            bot3_theta = math.degrees(math.acos(((m_ref_i*m_bot3_i)+(m_ref_j*m_bot3_j))/(math.sqrt((m_ref_i**2 + m_ref_j**2)*(m_bot3_i**2 + m_bot3_j**2)))))
            if m_bot3_j < 0:
                bot3_theta = 360-bot3_theta
            

            # bot1_theta = math.atan((m_bot1-m_ref)/(1+(m_bot1*m_ref)))
            # bot2_theta = math.atan((m_bot2-m_ref)/(1+(m_bot2*m_ref)))
            # bot3_theta = math.atan((m_bot3-m_ref)/(1+(m_bot3*m_ref)))

            

            dx = 2.5
            dy = 25.45

            # pen1_pose_x = (self.pen_pose_point(bot1_x, bot1_y ,dx,dy,bot1_theta))[0]
            # pen1_pose_y = (self.pen_pose_point(bot1_x, bot1_y ,dx,dy,bot1_theta))[1]
            pen1_pose_x = bot1_x
            pen1_pose_y = bot1_y

            # pen2_pose_x = (self.pen_pose_point(bot2_x, bot2_y ,dx,dy,bot2_theta))[0]
            # pen2_pose_y = (self.pen_pose_point(bot2_x, bot2_y ,dx,dy,bot2_theta))[1]
            pen2_pose_x = bot2_x
            pen2_pose_y = bot2_y

            # pen3_pose_x = (self.pen_pose_point(bot3_x, bot3_y ,dx,dy,bot3_theta))[0]
            # pen3_pose_y = (self.pen_pose_point(bot3_x, bot3_y ,dx,dy,bot3_theta))[1]
            pen3_pose_x = bot3_x
            pen3_pose_y = bot3_y
    

            pen1_pos = Pose2D()
            pen1_pos.x = pen1_pose_x
            pen1_pos.y = pen1_pose_y
            pen1_pos.theta = bot1_theta
            self.aruco_pose_pub_1.publish(pen1_pos)
            pen11_pos = Pose2D()
            pen11_pos.x = pen1_pose_x
            pen11_pos.y = pen1_pose_y
            pen11_pos.theta = (bot1_theta*math.pi)/180
            self.aruco_pose_pub_1_eval.publish(pen11_pos)


            pen2_pos = Pose2D()
            pen2_pos.x = pen2_pose_x
            pen2_pos.y = pen2_pose_y
            pen2_pos.theta = bot2_theta
            self.aruco_pose_pub_2.publish(pen2_pos)
            pen22_pos = Pose2D()
            pen22_pos.x = pen2_pose_x
            pen22_pos.y = pen2_pose_y
            pen22_pos.theta = (bot2_theta*math.pi)/180
            self.aruco_pose_pub_2_eval.publish(pen22_pos)

            pen3_pos = Pose2D()
            pen3_pos.x = pen3_pose_x
            pen3_pos.y = pen3_pose_y
            pen3_pos.theta = bot3_theta
            self.aruco_pose_pub_3.publish(pen3_pos)
            pen33_pos = Pose2D()
            pen33_pos.x = pen3_pose_x
            pen33_pos.y = pen3_pose_y
            pen33_pos.theta = (bot3_theta*math.pi)/180
            self.aruco_pose_pub_3_eval.publish(pen33_pos)

            if len(bot1_prev)>=max_length:
                bot1_prev.pop(0)
            if len(bot2_prev)>=max_length:
                bot2_prev.pop(0)
            if len(bot3_prev)>=max_length:
                bot3_prev.pop(0)
            if len(nbot1_prev)>=max_length:
                nbot1_prev.pop(0)
            if len(nbot2_prev)>=max_length:
                nbot2_prev.pop(0)
            if len(nbot3_prev)>=max_length:
                nbot3_prev.pop(0)
            if len(ref1_prev)>=max_length:
                ref1_prev.pop(0)
            if len(ref2_prev)>=max_length:
                ref2_prev.pop(0)
            if len(ref3_prev)>=max_length:
                ref3_prev.pop(0)
            if len(ref4_prev)>=max_length:
                ref4_prev.pop(0)
            
            
            
                

    # def get_corners(self, ids, corners, bot_marker_id):
    #     for i in range(len(ids)):
    #         if ids[i] == bot_marker_id:
    #             bot_corners = corners[i]
    #             # bot1_prev.append(bot_corners)
    #     return bot_corners
    
    # def get_x1_y1_x3_y3(self, bot_corners):
    #     x1 = ((bot_corners[0])[0])[0]
    #     y1 = ((bot_corners[0])[0])[1]
    #     x3 = ((bot_corners[0])[2])[0]
    #     y3 = ((bot_corners[0])[2])[1]
    #     return x1,y1,x3,y3




    def pen_pose_point(self,xc,yc,x,y,angle_d):
        angle = (angle_d*math.pi)/180
        c = np.cos(angle)
        s = np.sin(angle)

        rot_mat = np.array([[c, -s], 
                            [s, c]])
        pts_to_be_rot = np.array([[x],
                                  [y]])
        pts_rotated = np.dot(rot_mat, pts_to_be_rot)
        rotated_x = (pts_rotated[0])[0]
        rotated_y = (pts_rotated[1])[0]
        
        penpose_x = xc + rotated_x
        penpose_y = yc + rotated_y

        return penpose_x, penpose_y


    def display_image(self, cv_image, name):
        cv.imshow(name, cv_image)
        cv.waitKey(1)
        
    def transform_img(self, input_frame, corner1):
        
        pts2 = np.float32([[0, 0], [500, 0],
                        [0, 500], [500, 500]])
    
        matrix = cv.getPerspectiveTransform(corner1, pts2)
        result = cv.warpPerspective(input_frame, matrix, (500, 500))
        return result
 
def main(args = None):
    rclpy.init(args=args)
    Publisher_node = Publisher()
    rclpy.spin(Publisher_node)
    Publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
