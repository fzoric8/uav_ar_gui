#!/usr/bin/env python2.7
import rospy
import sys
import numpy as np
import math
from nav_msgs.msg import Odometry
from PIL import Image as PILImage, ImageFont,ImageDraw
from sensor_msgs.msg import Image as ROSImage
from tf.transformations import euler_from_quaternion, quaternion_from_euler
#http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html
class PrintPosition():
    def __init__ (self, frequency): 
        self.frequency = int(frequency) 
        self.image_pub = rospy.Publisher('/uav/gui', ROSImage, queue_size=1) 
        #https://answers.ros.org/question/243855/how-do-publishersubscriber-message-queues-work/
        
        rospy.init_node('drone_odom', anonymous=True)
        self.odom_sub = rospy.Subscriber('/firefly/ground_truth/odometry', Odometry, self.odom_callback, queue_size=1)
        
        self.odom_msg_recv = False
        
        # ROS Image
        self.ros_img = ROSImage()
        
    def odom_callback(self, data):
        self.odom_msg_recv = True
        self.odom=Odometry()
        height=round(data.pose.pose.position.z, 3)
        height=str(height)
        linvel_x=data.twist.twist.linear.x
        linvel_y=data.twist.twist.linear.y
        linvel_z=data.twist.twist.linear.z
        linear_velocity=math.sqrt(linvel_x*linvel_x + linvel_y*linvel_y + linvel_z*linvel_z)
        linear_velocity=round(linear_velocity,3)
        linear_velocity_str=str(linear_velocity)
        yaw=PrintPosition.get_rotation(self.odom)
        print('JAW' + str(yaw))
        pil_img = PrintPosition.draw_gui(height, linear_velocity_str, 40)
        self.ros_img = PrintPosition.convert_pil_to_ros_img(self, pil_img)

    @staticmethod
    def get_rotation(msg):
        orientation_q = msg.pose.pose.orientation
        roll = pitch = yaw = 0.0
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        return yaw

    @staticmethod
    def convert_pil_to_ros_img(self, img):
        img=img.convert('RGB')
        msg = ROSImage()
        stamp = rospy.Time.now()
        msg.height = img.height
        msg.width = img.width
        msg.encoding = "rgb8"
        msg.is_bigendian = False
        msg.step = 3 * img.width
        msg.data = np.array(img).tobytes()
        return msg 

    @staticmethod
    def draw_compass_on_image(pil_img, ellipse_center_x, ellipse_center_y, ellipse_radius, angle_deg, font):
        draw=ImageDraw.Draw(pil_img)
        draw.ellipse((ellipse_center_x-50, 20, 620, 120), fill = (211, 211, 211), outline ='black',)
        draw.text((ellipse_center_x-3, ellipse_center_y-ellipse_radius-20), "N", (0,0,0), font=font)
        draw.text((ellipse_center_x + ellipse_radius + 6, ellipse_center_y-3), "E", (0,0,0), font=font)
        draw.text((ellipse_center_x-3, ellipse_center_y+ellipse_radius+5), "S", (0,0,0), font=font)
        draw.text((ellipse_center_x - ellipse_radius - 20, ellipse_center_y-3), "W", (0,0,0), font=font)
        draw.ellipse((ellipse_center_x-4, ellipse_center_y-4, ellipse_center_x+4, ellipse_center_y+4), fill= 'red', outline='red')
        if angle_deg<0:
            angle_deg=360+angle_deg
        angle_rad = math.radians(angle_deg)
        if angle_deg>=0 and angle_deg<90:
            angle_deg=90-angle_deg
            angle_rad=math.radians(angle_deg)
            second_point_x = (ellipse_radius * math.cos(angle_rad)) + ellipse_center_x
            second_point_y = ellipse_center_y - ellipse_radius * math.sin(angle_rad)
        elif angle_deg>=90 and angle_deg<=180:
            angle_deg=angle_deg-90
            angle_rad=math.radians(angle_deg)
            if angle_deg==0:
                second_point_x = ellipse_center_x + ellipse_radius
            else:
                second_point_x = ellipse_radius * math.cos(angle_rad) + ellipse_center_x
            second_point_y = ellipse_radius * math.sin(angle_rad) + ellipse_center_y
        elif angle_deg>180 and angle_deg<=270:
            angle_deg=90 - (angle_deg-180) 
            angle_rad=math.radians(angle_deg)
            second_point_x = ellipse_center_x - ellipse_radius * math.cos(angle_rad)
            second_point_y = ellipse_radius * math.sin(angle_rad) + ellipse_center_y
        else:
            angle_deg=360-angle_deg+90
            angle_rad=math.radians(angle_deg)
            second_point_x = ellipse_center_x + ellipse_radius * math.cos(angle_rad)
            second_point_y = ellipse_center_y - ellipse_radius * math.sin(angle_rad)
        draw.line((ellipse_center_x, ellipse_center_y, second_point_x, second_point_y), fill=(255,0,0), width=3)
        

        return pil_img

    def run(self):
        rate = rospy.Rate(self.frequency)
        print("Entered run")
        while not rospy.is_shutdown():
            rate.sleep()
            print("running")
            
            # Publish saved msg 
            if self.odom_msg_recv:
                self.image_pub.publish(self.ros_img)

    @staticmethod
    def draw_gui(drone_height, linear_velocity, yaw):
        ellipse_center_x = 570
        ellipse_center_y = 70
        ellipse_radius = 50
        angle_deg=yaw
        pil_img=PILImage.new("RGBA", (640,480), 'white')
        font=ImageFont.truetype("/home/developer/arial.ttf", 15, encoding="unic")
        draw=ImageDraw.Draw(pil_img)
        draw.text((10,0), "Height:", (0,0,0), font=font)
        draw.text((20,25), drone_height + " m", (0,0,0), font=font)
        draw.text((10,200), "Linear velocity:", (0,0,0), font=font)
        draw.text((20,225), linear_velocity + " m/s", (0,0,0), font=font)
        pil_img = PrintPosition.draw_compass_on_image(pil_img, ellipse_center_x, ellipse_center_y, ellipse_radius, angle_deg, font)
        return pil_img

if __name__ == '__main__':
    try:
        p = PrintPosition(sys.argv[1])
        print("Here")
        p.run()
    except rospy.ROSInterruptException: pass