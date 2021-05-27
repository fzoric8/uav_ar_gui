#!/usr/bin/env python2.7
import rospy
import sys
import yaml
from rospy.numpy_msg import numpy_msg
import numpy as np
import math
from nav_msgs.msg import Odometry
from PIL import Image as PILImage, ImageFont, ImageDraw
from sensor_msgs.msg import Image as ROSImage
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class PrintPosition():
    """GUI for drone simulation
    """
    def __init__(self, frequency):
        """Constructor of PrintPosition class

        Args:
            frequency (integer): A frequency used for sleep method to wait until next iteration
        """
        self.frequency = int(frequency)
        rospy.init_node('drone_odom', anonymous=True)

        with open("/home/developer/catkin_ws/src/probniPack/config/config.yml", "r") as ymlfile:
            cfg = yaml.load(ymlfile)

        self.image_pub = rospy.Publisher(cfg["topics"]["ui_pub"], ROSImage, queue_size=1)

        self.odom_sub = rospy.Subscriber(cfg["topics"]["odm_sub"], Odometry, self.odom_callback, queue_size=1)

        self.cam_sub = rospy.Subscriber(cfg["topics"]["cam_sub"], numpy_msg(ROSImage), self.cam_callback, queue_size=1)

        self.odom_msg_recv = False
        self.img_recv = False
        # ROS Image
        self.ros_img = ROSImage()
        self.cam_img = None
        self.pimg=None


    def cam_callback(self, image):
        """Callback function for getting image from drone camera

        Args:
            image (rospy.numpy_msg.Numpy_sensor_msgs__Image): Image from a drone sensor
        """
        self.img_recv = True
        self.cam_img = np.frombuffer(image.data, dtype=np.uint8).reshape(
            image.height, image.width)
        self.cam_img = np.stack((self.cam_img,)*3, axis=-1)


    def odom_callback(self, data):
        """Callback function for Odometry (Calculates the height, linear velocity and yaw rotation of the drone; Processes the image for GUI)

        Args:
            data (nav_msgs.msg._Odometry.Odometry): Odometry represents an estimate of a position and velocity in free space
        """
        self.odom_msg_recv = True
        self.odom = Odometry()
        height = round(data.pose.pose.position.z, 3)
        height = str(height)
        linvel_x = data.twist.twist.linear.x
        linvel_y = data.twist.twist.linear.y
        linvel_z = data.twist.twist.linear.z
        linear_velocity = math.sqrt(linvel_x*linvel_x + linvel_y*linvel_y + linvel_z*linvel_z)
        linear_velocity = round(linear_velocity, 3)
        linear_velocity_str = str(linear_velocity)
        roll,pitch,yaw = PrintPosition.get_rotation(data)
        roll = math.degrees(roll)
        yaw = math.degrees(yaw)
        pitch = math.degrees(pitch)
        try:
            if type(self.cam_img) is np.ndarray:
                if self.img_recv:
                    pil_img = PILImage.fromarray(self.cam_img.astype('uint8'), 'RGB')
                    self.pimg = pil_img
            else:
                pil_img = PILImage.new("RGBA", (640, 480), 'white')
        except Exception as e:
            if self.img_recv and self.pimg: 
                pil_img = self.pimg
        start_t = rospy.Time.now().to_sec()
        pil_img = PrintPosition.draw_gui(height, linear_velocity_str, roll, pitch, yaw, pil_img)
        duration = rospy.Time.now().to_sec() - start_t
        debug_duration = True
        if debug_duration:
            print("Draw GUI on image lasts: {}".format(duration))
        self.ros_img = PrintPosition.convert_pil_to_ros_img(self, pil_img)


    @staticmethod
    def get_rotation(msg):
        """Function that returns the rotation of the drone in the yaw direction

        Args:
            msg (nav_msgs.msg._Odometry.Odometry): Odometry represents an estimate of a position and velocity in free space

        Returns:
            float: Rotation of a drone in the yaw direction in radians
        """
        roll = pitch = yaw = 0.0
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x,orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        return (roll, pitch, yaw)


    @staticmethod
    def convert_pil_to_ros_img(self, img):
        """Function for converting pillow to ros image

        Args:
            img (PIL.Image.Image): Pillow image that represents GUI

        Returns:
            sensor_msgs.msg._Image.Image: ROS image for image publisher
        """
        img = img.convert('RGB')
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
        """Function that draws compass on pillow image

        Args:
            pil_img (PIL.Image.Image): Pillow image used for drawing GUI
            ellipse_center_x (int): X coordinate for positioning of the compass (top left corner of an imaginary square)
            ellipse_center_y (int): Y coordiante for positioning of the compass (top left corner of an imaginary square)
            ellipse_radius (int): Radius of the compass
            angle_deg (float): Yaw rotation of a drone in degrees
            font (PIL.ImageFont.FreeTypeFont): Font style used to format text on pillow image 

        Returns:
            PIL.Image.Image: Pillow image with a compass on it
        """
        draw = ImageDraw.Draw(pil_img)
        draw.ellipse((ellipse_center_x, ellipse_center_y, ellipse_center_x + 100, ellipse_center_y + 100), fill=(211, 211, 211), outline='black',)
        draw.text((ellipse_center_x + 47, ellipse_center_y - 20), "N", (0, 0, 0), font=font)
        draw.text((ellipse_center_x + ellipse_radius + 56, ellipse_center_y + ellipse_radius), "E", (0, 0, 0), font=font)
        draw.text((ellipse_center_x + 47, ellipse_center_y + ellipse_radius + ellipse_radius + 5), "S", (0, 0, 0), font=font)
        draw.text((ellipse_center_x - 20, ellipse_center_y + ellipse_radius), "W", (0, 0, 0), font=font)
        draw.ellipse((ellipse_center_x + 46, ellipse_center_y + 46, ellipse_center_x+54, ellipse_center_y + 54), fill='red', outline='red')

        if angle_deg < 0:
            angle_deg = 360+angle_deg
        angle_rad = math.radians(angle_deg)

        if angle_deg >= 0 and angle_deg < 90:
            angle_deg = 90-angle_deg
            angle_rad = math.radians(angle_deg)
            second_point_x = ellipse_center_x - (ellipse_radius * math.cos(angle_rad)) 
            second_point_y = ellipse_center_y - ellipse_radius * math.sin(angle_rad)
        elif angle_deg >= 90 and angle_deg <= 180:
            angle_deg = angle_deg-90
            angle_rad = math.radians(angle_deg)
            if angle_deg == 0:
                second_point_x = ellipse_center_x + ellipse_radius
            else:
                second_point_x = ellipse_center_x - ellipse_radius * math.cos(angle_rad)
            second_point_y = ellipse_radius * math.sin(angle_rad) + ellipse_center_y
        elif angle_deg > 180 and angle_deg <= 270:
            angle_deg = (angle_deg-180)-90
            angle_rad = math.radians(angle_deg)
            second_point_x = ellipse_center_x + ellipse_radius * math.cos(angle_rad)
            second_point_y = ellipse_center_y - ellipse_radius *  math.sin(angle_rad)
        else:
            angle_deg = 360-angle_deg-90
            angle_rad = math.radians(angle_deg)
            second_point_x = ellipse_center_x + ellipse_radius * math.cos(angle_rad)
            second_point_y = ellipse_center_y + ellipse_radius * math.sin(angle_rad)

        draw.line((ellipse_center_x+50, ellipse_center_y + 50, second_point_x, second_point_y), fill=(255, 0, 0), width=3)

        return pil_img
    

    @staticmethod
    def draw_roll_attitude_indicator(pil_img, ellipse_center_x, ellipse_center_y, ellipse_radius, angle_deg_roll, font):
        """Function for drawing roll attitude indicator on a pillow image

        Args:
            pil_img (PIL.Image.Image): Pillow image used for drawing GUI
            ellipse_center_x (int): X coordinate for positioning of the roll artificial horizon
            ellipse_center_y (int): Y coordinate for positioning of the roll artificial horizon
            ellipse_radius (int): Radius of the attitude indicator
            angle_deg_roll (float): Roll rotation of a drone in degrees
            font (PIL.ImageFont.FreeTypeFont): Font style used to format text on pillow image 

        Returns:
            PIL.Image.Image: Pillow image with a roll_attitude_indicator on it
        """
        draw = ImageDraw.Draw(pil_img)
        draw.ellipse((ellipse_center_x, 200, 670, 300), fill=(134, 197, 218))
        draw.text((ellipse_center_x-12+50, ellipse_center_y + 110), "Roll", (0,0,0), font=font)
        draw.chord((ellipse_center_x, 200, 670, 300), angle_deg_roll, angle_deg_roll+180, fill=(0,190,0))
        draw.line((ellipse_center_x, 250, ellipse_center_x+100, 250), fill=(0, 0, 0), width=1)
        return pil_img


    @staticmethod
    def draw_roll_and_pitch_attitude_indicator(pil_img, ellipse_center_x, ellipse_center_y, ellipse_radius, angle_deg_roll, angle_deg_pitch, font):
        """Function for drawing roll attitude indicator on a pillow image
        Args:
            pil_img (PIL.Image.Image): Pillow image used for drawing GUI
            ellipse_center_x (int): X coordinate for positioning of the roll artificial horizon
            ellipse_center_y (int): Y coordinate for positioning of the roll artificial horizon
            ellipse_radius (int): Radius of the attitude indicator
            angle_deg_roll (float): Roll rotation of a drone in degrees
            angle_deg_pitch (float): Pitch rotation of a drone in degrees
            font (PIL.ImageFont.FreeTypeFont): Font style used to format text on pillow image 
        Returns:
            PIL.Image.Image: Pillow image with a roll_and_pitch_attitude_indicator on it
        """
        draw = ImageDraw.Draw(pil_img)
        draw.ellipse((ellipse_center_x, ellipse_center_y, ellipse_center_x + 100, ellipse_center_y + 100), fill=(134, 197, 218))
        draw.text((ellipse_center_x - 5, ellipse_center_y - 18), "Artificial horizon", (0,0,0), font=font)
        draw.chord((ellipse_center_x, ellipse_center_y, ellipse_center_x + 100, ellipse_center_y + 100), angle_deg_roll - angle_deg_pitch, angle_deg_roll + angle_deg_pitch + 180, fill=(0,190,0))
        draw.line((ellipse_center_x, ellipse_center_y + 50, ellipse_center_x + 100, ellipse_center_y + 50), fill=(0, 0, 0), width=1)
        return pil_img


    @staticmethod
    def draw_pitch_attitude_indicator(pil_img, ellipse_center_x, ellipse_center_y, ellipse_radius, angle_deg_pitch, font):
        """Function for drawing pitch attitude indicator on a pillow image

        Args:
            pil_img (PIL.Image.Image): Pillow image used for drawing GUI
            ellipse_center_x (int): X coordinate for positioning of the pitch artificial horizon
            ellipse_center_y (int): Y coordinate for positioning of the pitch artificial horizon
            ellipse_radius (int): Radius of the attitude indicator
            angle_deg_pitch (float): Pitch rotation of a drone in degrees
            font (PIL.ImageFont.FreeTypeFont): Font style used to format text on pillow image 

        Returns:
            PIL.Image.Image: Pillow image with a pitch_attitude_indicator on it
        """
        draw = ImageDraw.Draw(pil_img)
        draw.ellipse((ellipse_center_x, 350, 670, 450), fill=(134, 197, 218))
        draw.text((ellipse_center_x-16+50, ellipse_center_y + 260), "Pitch", (0,0,0), font=font)
        if angle_deg_pitch>=0:
            draw.chord((ellipse_center_x, 350, 670, 450), -angle_deg_pitch, angle_deg_pitch+180, fill=(0,190,0))
        else:
            draw.chord((ellipse_center_x, 350, 670, 450), -angle_deg_pitch, angle_deg_pitch+180, fill=(0,190,0))

        return pil_img


    def run(self):
        """Function for publishing ros image
        """
        rate = rospy.Rate(self.frequency)
        print("Entered run")
        while not rospy.is_shutdown():
            rate.sleep()
            print("running")

            # Publish saved msg
            if self.odom_msg_recv:
                self.image_pub.publish(self.ros_img)


    @staticmethod
    def draw_gui(drone_height, linear_velocity, roll, pitch, yaw, pil_img):
        """Function for drawing GUI on a pillow image

        Args:
            drone_height (str): String representation of a drone height
            linear_velocity (str): String representation of a linear_velocity of a drone
            roll (float): Rotation of a drone in roll direction
            pitch (float): Rotation of a drone in pitch direction
            yaw (float): Rotation of a drone in yaw direction 
            pil_img (PIL.Image.Image): Pillow image for gui representation

        Returns:
            PIL.Image.Image: Pillow image that represents GUI
        """
        with open("/home/developer/catkin_ws/src/probniPack/config/finalgui.yml", "r") as ymlfile:
            cfg = yaml.load(ymlfile)

        comp_ellipse_center_x = cfg["compass"]["px"]
        comp_ellipse_center_y = cfg["compass"]["py"]
        comp_ellipse_radius = cfg["compass"]["rad"]
        ah_ellipse_center_x = cfg["artificial_horizon"]["px"]
        ah_ellipse_center_y = cfg["artificial_horizon"]["py"]
        height_px = cfg["height"]["px"]
        height_py = cfg["height"]["py"]
        linear_velocity_px = cfg["linear_velocity"]["px"]
        linear_velocity_py = cfg["linear_velocity"]["py"]
        angle_deg_roll = roll
        angle_deg_yaw = yaw
        angle_deg_pitch = pitch

        font = ImageFont.truetype("/home/developer/catkin_ws/src/probniPack/include/probniPack/fonts/roboto.regular.ttf", 15, encoding="unic")
        draw = ImageDraw.Draw(pil_img)
        draw.rectangle((height_px - 3, height_py - 2, height_px + 75, height_py + 30), fill = (177, 177, 177), outline = None, width = 1)
        draw.rectangle((linear_velocity_px - 3, linear_velocity_py - 2, linear_velocity_px + 130, linear_velocity_py + 30), fill = (177, 177, 177), outline = None, width = 1)
        draw.text((height_px, height_py), "HEIGHT:", (0, 0, 0), font=font)
        draw.text((height_px + 20, height_py + 15), drone_height + " m", (0, 0, 0), font=font)
        draw.text((linear_velocity_px, linear_velocity_py), "LINEAR VELOCITY:", (0, 0, 0), font=font)
        draw.text((linear_velocity_px + 20, linear_velocity_py + 15), linear_velocity + " m/s", (0, 0, 0), font=font)
        pil_img = PrintPosition.draw_compass_on_image(pil_img, comp_ellipse_center_x, comp_ellipse_center_y, comp_ellipse_radius, angle_deg_yaw, font)
        pil_img = PrintPosition.draw_roll_and_pitch_attitude_indicator(pil_img, ah_ellipse_center_x, ah_ellipse_center_y, comp_ellipse_radius, angle_deg_roll, angle_deg_pitch, font)
        return pil_img


if __name__ == '__main__':
    try:
        p = PrintPosition(sys.argv[1])
        print("Here")
        p.run()
    except rospy.ROSInterruptException:
        pass
