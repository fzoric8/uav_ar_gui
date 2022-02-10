#!/usr/bin/env python2.7
import rospy
import sys
import yaml
import io
import math
import rospkg
import numpy as np


from PIL import Image as PILImage, ImageFont, ImageDraw

from sensor_msgs.msg import Image as ROSImage
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Odometry
from rospy.numpy_msg import numpy_msg

from tf.transformations import euler_from_quaternion, quaternion_from_euler

#TODO: 
# - Add signal health to determine latency between UAV and operator (Check timestamp of an image, and current time and determine if it's ok to operate)
# - Add stuff to determine what we control (uav/arm)
# -

class fpvGUI():
    """GUI for drone simulation
    """
    def __init__(self, frequency, config_name, gui_config):
        """Constructor of fpvGUI class

        Args:
            frequency (integer): A frequency used for sleep method to wait until next iteration
        """
        self.frequency = int(frequency)
        rospy.init_node('uav_ar_gui', anonymous=True)

        rospack = rospkg.RosPack()
        self.origin_path = rospack.get_path("uav_ar_gui")

        # Config paths
        self.font_path = "{}/include/fonts/roboto.regular.ttf".format(self.origin_path)

        # Open config path 
        with open("{}/config/{}".format(self.origin_path, config_name), "r") as ymlfile:
            self.cfg = yaml.load(ymlfile)

        # Gui config (Element positions)
        with open("{}/config/{}".format(self.origin_path, gui_config), "r") as ymlfile:
            self.gui_cfg = yaml.load(ymlfile)

        # Use zone drawing for human pose estimation to enable feedback on FPV 
        self.use_hpe = False

        # Recv flags
        self.odom_msg_recv = False
        self.img_recv = False
        self.compressed_img_recv = False
        
        # ROS Image
        self.ros_img = ROSImage()
        self.cam_img = None
        self.pil_img = None

        # Use proper image (Compressed for rPi) --> faster
        self.use_normal_image = True
        self.use_compressed_image = not self.use_normal_image

        self._init_subscribers(); self._init_publishers(); 

        rospy.loginfo("[UAVArGui] Initialized!")

    def _init_subscribers(self): 
        
        # Subs
        odom_sub_name = self.cfg["topics"]["odm_sub"]
        self.odom_sub = rospy.Subscriber(odom_sub_name, Odometry, self.odom_callback, queue_size=1)

        cam_compressed_sub_name = self.cfg["topics"]["cam_compressed_sub"]
        self.cam_compressed_sub = rospy.Subscriber(cam_compressed_sub_name, CompressedImage, self.cam_compressed_callback, queue_size=1)

        cam_sub_name = self.cfg["topics"]["cam_sub"]
        self.cam_sub = rospy.Subscriber(cam_sub_name, numpy_msg(ROSImage), self.cam_callback, queue_size=1)

        if self.use_hpe: 
            hpe_zones_name = self.cfg["topics"]["hpe_zone_sub"]
            self.hpe_sub = rospy.Subscriber(hpe_zones_name, numpy_msg(ROSImage), self.hpe_callback, queue_size=1)
            self.hpe_img_recv = False

    def _init_publishers(self): 
        
        # Pubs
        ui_pub_name = self.cfg["topics"]["ui_pub"]
        self.image_pub = rospy.Publisher(ui_pub_name, ROSImage, queue_size=1)

    def cam_callback(self, image):
        """Callback function for getting image from drone camera

        Args:
            image (rospy.numpy_msg.Numpy_sensor_msgs__Image): Image from a drone sensor
        """
        self.img_recv = True
        
        self.img_width = image.width
        self.img_height = image.height

        if (image.encoding == "rgb8"):
            self.cam_img = np.frombuffer(image.data, dtype=np.uint8).reshape(
                image.height, image.width, 3)
          
        else:
            
            self.cam_img = np.frombuffer(image.data, dtype=np.uint8).reshape(
                image.height, image.width)
        
        
            self.cam_img = np.stack((self.cam_img,)*3, axis=-1)

        if self.use_normal_image:
            self.pil_img = PILImage.fromarray(self.cam_img.astype('uint8'), 'RGB')

    def cam_compressed_callback(self, compr_image):
        
        self.compressed_img_recv = True

        self.comp_img_reciv_t = compr_image.header.stamp.to_sec()
        self.compressed_img = compr_image.data

        if self.use_compressed_image:
            self.pil_img = PILImage.open(io.BytesIO(bytearray(self.compressed_img)))

        # Transform directly to PILLOW image

    def odom_callback(self, data):
        """Callback function for Odometry (Calculates the height, linear velocity and yaw rotation of the drone; Processes the image for GUI)

        Args:
            data (nav_msgs.msg._Odometry.Odometry): Odometry represents an estimate of a position and velocity in free space
        """

        self.odom_msg_recv = True
        self.odom = Odometry()

        self.height = round(data.pose.pose.position.z, 3)

        linvel_x = data.twist.twist.linear.x
        linvel_y = data.twist.twist.linear.y
        linvel_z = data.twist.twist.linear.z
        linear_velocity = math.sqrt(linvel_x*linvel_x + linvel_y*linvel_y + linvel_z*linvel_z)
        self.linear_velocity = round(linear_velocity, 3)

        roll,pitch,yaw = self.get_rotation(data)
        self.roll = math.degrees(roll)
        self.yaw = math.degrees(yaw)
        self.pitch = math.degrees(pitch)
  
    def hpe_callback(self, image): 
        #TODO: 
        # Save it as PIL image 

        self.hpe_img_recv = True

        if (image.encoding == "rgb8"):
            self.hpe_img = np.frombuffer(image.data, dtype=np.uint8).reshape(
                image.height, image.width, 3)

    def get_rotation(self, msg):
        """Function that returns the rotation of the drone in the yaw direction

        Args:
            msg (nav_msgs.msg._Odometry.Odometry): Odometry represents an estimate of a position and velocity in free space

        Returns:
            float: Rotation of a drone in the yaw direction in radians
        """
        roll = pitch = yaw = 0.0
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        return (roll, pitch, yaw)

    def create_fpv(self, pil_img, hpe_img=None):
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

        comp_cx = self.gui_cfg["compass"]["cx"]
        comp_cy = self.gui_cfg["compass"]["cy"]
        comp_d   = self.gui_cfg["compass"]["d"]

        ah_cx = self.gui_cfg["artificial_horizon"]["cx"]
        ah_cy = self.gui_cfg["artificial_horizon"]["cy"]
        ah_d   = self.gui_cfg["artificial_horizon"]["d"]

        height_px = self.gui_cfg["height"]["cx"]
        height_py = self.gui_cfg["height"]["cy"]

        linear_velocity_px = self.gui_cfg["linear_velocity"]["cx"]
        linear_velocity_py = self.gui_cfg["linear_velocity"]["cy"]

        roll_deg = self.roll
        yaw_deg = self.yaw
        pitch_deg = self.pitch

        # Draw linear velocity and height
        font = ImageFont.truetype(self.font_path, 15, encoding="unic")
        draw = ImageDraw.Draw(pil_img)
        draw.rectangle((height_px - 3, height_py - 2, height_px + 75, height_py + 30), fill = (177, 177, 177), outline = None, width = 1)
        draw.rectangle((linear_velocity_px - 3, linear_velocity_py - 2, linear_velocity_px + 130, linear_velocity_py + 30), fill = (177, 177, 177), outline = None, width = 1)
        draw.text((height_px, height_py), "HEIGHT:", (0, 0, 0), font=font)
        draw.text((height_px + 20, height_py + 15), str(self.height) + " m", (0, 0, 0), font=font)
        draw.text((linear_velocity_px, linear_velocity_py), "LINEAR VELOCITY:", (0, 0, 0), font=font)
        draw.text((linear_velocity_px + 20, linear_velocity_py + 15), str(self.linear_velocity) + " m/s", (0, 0, 0), font=font)
        
        # Draw compass
        pil_img = fpvGUI.draw_compass_on_image(pil_img, comp_cx, comp_cy, comp_d, yaw_deg, font)
        # Draw roll and pitch attitude indicator
        pil_img = fpvGUI.draw_roll_and_pitch_attitude_indicator(pil_img, ah_cx, ah_cy, ah_d, roll_deg, pitch_deg, font)
        # Add zones for feedback on UI 
        if hpe_img:
            img_width, img_height = self.img_width, self.img_height
            pil_img = fpvGUI.add_hpe_zones_on_image(pil_img, hpe_img, img_width, img_height)

        return pil_img

    def run(self):

        rate = rospy.Rate(self.frequency)

        while not rospy.is_shutdown():
            rate.sleep()

            if self.use_hpe: 
                self.recv_condition = self.img_recv and self.hpe_img_recv
            else: 
                self.recv_condition = self.img_recv

            rospy.logdebug("self.recv_condition: {}".format(self.recv_condition))
            rospy.logdebug("self.img_recv: {}".format(self.img_recv))

            if self.odom_msg_recv and self.recv_condition:
                
                # Create hpe_img for UAV gui draw!
                if self.use_hpe:
                    hpe_img = self.hpe_img; 
                else:
                    hpe_img = None

                rospy.loginfo("Started!")
                start_t = rospy.Time.now().to_sec()
                
                pil_img = self.pil_img
                pil_img = self.create_fpv(pil_img, hpe_img)

                duration = rospy.Time.now().to_sec() - start_t
                
                debug_duration = True
                if debug_duration:
                    rospy.logdebug("Draw GUI on image lasts: {}".format(duration))

                self.ros_img = fpvGUI.convert_pil_to_ros_img(self, pil_img)

                self.image_pub.publish(self.ros_img)

            rospy.sleep(0.025)

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
    def draw_compass_on_image(pil_img, cx, cy, d, angle_deg, font):
        """Function that draws compass on pillow image

        Args:
            pil_img (PIL.Image.Image): Pillow image used for drawing GUI
            cx (int): X coordinate for positioning of the compass (center_x of an imaginary square)
            cy (int): Y coordiante for positioning of the compass (center_y of an imaginary square)
            diameter (int): Diameter of the compass
            angle_deg (float): Yaw rotation of a drone in degrees
            font (PIL.ImageFont.FreeTypeFont): Font style used to format text on pillow image 

        Returns:
            PIL.Image.Image: Pillow image with a compass on it
        """

        # Compass diameter
        r = d/2
        # Upper left corner of a compass
        px1, py1 = cx - r, cy - r 

        # TODO: Fix text to fit new compass
        draw = ImageDraw.Draw(pil_img)
        draw.ellipse((px1, py1, cx + r, cy + r), fill=(211, 211, 211), outline='black', width=3)
        #draw.text((cx + 47, cy - 20), "N", (0, 0, 0), font=font)
        #draw.text((cx + r + 56, cy + r), "E", (0, 0, 0), font=font)
        #draw.text((cx + 47, cy + r + r + 5), "S", (0, 0, 0), font=font)
        #draw.text((cx - 20, cy + r), "W", (0, 0, 0), font=font)

        if angle_deg < 0:
            angle_deg = 360 + angle_deg
        angle_rad = math.radians(angle_deg)

        # Could be possible to use atan2 instead of all this sine/cosine operations
        if angle_deg >= 0 and angle_deg < 90:
            angle_deg = 90 - angle_deg
            angle_rad = math.radians(angle_deg)
            px2 = cx - r * math.cos(angle_rad)
            py2 = cy - r * math.sin(angle_rad)

        elif angle_deg >= 90 and angle_deg <= 180:
            angle_deg = angle_deg - 90
            angle_rad = math.radians(angle_deg)
            if angle_deg == 0:
                px2 = cx
            else:
                px2 = cx - r * math.cos(angle_rad)
            py2 = (r * math.sin(angle_rad)) + cy + r

        elif angle_deg > 180 and angle_deg <= 270:
            angle_deg = 360 - angle_deg - 90
            angle_rad = math.radians(angle_deg)
            px2 = cx + r * math.cos(angle_rad)
            py2 = cy + r * math.sin(angle_rad)

        else:
            angle_deg = 90 - (360-angle_deg)
            angle_rad = math.radians(angle_deg)
            px2 = cx + r * math.cos(angle_rad)
            py2 = cy - r * math.sin(angle_rad)

        draw.line((cx, cy, px2, py2), fill=(255, 0, 0), width=5)
        draw.ellipse((cx - r/20, cy - r/20, cx + r/20, cy + r/20), fill=(169, 169, 169), outline = None)
        
        return pil_img

    @staticmethod
    def draw_roll_and_pitch_attitude_indicator(pil_img, cx, cy, d, roll_deg, pitch_deg, font):
        """Function for drawing roll attitude indicator on a pillow image
        Args:
            pil_img (PIL.Image.Image): Pillow image used for drawing GUI
            cx (int): X coordinate for positioning of the roll artificial horizon
            cy (int): Y coordinate for positioning of the roll artificial horizon
            ellipse_radius (int): Radius of the attitude indicator
            angle_deg_roll (float): Roll rotation of a drone in degrees
            angle_deg_pitch (float): Pitch rotation of a drone in degrees
            font (PIL.ImageFont.FreeTypeFont): Font style used to format text on pillow image 
        Returns:
            PIL.Image.Image: Pillow image with a roll_and_pitch_attitude_indicator on it
        """
        draw = ImageDraw.Draw(pil_img)

        # radius = diameter/2
        r = d/2

        px1 = cx - r; py1 = cy - r 
        px2 = cx + r; py2 = cy + r; 

        draw.ellipse((px1, py1, px2, py2), fill=(134, 197, 218))
        draw.text((cx - 5, cy - 18), "Artificial horizon", (0,0,0), font=font)
        draw.chord((px1, py1, px2, py2), roll_deg - pitch_deg, roll_deg + pitch_deg + 180, fill=(0,190,0))
        draw.line((px1, cy, cx + r, cy), fill=(0, 0, 0), width=1)
        
        return pil_img

    @staticmethod
    def add_hpe_zones_on_image(src_pil_img, target_pil_img, img_width, img_height): 

        # Conditions 
        # Target Image has to have smaller resolution than src PIL image 
        target_pil_img.resize((img_width/4, img_height/4)) 
        src_pil_img.paste(target_pil_img, box=(0, (img_height * 3)/4))

        return src_pil_img

  
if __name__ == '__main__':
    try:
        p = fpvGUI(sys.argv[1], sys.argv[2], sys.argv[3])
        p.run()

    except rospy.ROSInterruptException:
        pass

      
      