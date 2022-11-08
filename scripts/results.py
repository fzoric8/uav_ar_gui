#!/usr/bin/env python2.7
import rospy
import sys
import yaml
import io
import math
import rospkg
import numpy as np
import sqlite3


from PIL import Image as PILImage, ImageFont, ImageDraw

from std_msgs.msg import String
from sensor_msgs.msg import Image as ROSImage
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Odometry
from rospy.numpy_msg import numpy_msg

from tf.transformations import euler_from_quaternion, quaternion_from_euler

class monitorResults():
    """GUI for drone simulation
    """
    def __init__(self, frequency):
        """Constructor of fpvGUI class

        Args:
            frequency (integer): A frequency used for sleep method to wait until next iteration
        """
        
        self.frequency = int(frequency)
        self.db_conn = None
        rospy.init_node('monitor_results_node', anonymous=True, log_level=rospy.INFO)

        # Config paths
        rospack = rospkg.RosPack()
        self.origin_path = rospack.get_path("uav_ar_gui")
        self.font_path = "{}/include/fonts/zagreb_underground.ttf".format(self.origin_path)

        try:
            path_to_db = "/home/developer/catkin_ws/src/med_uav_control/db"
            self.db_conn = sqlite3.connect('{}/race.db'.format(path_to_db)) 

        except Exception as e:
            rospy.logwarn("{}".format(str(e)))

        # Img plot dim
        self.img_w = 800
        self.img_h = 600

        self.leaderboard_pub = rospy.Publisher("/leaderboard", ROSImage, queue_size=1)

    def fetch_data(self, connection): 
        cur = connection.cursor()
        cur.execute("SELECT * FROM RaceResults")

        rows = cur.fetchall()
        results = []
        data = []
        for row in rows:
            results.append(float(row[2]))
            data.append(row)

        return results, data

    
    def sort_data(self, results, data): 

        indices = sorted(range(len(results)), key=lambda k: results[k])
        data_ = [self.deconstruct_row(data[i]) for i in indices if i < 10]
        
        return data_

    def deconstruct_row(self, row): 

        mail = row[0]
        name = row[1]
        result = row[2]

        return {"mail": mail, "name": name, "result": result}

    def create_pil_img(self): 
        
        array = np.zeros((self.img_w, self.img_h, 3))*255
        pil_img = PILImage.fromarray(array, 'RGB')
        return pil_img

    def plot_txt_on_pil_img(self, pil_img, data): 
        
        # Draw linear velocity and height
        small_font = ImageFont.truetype(self.font_path, 24, encoding="unic")
        large_font = ImageFont.truetype(self.font_path, 32, encoding="unic")
        draw = ImageDraw.Draw(pil_img)
        c_off_x = 40; c_off_y = 50
        draw.text((c_off_x, c_off_y), "PLACE \t\t NAME \t\t TIME\n", (0, 255, 0), font=small_font)
        for i, d in enumerate(data): 
            draw.text((c_off_x, 2*c_off_y + i*c_off_y), "{}. \t {} \t {}\n".format(i+1, d["name"], d["result"]), (0, 255, 0), font=large_font)
       

    def run(self):

        rate = rospy.Rate(self.frequency)

        while not rospy.is_shutdown():
            if self.db_conn:
                # Get results from DB
                results, data = self.fetch_data(self.db_conn)
                # Sort results
                sorted_data = self.sort_data(results, data)
                # Create pillow img 
                pil_img = self.create_pil_img()
                # Plot leaderboard on pillow img
                self.plot_txt_on_pil_img(pil_img, sorted_data)
                # Create ROS img from pillow img
                ros_img = monitorResults.convert_pil_to_ros_img(pil_img)
                # Publish ROS img
                self.leaderboard_pub.publish(ros_img)
            
            rate.sleep()              

                

    @staticmethod
    def convert_pil_to_ros_img(img):
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

            #self.ros_img = fpvGUI.create_ros_img(self, pil_img)
            #self.image_pub.publish(self.ros_img)

# Must be a better way to do this?!
# https://stackoverflow.com/questions/715417/converting-from-a-string-to-boolean-in-python
# I remember having this problem before, ask around about it!
def str2bool(v):
  return v.lower() in ("yes", "true", "t", "1")
  
if __name__ == '__main__':
    try:
        mr = monitorResults(sys.argv[1])
        mr.run()

    except rospy.ROSInterruptException:
        pass

      
      