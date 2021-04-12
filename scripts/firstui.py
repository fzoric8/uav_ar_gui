#!/usr/bin/env python3
import rospy
import sys
from nav_msgs.msg import Odometry
from PIL import Image as SensorImage, ImageFont,ImageDraw

class PrintPosition():
    def __init__ (self, frequency): 
        self.frequency = int(frequency) 
        self.image_pub = rospy.Publisher('/uav/gui', SensorImage, queue_size=10)
        
        rospy.init_node('drone_odom', anonymous=True)
        self.Odometry = rospy.Subscriber('/firefly/ground_truth/odometry', Odometry, self.odom_callback)
        rospy.spin()
        

    def odom_callback(self, data):
        self.odom=Odometry()
        height=str(data.pose.pose.position.z)
        self.draw_gui(height)
        print("positionz: ", height)
        

    @staticmethod
    def draw_gui(drone_height):
        img=Image.new("RGBA", (900,900), 'white')
        font=ImageFont.load_default()
        draw=ImageDraw.Draw(img)
        draw.text((0,150), drone_height, (0,0,0), font=font)
        img.save("img_with_posz")
        conv_pil_to_rosimg(img)


    def conv_pil_to_rosimg(self, img):
        img=img.convert('RGB')
        msg = SensorImage()
        msg.header.stamp = rospy.Time.now()
        msg.height = img.height
        msg.width = img.width
        msg.encoding = "rgb8"
        msg.is_bigendian = False
        msg.step = 3 * img.width
        msg.data = np.array(img).tobytes()
        self.image_pub.publish(msg)


    def run(self):

        rate = rospy.Rate(self.frequency)

        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        p = PrintPosition(sys.argv[1])
        p.run()
    except rospy.ROSInterruptException: pass
        
