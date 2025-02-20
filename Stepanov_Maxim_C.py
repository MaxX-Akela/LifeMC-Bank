from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import rospy

bridge = CvBridge()

color = 'error'

def color_callback(data):
    global color
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
    img_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)[119:120, 159:160] # Still using a single pixel, consider a region of interest (ROI) for robustness

    # HSV ranges for yellow.  These are approximate and may need adjustment based on your lighting conditions and camera.
    yellow_low_value = (20, 100, 100)  # Adjust these values as needed
    yellow_high_value = (30, 255, 255) # Adjust these values as needed

    yellow_final = cv2.inRange(img_hsv, yellow_low_value, yellow_high_value)

    if yellow_final[0][0] == 255:
        color = 'yellow'
    else:
        color = 'error'

    #34 141 123
    #49 112 125

#rospy initialization MUST be before subscriber creation
rospy.init_node('color_detector', anonymous=True)

# Subscribe to the image topic.  Make sure this topic name is correct for your robot.
image_sub = rospy.Subscriber("main_camera/image_raw_throttled", Image, color_callback)

# Keep the node running.  This is crucial for ROS.
rospy.spin()