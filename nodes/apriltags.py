#!/usr/bin/env python

import cv2
from pupil_apriltags import Detector
from numpy import asarray, uint8, arctan2, pi
from PIL import Image
import rospy
from geometry_msgs.msg import Pose2D
import argparse

# [x, y] position corresponding to each AprilTag
pos_image_1 = [0, 0]
pos_image_2 = [5, 5]
pos_image_3 = [-5, 5]
id_to_position = {20:pos_image_1, 21:pos_image_2, 22:pos_image_3}

# Parser to decide which file to analyse, or if we should use the webcam instead
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--image", required=False, help="path to input image containing AprilTag", default="/home/victor/tags_test_iFollow/ar_tag_1.JPG")
ap.add_argument("--webcam", required=False, help="decides to use image from the webcam instead of a jpg file", default=False)
args = vars(ap.parse_args())

if args["webcam"] == False:
   # the webcam is not used
   initial_image = args["image"]


else:
   # the webcam is used
   cap = cv2.VideoCapture(0)
   while True:
      if cap.isOpened():
         print("it is open")
         ret, webcam_frame = cap.read()
         webcam_frame = cv2.resize(webcam_frame, None, fx=2, fy=2, interpolation=cv2.INTER_AREA)
         cv2.imshow("webcam_frame", webcam_frame)

         k=cv2.waitKey(1)
         if k%256 == 27:
            # ESC pressed
            print("Escape hit, closing...")
            break
         elif k%256 == 32:
            # SPACE pressed
            cv2.imwrite("webcam.jpg", webcam_frame)
            break

   cap.release()

   cv2.destroyAllWindows()

   initial_image = "webcam.jpg"


# Convert the image to gray scale because the results might be better that way
# These 3 lines could be commented 
image = cv2.imread(initial_image)
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
cv2.imwrite("gray.jpg", gray)

# Set up the AprilTag Detector
bw_image = Image.open("gray.jpg").convert('L')
print("type dw", type(bw_image))
img = asarray(bw_image, dtype=uint8)


at_detector = Detector(
   families="tag36h11",
   nthreads=1,
   quad_decimate=1.0,
   quad_sigma=0.0,
   refine_edges=1,
   decode_sharpening=0.25,
   debug=0
)

detected_tags = at_detector.detect(img)

# Create the publisher that will publish the topic nav_goal
rospy.init_node('apriltag_to_navgoal')
pub = rospy.Publisher('nav_goal', Pose2D, queue_size=10)
rate = rospy.Rate(1)
nav_goal = Pose2D()

if len(detected_tags) == 0:
   print("No AprilTag detected. Not moving")
elif len(detected_tags) == 1:
   tag_id = detected_tags[0].tag_id
   print("AprilTag number "+str(tag_id)+" detected")
   nav_goal.x = id_to_position[tag_id][0]
   nav_goal.y = id_to_position[tag_id][1]

   center = detected_tags[0].center
   A, B, C, D = detected_tags[0].corners[0], detected_tags[0].corners[1], detected_tags[0].corners[2], detected_tags[0].corners[3]
   print(center)
   print(detected_tags[0].corners)
   if B[1] == C[1]:
      # arctan is not define, so the values of theta have to be found "manually"
      if B[1] > center[1]:
         theta = 0
      else:
         theta = pi
   else :
      theta = arctan2((B[0]-C[0]),(B[1]-C[1]))
      nav_goal.theta = theta
   
   print("Found Orientation : ", theta)
      
   # I decided to publish the nav_goal over and over
   # Another idea would be to create a while loop on the all program, to change the position
   while not rospy.is_shutdown():
      print("Moving to - %s", str(nav_goal))
      pub.publish(nav_goal)
      rate.sleep()


else:
   print("Several AprilTags detected. Not moving")

   for detected_tag in detected_tags:
      # print data corresponding to the april tags detected
      print(detected_tag.tag_id)
      print(detected_tag.center)
