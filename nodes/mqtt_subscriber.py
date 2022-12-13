#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import paho.mqtt.client as mqtt
import time


def on_message(client, userdata, msg):
    # ros publisher publishes data received by the mqtt subscriber
    msg_string = str(msg.payload, 'utf-8')
    info = msg_string.split(",")
    cmd_web.linear.x = float(info[0]);cmd_web.linear.y = 0.0; cmd_web.linear.z = 0.0
    cmd_web.angular.x = 0.0; cmd_web.angular.y = 0.0; cmd_web.angular.z = float(info[1])
    rospy.loginfo("Received the message - %s", str(cmd_web))
    pub.publish(cmd_web)



try:
    rospy.init_node('distance_turtlebot3_teleop')
    pub = rospy.Publisher('cmd_web', Twist, queue_size=10)
    rate = rospy.Rate(1)
    cmd_web = Twist()

    # mqtt subscriber to the mqtt publisher
    mqtt_broker = "mqtt.eclipseprojects.io"
    client = mqtt.Client("Robot")
    client.connect(mqtt_broker)

    client.loop_start()
    client.subscribe("mqtt_topic")
    client.on_message = on_message 

    rospy.spin()
    

except rospy.ROSInterruptException:
    print("Exception has occured")
    pass

    