#!/usr/bin/env python

# import needed for the mqtt broker
import paho.mqtt.client as mqtt
import time
from geometry_msgs.msg import Twist

# imports needed for teleop with the keyboard
import sys, select, os
if os.name == 'nt':
  import msvcrt, time
else:
  import tty, termios

mqtt_broker = "mqtt.eclipseprojects.io"
client = mqtt.Client("Teleop_key")
client.connect(mqtt_broker)

### Functions used in the teleop node given by the constructor ###
BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

msg = """
Control Your TurtleBot3!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity (Burger : ~ 0.22, Waffle and Waffle Pi : ~ 0.26)
a/d : increase/decrease angular velocity (Burger : ~ 2.84, Waffle and Waffle Pi : ~ 1.82)

space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""

def getKey():
    if os.name == 'nt':
        timeout = 0.1
        startTime = time.time()
        while(1):
            if msvcrt.kbhit():
                if sys.version_info[0] >= 3:
                    return msvcrt.getch().decode()
                else:
                    return msvcrt.getch()
            elif time.time() - startTime > timeout:
                return ''

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def checkLinearLimitVelocity(vel):
    vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    return vel

def checkAngularLimitVelocity(vel):
    vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    return vel


status = 0
target_linear_vel   = 0.0
target_angular_vel  = 0.0
control_linear_vel  = 0.0
control_angular_vel = 0.0

if os.name != 'nt':
    settings = termios.tcgetattr(sys.stdin)

while True:
    #try:
    key = getKey()
    if key == 'w' :
        target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
        status = status + 1
        print(vels(target_linear_vel,target_angular_vel))
    elif key == 'x' :
        target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
        status = status + 1
        print(vels(target_linear_vel,target_angular_vel))
    elif key == 'a' :
        target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
        status = status + 1
        print(vels(target_linear_vel,target_angular_vel))
    elif key == 'd' :
        target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
        status = status + 1
        print(vels(target_linear_vel,target_angular_vel))
    #elif key == ' ' or key == 's' :
    elif key == 's' :
        target_linear_vel   = 0.0
        control_linear_vel  = 0.0
        target_angular_vel  = 0.0
        control_angular_vel = 0.0
        print(vels(target_linear_vel, target_angular_vel))
    else:
        if (key == '\x03'):
            break

    if status == 20 :
        print(msg)
        status = 0

    #print("before twist")
    #twist = Twist()
    #print("after twist, twist is", twist)

    control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
    #twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
    #twist_linear = [control_linear_vel, 0.0, 0.0]

    control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
    #twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel
    #twist_angular = [0.0, 0.0, control_angular_vel]

    # Mqqt broker instead of ROS publisher
    #twist = bytearray([control_linear_vel, control_angular_vel])
    #print("before publish twist is", twist)
    payload = str(control_linear_vel)+","+str(control_angular_vel)
    print("in mqtt", type(payload))
    client.publish("mqtt_topic", payload=str(control_linear_vel)+","+str(control_angular_vel))
    #print("after publish")
    print("Just published " + str(control_linear_vel)+","+str(control_angular_vel) + " to TOPIC mqtt_topic")
    #time.sleep(1)

    # except:
    #     print("except")
    #     pass
        #print(e)