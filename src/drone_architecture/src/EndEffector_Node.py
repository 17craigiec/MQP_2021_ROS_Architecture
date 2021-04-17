#! /usr/bin/env python

from drone_architecture.msg import endEffectorControl
import rospy
import serial
import time


ser = serial.Serial('/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0', 115200, timeout=1)
setpoint = 100

def callback(data):
    ser.flush()
    if(data.m_reset):
        for _ in range(10):
            value = str(49) + "\n"
            ser.write(value.encode('utf-8'))

    else:
        global setpoint
        camera_pixel_midpoint = 110
        # tolerance = 5
        # if(data.m_YCentroid < camera_pixel_midpoint + tolerance):
        #     value = str(setpoint) + "\n"
        #     ser.write(value.encode('utf-8'))
        #     setpoint = setpoint+1
        # elif (data.m_YCentroid >  camera_pixel_midpoint - tolerance):
        #     value = str(setpoint) + "\n"
        #     ser.write(value.encode('utf-8'))
        #     setpoint = setpoint-1
        # # P = 0.05
        
        
        err = camera_pixel_midpoint - data.m_YCentroid
        print("Error" + str(err))
        print("setpoint before: " + str(setpoint))

        if(0< err <= 20):
            setpoint = setpoint+1
        elif (-20 <= err < 0):
            setpoint = setpoint-1
        elif (20 < err <= 100 or -100 < err <= -20):
            setpoint = setpoint+int(err*.05)
        elif (err > 100):
            setpoint = setpoint+5
        else:
            setpoint = setpoint-5

        if setpoint > 140:
            setpoint = 140
        elif setpoint <49:
            setpoint = 49

        value = str(setpoint) + "\n"
        ser.write(value.encode('utf-8'))

        print("centroid" + str(data.m_YCentroid))
        print("setpoint after: " + str(setpoint))
    

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('EndEffector_Node', anonymous=True)

    rospy.Subscriber("EndEffectorControl", endEffectorControl, callback, queue_size = 1)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()


