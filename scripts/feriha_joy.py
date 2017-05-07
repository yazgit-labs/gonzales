#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import math

import os, struct, array
from fcntl import ioctl

axis_states = {}

axis_names = {
    0x00 : 'x',
    0x01 : 'y',
    0x02 : 'z',
}

axis_map = []

fn = '/dev/input/js1'
print('Opening %s...' % fn)
jsdev = open(fn, 'rb')

buf = array.array('B', [0])
ioctl(jsdev, 0x80016a11, buf) # JSIOCGAXES
num_axes = buf[0]

buf = array.array('B', [0] * 0x40)
ioctl(jsdev, 0x80406a32, buf) # JSIOCGAXMAP

for axis in buf[:num_axes]:
    axis_name = axis_names.get(axis, 'unknown(0x%02x)' % axis)
    axis_map.append(axis_name)
    axis_states[axis_name] = 0.0

def main():
       
    cmdvel = rospy.Publisher("/feriha/cmd_vel", Twist, queue_size=1)
    rospy.init_node("Feriha_joystick", anonymous=True)
    rate = rospy.Rate(10) 
    movement_msg = Twist()
    while not rospy.is_shutdown():
        evbuf = jsdev.read(8)
        #if evbuf:
        time, value, type, number = struct.unpack('IhBB', evbuf)
        if type & 0x02:
            axis = axis_map[number]
            if axis:
                fvalue = value / 5461.0
                axis_states[axis] = fvalue
                if fvalue > 3:
                    fvalue = 6
                elif fvalue < -3:
                    fvalue = -6
                else:
                    fvalue = 0
                print "%s: %.3f" % (axis, fvalue)
                if axis == "y":
                    movement_msg.linear.x = -fvalue
                if axis == "z":
                    movement_msg.angular.z = fvalue  

        cmdvel.publish(movement_msg)
        
        rate.sleep()
                                                                                                                        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass