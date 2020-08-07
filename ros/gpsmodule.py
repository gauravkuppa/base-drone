#To Do: NavSatFix message being sent to a navsat_transform_node
''' 
https://medium.com/@ericmaggard/building-an-autonomous-car-using-a-1-10th-scale-rc-car-part-2-38aa8ee9abed
    The GPS Module requires some initial setup on the pi for UART serial communication 
    Go to the adafruit section to see the commands/file edits done to set it up

    pip install pynmea2 once working. 
    catkin_make and install

'''

#!/usr/bin/env python
import rospy
import serial
import pynmea2 #nmea is a GPS related data
from sensor_msgs.msg import NavSatFix
s = serial.Serial('/dev/ttyS0', 9600)
if s.isOpen() is False:
    sys.exit(1)
def gps_talker():
    pub = rospy.Publisher('navsat', NavSatFix, queue_size=10)
    rospy.init_node('gps_talker', anonymous=True)
    msg = NavSatFix()
    seq = 0
    while not rospy.is_shutdown():
        line = s.readline()
        data = pynmea2.parse(line)
        if line[1:5] == 'GPRM':
            seq += 1
            msg.header.seq = seq
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'world'
            if data.status == 'A':
                msg.status.status = int(1)
            else:
                msg.status.status = int(0)
            msg.status.service = int(0)
            msg.latitude = float(data.lat)
            msg.longitude = float(data.lon)
            # data.true_course=142.46
            # data.spd_over_grnd
        if line[1:5] == 'GPGG':
            seq += 1
            msg.header.seq = seq
            msg.latitude = float(data.lat)
            msg.longitude = float(data.lon)
            msg.altitude = float(data.altitude)*3.28084
        pub.publish(msg)
if __name__ == '__main__':
    try:
        gps_talker()
    except rospy.ROSInterruptException:
        pass