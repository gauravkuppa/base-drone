#Python Version

# Import ros, Imu.h, WiringPi, WiringPiI2c
#import rospy
#pip install wiringpi
#pip install PiI2C as well probably

from sensor_msgs.msg import Imu


I2C_address = 0x00 #placeholder
POWER_MAN = 0x00 #placeholder


#read word function?
''' 
 high value reading
 low value rading
 value reading
 return the read value????
''' 
def read_word_2c(fd, address):
	high = wiringPiI2CReadReg8(fd, address)
    low = wiringPiI2CReadReg8(fd, address+1)
    val = (high << 8) + low
    return float((val >= 0x8000) ? -((65535 - val) + 1) : val)
    #^Double check what these address locations may mean


#Main function. establish nodes, messages, publishers, and subs
''' 
	find device, if not foundm exit

	sPossibly wake up Pi w/ wiringPiI2cWriteReg16() command


	initialize ros node
	nodeHandle
	publisher node (figure out name later) and advertise it to sensor_msgs::IMu
	set the rate

'''
def main: #lmk if this is needed

#https://www.instructables.com/id/Raspberry-Pi-I2C-Python/
#Install I2C into Pi, test if it works
	#I2C address
	fd = wiringPiI2CSetup(I2C_address)
	if (fd == -1):
		print("I2C Device not found \n")
		return -1

	#Wake function, find some documentation
	wiringPiI2CWriteReg16(fd, POWER_MAN, 0)

	pub = rospy.Publisher('imu', )
	rospy.init_node('MPU6050', anonymous = True)
	rate = rospy.Rate(10) #Change form 10Hz to whatever sampling we use

#Publishing loop
	while not rospy.is_shutdown():
		sensor_msgs::Imu msg
		msg.header.stamp = rospy.Time.now()
		msg.header.frame_id = '0'

		#Ways to figure out these addresses??
		msg.angular_velocity.x = read_word_2c()
		msg.angular_velocity.y = read_word_2c() 
		msg.angular_velocity.y = read_word_2c()

		msg.linear_acceleration.x = read_word_2c()
    	msg.linear_acceleration.y = read_word_2c()
    	msg.linear_acceleration.z = read_word_2c()

    	pub.publish(msg)
    	rospy.spin()
    	rospy.sleep(rate)

''' 
	declare message
	put a time stamp 
	and a frame_id to the message



	read gyroscope values using message variable API calls w/ PiI2C

	create a rescale value? depending on sensitivity
	read accelerometer values using message var API calls w/ PiI2C


	publish the message
	spinOnce
	sleep
'''

		

	
 