
#pip install wiringpi
#pip install PiI2C as well probably
import smbus
import rospy


from sensor_msgs.msg import Imu

#NOTE IMU IS ON 0x68
# TODO: figure out these messages
#I2C_address = 0x68 #placeholder
#POWER_MAN = 0x00 #placeholder

######## https://github.com/OSUrobotics/mpu_6050_driver/blob/master/src/mpu_6050_driver/registers.py #######
PWR_MGMT_1 = 0x6b

ACCEL_CONFIG = 0x1C
ACCEL_XOUT_H = 0x3B
ACCEL_XOUT_L = 0x3C
ACCEL_YOUT_H = 0x3D
ACCEL_YOUT_L = 0x3E
ACCEL_ZOUT_H = 0x3F
ACCEL_ZOUT_L = 0x40

GYRO_CONFIG = 0x1B
GYRO_XOUT_H = 0x43
GYRO_XOUT_L = 0x44
GYRO_YOUT_H = 0x45
GYRO_YOUT_L = 0x46
GYRO_ZOUT_H = 0x47
GYRO_ZOUT_L = 0x48

#############################################################################################################
ADDR = None
bus = None
IMU_FRAME = None
imu_pub = None

#read word function
def read_word(adr):
	high = bus.read_byte_data(ADDR, adr)
	low = bus.read_byte_data(ADDR, adr+1)
	val = (high << 8) + low
	return val


def read_word_2c(adr):

	val = read_word(adr)
	#return -((65535-val)+1) if val >= 0x8000 else val
	if(val >= 0x8000):
		return -((65535-val)+1)
	else:
		return val 


def publish_imu(timer_e):
	"""
	https://github.com/OSUrobotics/mpu_6050_driver/blob/master/scripts/imu_node.py
	"""
	img_msg = Imu()
	imu_msg.header.frame_id = IMU_FRAME

	accel_x = read_word_2c(ACCEL_XOUT_H)/16384.0
	accel_y = read_word_2c(ACCEL_YOUT_H)/16384.0	
	accel_z = read_word_2c(ACCEL_ZOUT_H)/16384.0

	# Calculate a quaternion representing the orientation
	accel = accel_x, accel_y, accel_z

	# Read the gyro vals
	gyro_x = read_word_2c(GYRO_XOUT_H) / 131.0
	gyro_y = read_word_2c(GYRO_YOUT_H) / 131.0
	gyro_z = read_word_2c(GYRO_ZOUT_H) / 131.0

	# Load up the IMU message

	imu_msg.linear_acceleration.x = accel_x
	imu_msg.linear_acceleration.y = accel_y
	imu_msg.linear_acceleration.z = accel_z

	imu_msg.angular_velocity.x = gyro_x
	imu_msg.angular_velocity.y = gyro_y
	imu_msg.angular_velocity.z = gyro_z

	imu_msg.header.stamp = rospy.Time.now()

	imu_pub.publish(imu_msg)

#Main function. establish nodes, messages, publishers, and subs

def main(): #lmk if this is needed

	rospy.init_node('imu_node', anonymous = True)
	bus = smbus.SMBus(rospy.get_param('~bus', 1))
	ADDR = rospy.get_param('-device_address', 0x68)
	if type(ADDR) == str:
		ADDR = int(ADDR, 16)
	
	IMU_FRAME = rospy.get_param('-imu_frame', 'imu_link')
	bus.write_byte_data(ADDR, PWR_MGMT_1, 0) # send power to rpi

	imu_pub = rospy.Publisher('imu/data', Imu)


	imu_timer = rospy.Timer(rospy.Duration(0.02), publish_imu)
	##not sure if above and below lines perform the same task
	while not rospy.is_shutdown():
		publish_imu()
		rospy.Rate(10)

	rospy.spin()

if __name__ == '__main__':
	main()