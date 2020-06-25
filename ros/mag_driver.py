import rospy
import smbus
from sensor_msgs.msg import Imu

ADDR = None
bus = None
MAG_FRAME = None
mag_pub = None


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

def publish_mag(timer_e):
    mag_msg = Imu()
    mag_msg.header.frame_id = MAG_FRAME
    mag_x = read_word_2c()
    mag_y = read_word_2c()
    mag_z = read_word_2c()




def main():
    rospy.init_node('mag_node', anonymous=True)
    bus = smbus.SMBus(rospy.get_param('~bus', 1)) # what is this rospy.get_param('-bus') do?
    
    # TODO: ask Raj to figure out I2C address, https://www.melexis.com/-/media/files/documents/datasheets/mlx90393-datasheet-melexis.pdf, pg. 5 
    ADDR = rospy.get_param('-device_address', 0x0C) # or 0x18
    MAG_FRAME = rospy.get_param('-mag_frame', 'mag_link')
    # do i need to send power to board again?
    mag_pub = rospy.Publisher('mag/data', Imu)
    mag_timer = rospy.Timer(rospy.Duration(1/10), publish_mag)
    rospy.spin()

if __name__ == '__main__':
	main()