#!/usr/bin/env python
# license removed for brevity
import rospy
from doogie_perception.matrixhandle import MatrixHandle 
from std_msgs.msg import UInt8MultiArray
from std_msgs.msg import MultiArrayDimension
from sensor_msgs.msg import Range
from std_msgs.msg import UInt8

from doogie_msgs.srv import *
import Adafruit_ADS1x15

# Create the I2C bus
adc = Adafruit_ADS1x15.ADS1115()

GAIN = 1
class DoogiePerception:

    def __init__(self):

        rospy.init_node('doogie_perception_node', anonymous=True)
        #pub = rospy.Publisher('doogie_wall_perception', UInt8MultiArray, queue_size=1)

        self.pub_sensor_ir_left = rospy.Publisher('sensor_ir_left', Range, queue_size=1)
        self.pub_sensor_ir_front_left = rospy.Publisher('sensor_ir_front_left', Range, queue_size=1)
        self.pub_sensor_ir_front_right = rospy.Publisher('sensor_ir_front_right', Range, queue_size=1)
        self.pub_sensor_ir_right = rospy.Publisher('sensor_ir_right', Range, queue_size=1)

        # self.pub_test_row = rospy.Publisher('row_test', UInt8, queue_size=1)
        # self.pub_test_col = rospy.Publisher('column_test', UInt8, queue_size=1)
        # self.pub_test_ori = rospy.Publisher('orientation_test', UInt8, queue_size=1)

        self.matrix_row = 10
        self.matrix_column = 10
        self.doogie_orientation = 10

        self.left_sensor_msg = Range()
        self.front_left_sensor_msg = Range()
        self.front_right_sensor_msg = Range()
        self.right_sensor_msg = Range()

        self.rate = rospy.Rate(10) # 10hz

    def handle_update_matrix(self, req):
        self.matrix_row = req.row
        self.matrix_column = req.column
        self.doogie_orientation = req.orientation

        return UpdateMatrixResponse(True)


    def d_perception(self):

        print "Server running"

        service = rospy.Service('update_matrix', UpdateMatrix, self.handle_update_matrix)

        # mh = MatrixHandle()
        # mh.initMatrix(3, 3, "Linha", "Coluna", -1)
        # mh.setMatrixElement(mh.mat, 2, 2, 52)
        while not rospy.is_shutdown():
            now = rospy.get_rostime()
            
            left_sensor_read = float(adc.read_adc(0, gain=GAIN))
            front_left_sensor_read = float(adc.read_adc(1, gain=GAIN))
            front_right_sensor_read = float(adc.read_adc(2, gain=GAIN))
            right_sensor_read = float(adc.read_adc(3, gain=GAIN))
            #left sensor message
            self.left_sensor_msg.radiation_type = self.left_sensor_msg.INFRARED
            self.left_sensor_msg.header.stamp.secs = now.secs
            self.left_sensor_msg.header.stamp.nsecs = now.nsecs
            self.left_sensor_msg.header.frame_id = 'left sensor' 
            self.left_sensor_msg.range = left_sensor_read
            
            #front left sensor message
            self.front_left_sensor_msg.radiation_type = self.front_left_sensor_msg.INFRARED
            self.front_left_sensor_msg.header.stamp.secs = now.secs       
            self.front_left_sensor_msg.header.stamp.nsecs = now.nsecs       
            self.front_left_sensor_msg.header.frame_id = 'front left sensor'       
            self.front_left_sensor_msg.range = front_left_sensor_read

            #front right sensor message
            self.front_right_sensor_msg.radiation_type = self.front_right_sensor_msg.INFRARED
            self.front_right_sensor_msg.header.stamp.secs = now.secs       
            self.front_right_sensor_msg.header.stamp.nsecs = now.nsecs       
            self.front_right_sensor_msg.header.frame_id = 'front right sensor'       
            self.front_right_sensor_msg.range = front_right_sensor_read    

            #right sensor message
            self.right_sensor_msg.radiation_type = self.right_sensor_msg.INFRARED
            self.right_sensor_msg.header.stamp.secs = now.secs       
            self.right_sensor_msg.header.stamp.nsecs = now.nsecs       
            self.right_sensor_msg.header.frame_id = 'right sensor'       
            self.right_sensor_msg.range = right_sensor_read     

            #rospy.loginfo(matrix)
            #rospy.loginfo("%d", mh.getMatrixElement(mh.mat, 2, 2))
            self.pub_sensor_ir_left.publish(self.left_sensor_msg)
            self.pub_sensor_ir_front_left.publish(self.front_left_sensor_msg)
            self.pub_sensor_ir_front_right.publish(self.front_right_sensor_msg)
            self.pub_sensor_ir_right.publish(self.right_sensor_msg)

            self.pub_test_row.publish(self.matrix_row)
            self.pub_test_col.publish(self.matrix_column)
            self.pub_test_ori.publish(self.doogie_orientation)



            self.rate.sleep()

if __name__ == '__main__':
    try:
        d = DoogiePerception()
        d.d_perception()
    except rospy.ROSInterruptException:
        pass
