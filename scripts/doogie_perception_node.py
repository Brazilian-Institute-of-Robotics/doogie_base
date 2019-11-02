#!/usr/bin/env python
# license removed for brevity
import rospy
from doogie_perception.matrixhandle import MatrixHandle 
from doogie_msgs.msg import MazeCell
from doogie_msgs.msg import MazeCellMultiArray
from doogie_msgs.msg import UpdateMatrix
from std_msgs.msg import MultiArrayDimension
<<<<<<< HEAD
from doogie_test.msg import IrSensors

class DoogiePerception:


    def __init__(self):
        
        self.matrix = MatrixHandle()

        self.matrix.initMatrix(16, 16, "Linha", "Coluna")
        rospy.init_node('doogie_perception_node', anonymous=True)

        
        self.matrix_publisher = rospy.Publisher('maze_walls_matrix', MazeCellMultiArray, queue_size=1)
        self.sensors_subscriber = rospy.Subscriber('ir_sensors', IrSensors, self.process_sensors_data)
        self.update_matrix_subsriber = rospy.Subscriber('update_matrix', UpdateMatrix, self.update_matrix)

        self.sensors = IrSensors()

        self.left_wall = bool()
        self.right_wall = bool()
        self.front_wall = bool()

        self.north_wall = bool()
        self.south_wall = bool()
        self.east_wall = bool()
        self.west_wall = bool()

        self.maze_cell_row = int()
        self.maze_cell_column = int()
        self.doogie_orientation = int()

        self.rate = rospy.Rate(10)
        

#recebe os dados dos sensores do topico e converte para informacao booleana
    def process_sensors_data(self, data): 
        
        constant = 20

        self.sensors = data
        if self.sensors.front_left_sensor.range >= constant:
            self.left_wall = False
        else:
            self.left_wall = True
        if self.sensors.front_right_sensor.range >= constant:
            self.right_wall = False
        else:
            self.right_wall = True
        if (self.sensors.left_sensor.range >= constant) and (self.sensor.right_sensor.range >= constant):
            self.front_wall = False
        else:
            self.front_wall = True

        self.define_wall_presence(self.maze_cell_row, self.maze_cell_column, self.doogie_orientation)
        self.publish_matrix(self.maze_cell_row, self.maze_cell_column, self.doogie_orientation)



    def define_wall_presence(self, row, column, orientation):
        
        if orientation == 1:

            self.north_wall = self.front_wall
            if (row - 1) < 0:
                self.south_wall = True
            else:
                self.south_wall = self.matrix.getMatrixElement(self.matrix.mat, (row - 1) , column).north_wall
            self.east_wall = self.right_wall
            self.west_wall = self.left_wall

        if orientation == 2:

            self.north_wall = self.left_wall
            self.south_wall = self.right_wall
            self.east_wall = self.front_wall
            if (column - 1) < 0:
                self.west_wall = True
            else:
                self.west_wall = self.matrix.getMatrixElement(self.matrix.mat, row, (column - 1)).east_wall

        if orientation == 3:

            if row + 1 > 15:
                self.north_wall = True
            else:
                self.north_wall = self.matrix.getMatrixElement(self.matrix.mat, (row + 1), column).south_wall
            self.south_wall = self.front_wall
            self.east_wall = self.left_wall
            self.west_wall = self.right_wall

        if orientation == 4:

            self.north_wall = self.right_wall
            self.south_wall = self.left_wall
            if (column + 1) > 15:
                self.east_wall = True
            else:
                self.east_wall = self.matrix.getMatrixElement(self.matrix.mat, row, (column + 1)).west_wall
            self.west_wall = self.front_wall


    def update_matrix(self, data):
        self.maze_cell_row = data.row
        self.maze_cell_column = data.column
        self.doogie_orientation = data.orientation


    def publish_matrix(self, row, column, orientation):
        if row not in range(16) or column not in range(16) or orientation not in range(1,5):
            rospy.logerr("Parameters out of boundries")
        else:
            self.define_wall_presence(row, column, orientation)
            self.matrix.setMatrixElement(self.matrix.mat, row, column, self.north_wall, self.south_wall, self.east_wall, self.west_wall, True)
            print(self.front_wall)
            print(self.right_wall)
            print(self.left_wall)
            self.matrix_publisher.publish(self.matrix.mat)

        # rospy.loginfo("{}, {}, {}".format(req.row, req.column, req.orientation))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        doogie_perception = DoogiePerception()
        doogie_perception.run()
=======
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
>>>>>>> 55b8db0c93de60dde2ea5d250dcc4bb7ebba019f
    except rospy.ROSInterruptException:
        pass
