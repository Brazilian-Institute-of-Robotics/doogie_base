#!/usr/bin/env python
# license removed for brevity
import rospy
from doogie_perception.matrixhandle import MatrixHandle 
from doogie_msgs.msg import MazeCell
from doogie_msgs.msg import MazeCellMultiArray
from doogie_msgs.msg import UpdateMatrix
from std_msgs.msg import MultiArrayDimension
from doogie_msgs.msg import WallDistances

class DoogiePerception:


    def __init__(self):
        
        self.matrix = MatrixHandle()

        self.matrix.initMatrix(16, 16, "Linha", "Coluna")
        rospy.init_node('doogie_perception_node', anonymous=True)

        
        self.matrix_publisher = rospy.Publisher('maze_walls_matrix', MazeCellMultiArray, queue_size=1)
        self.sensors_subscriber = rospy.Subscriber('wall_distances', WallDistances, self.process_sensors_data)
        self.update_matrix_subsriber = rospy.Subscriber('update_matrix', UpdateMatrix, self.update_matrix)

        self.sensors = WallDistances()

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
        
        constant = 140

        self.sensors = data
        if self.sensors.front_left_sensor.range >= constant:
            self.left_wall = False
        else:
            self.left_wall = True
        if self.sensors.front_right_sensor.range >= constant:
            self.right_wall = False
        else:
            self.right_wall = True
        if (self.sensors.left_sensor.range >= constant) and (self.sensors.right_sensor.range >= constant):
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
    except rospy.ROSInterruptException:
        pass
