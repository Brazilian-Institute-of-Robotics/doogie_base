#!/usr/bin/env python
# license removed for brevity
import rospy
from doogie_msgs.msg import MazeCellMultiArray
from doogie_msgs.msg import MazeCell
from std_msgs.msg import MultiArrayDimension

class MatrixHandle():
    def __init__(self):
        self.mat = MazeCellMultiArray()
        self.mat.layout.dim.append(MultiArrayDimension())
        self.mat.layout.dim.append(MultiArrayDimension())


    def initMatrix(self, iSize, jSize, iLabel, jLabel, fill = 0, offset = 0):
        self.mat.layout.dim[0].label = iLabel
        self.mat.layout.dim[1].label = jLabel
        self.mat.layout.dim[0].size = iSize
        self.mat.layout.dim[1].size = jSize
        self.mat.layout.dim[0].stride = iSize*jSize
        self.mat.layout.dim[1].stride = jSize
        self.mat.layout.data_offset = offset
        emptyList = []
        for i in range(iSize):
            for j in range(jSize):
                emptyList.append(MazeCell())
        self.mat.data = emptyList


    def getMatrixElement(self, matrix, i, j):
        if type(matrix) is MazeCellMultiArray:
            dstride1 = matrix.layout.dim[1].stride
            offset = matrix.layout.data_offset
            return matrix.data[offset + i + dstride1*j]
        else:
            raise Exception('The array should be an MazeCellMultiArray')
    

    def setMatrixElement(self, matrix, i, j, northElement, southElement, eastElement, westElement, visited):
        if type(matrix) is MazeCellMultiArray:
            dstride1 = matrix.layout.dim[1].stride
            offset = matrix.layout.data_offset
            matrix.data[offset + i + dstride1*j].north_wall = northElement
            matrix.data[offset + i + dstride1*j].south_wall = southElement
            matrix.data[offset + i + dstride1*j].east_wall = eastElement
            matrix.data[offset + i + dstride1*j].west_wall = westElement
            matrix.data[offset + i + dstride1*j].visited = visited

            return matrix.data[offset + i + dstride1*j]
        else:
            raise Exception('The array should be an MazeCellMultiArray')




