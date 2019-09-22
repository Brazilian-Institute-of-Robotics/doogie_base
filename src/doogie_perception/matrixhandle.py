#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import UInt8MultiArray
from std_msgs.msg import MultiArrayDimension

class MatrixHandle():
    def __init__(self):
        self.mat = UInt8MultiArray()
        self.mat.layout.dim.append(MultiArrayDimension())
        self.mat.layout.dim.append(MultiArrayDimension())


    def initMatrix(self, iSize, jSize, iLabel, jLabel, fill, offset = 0):
        self.mat.layout.dim[0].label = iLabel
        self.mat.layout.dim[1].label = jLabel
        self.mat.layout.dim[0].size = iSize
        self.mat.layout.dim[1].size = jSize
        self.mat.layout.dim[0].stride = iSize*jSize
        self.mat.layout.dim[1].stride = jSize
        self.mat.layout.data_offset = offset
        self.mat.data = [fill]*iSize*jSize


    def getMatrixElement(self, matrix, i, j):
        if type(matrix) is UInt8MultiArray:
            dstride1 = matrix.layout.dim[1].stride
            offset = matrix.layout.data_offset
            return matrix.data[offset + i + dstride1*j]
        else:
            raise Exception('The array should be an UInt8MultiArray')
    

    def setMatrixElement(self, matrix, i, j, element):
        if type(matrix) is UInt8MultiArray:
            dstride1 = matrix.layout.dim[1].stride
            offset = matrix.layout.data_offset
            matrix.data[offset + i + dstride1*j] = element
            return matrix.data[offset + i + dstride1*j]
        else:
            raise Exception('The array should be an UInt8MultiArray')




