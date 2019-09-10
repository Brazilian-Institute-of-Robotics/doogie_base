#!/usr/bin/env python
# license removed for brevity
import rospy
from doogie_perception.matrixhandle import MatrixHandle 
from std_msgs.msg import UInt8MultiArray
from std_msgs.msg import MultiArrayDimension

def talker():
    pub = rospy.Publisher('doogie_wall_perception', UInt8MultiArray, queue_size=1)
    rospy.init_node('doogie_perception_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    test = "test"
    mh = MatrixHandle()
    mh.initMatrix(3, 3, "Linha", "Coluna", -1)
    while not rospy.is_shutdown():        
        #rospy.loginfo(matrix)
        rospy.loginfo("%d", mh.getMatrixElement(mh.mat, 2, 2))
        pub.publish(mh.mat)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
