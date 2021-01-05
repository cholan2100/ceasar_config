#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Float64
from std_msgs.msg import Header


from sensor_msgs.msg import JointState

class Actuator():
    def mapFromTo(self, x,a,b,c,d):
        y=(x-a)/(b-a)*(d-c)+c
        return y

    def __init__(self):
        self.conf = {}
        self.conf["front_left_foot"] = [90,510]


        rospy.Subscriber("joint_group_position_controller/command", JointTrajectory, self.callback)   #fixme msg
        pub = rospy.Publisher('joint_states', JointState, queue_size=10)  #fixme msg, rate
        rospy.init_node('ceasar_actuator', anonymous=True)
        rate = rospy.Rate(50)
        rospy.loginfo("Running Ceasar Actuator node")
        self.data = None
        while not rospy.is_shutdown():
            if self.data:
                hello_str = JointState()
                hello_str.header = Header()
                hello_str.header.stamp = rospy.Time.now()
                hello_str.name = self.data.joint_names
                hello_str.position = self.data.points[0].positions
                hello_str.velocity = []
                hello_str.effort = []
                pub.publish(hello_str)
                # rospy.loginfo(self.data.points[0].positions[0]) # LFS
                # rospy.loginfo(self.data.points[0].positions[2]) # LFL
            rate.sleep()

    def callback(self, data):
        self.data = data
        lfu_pos = data.points[0].positions[1]
        lfl_pos = data.points[0].positions[2]

        #TODO: translate radian to PWM
        lfu_pwm = self.mapFromTo(lfu_pos, -2.67, 1.55, 81, 501)
        lfl_pwm = self.mapFromTo(lfl_pos, 0.0, -1*180*(3.14/180), 90, 510)

        # rospy.loginfo("LFL deg %f - %d" % (lfl_pos * (180/3.14), lfl_pwm))
        rospy.loginfo("LFU deg %f - %d" % (lfu_pos * (180/3.14), lfu_pwm))
        #TODO: PCA9685 PWM update
        pass

if __name__ == '__main__':
    try:
        a = Actuator()
    except rospy.ROSInterruptException:
        pass