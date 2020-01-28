#!/usr/bin/env python

import xmlrpclib

import rospy

from sensor_msgs.msg import JointState


class CR200ROS(object):

    GRIPKIT_LOW = 0.02341/2
    GRIPKIT_HIGH = 0.08498/2

    def __init__(self, options):
        connstr = "http://{}:{}/RPC2".format(options['host'], options['port'])
        self.grpc = xmlrpclib.ServerProxy(connstr)
        self.gid = self.grpc.GetGrippers()[0]

        self.pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

    def publish_forever(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            pos = self.grpc.GetPos(self.gid) / 1000.0

            js = JointState()
            js.header.stamp = rospy.Time.now()
            js.header.frame_id = ''
            js.name = ['left_finger_joint', 'right_finger_joint']

            fpos = pos / 2 - self.GRIPKIT_LOW
            js.position = [fpos, fpos]
            self.pub.publish(js)

            rate.sleep()


def main():
    rospy.init_node('cr200ros')

    options = {'host': "10.1.0.2", 'port': 44221}
    gripper = CR200ROS(options)

    gripper.publish_forever()

if __name__ == '__main__':
    main()
