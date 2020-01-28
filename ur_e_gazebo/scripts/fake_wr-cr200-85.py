#!/usr/bin/env python

import threading
import rospy

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

from SocketServer import ThreadingMixIn
from SimpleXMLRPCServer import SimpleXMLRPCServer

class ThreadedSimpleXMLRCPServer(ThreadingMixIn, SimpleXMLRPCServer):
    pass

class FakeWSCR200Gripper:

    VELOCITY = 0.05

    GRIPKIT_NOT_REFERENCED = 0
    GRIPKIT_IDLE = 1
    GRIPKIT_RELEASED = 2
    GRIPKIT_NO_PART = 4
    GRIPKIT_HOLDING = 8

    def __init__(self):
        self.gid = "CRG 200-085 SN:000610"
        self.state = self.GRIPKIT_IDLE
        self.low  = 0.04
        self.high = 0.08

        self.pos = 0.04

        self._hard_low = 0.04
        self._hard_high = 0.08

        self.cmdp = rospy.Publisher("/gripper_controller/command", Float64MultiArray, queue_size=1)
        self.jsub = rospy.Subscriber("/joint_states", JointState, self._on_joint_state, queue_size=1)

        rospy.loginfo("Fake Weiss Robotics CR-200-85 Gripper in now online")

    def _on_joint_state(self, joints):
        j = {'left_finger_joint': 0.0, 'right_finger_joint': 0.0}
        for i, name in enumerate(joints.name):
            if name in j:
                j[name] = joints.position[i]

        pos = (j['left_finger_joint'] + j['right_finger_joint']) /2.0
        self.pos = self._hard_low + pos

    def Grip(self, gid, idx):
        if gid != self.gid or idx != 1:
            return -1

        if self.state == self.GRIPKIT_HOLDING:
            return -1

        pos = self.low / 2.0 - self._hard_low / 2.0
        self.cmdp.publish(Float64MultiArray(data=[pos, pos]))
        rospy.sleep(abs(self.pos - self.low)/self.VELOCITY)

        has_part = abs(self.pos - self.low) > 0.005
        self.state = self.GRIPKIT_HOLDING if has_part else self.GRIPKIT_NO_PART
        # TODO: validate the grip
        return 0

    def Release(self, gid, idx):
        if gid != self.gid or idx != 1:
            return -1

        if self.state == self.GRIPKIT_IDLE:
            return -1

        pos = self.high / 2.0 - self._hard_low / 2.0
        self.cmdp.publish(Float64MultiArray(data=[pos, pos]))
        rospy.sleep(abs(self.pos - self.high)/self.VELOCITY)

        self.state = self.GRIPKIT_RELEASED
        self.pos = self.high
        return 0

    def Disable(self, gid, idx):
        if gid != self.gid or idx != 1:
            return -1

        self.state = self.GRIPKIT_IDLE
        return 0

    def Home(self, gid):
        pass

    def Reference(self, gid):
        pass

    def GetState(self, gid, idx):
        if gid != self.gid or idx != 1:
            return -1

        return self.state

    def GetTempWarn(self, gid, idx):
        pass

    def GetTempFault(self, gid, idx):
        pass

    def GetFault(self, gid, idx):
        pass

    def GetMaintenance(self, gid, idx):
        pass

    def GetPos(self, gid):
        if gid != self.gid:
            return -1

        return self.pos * 1000.0

    def GetGrippers(self):
        return [self.gid]

    def GetParameterChecksum(self, gid):
        pass

    def StoreRemanent(self, gid):
        pass

    def GetReleaseLimit(self, gid, idx):
        pass

    def GetNoPartLimit(self, gid, idx):
        pass

    def GetForce(self, gid, idx):
        pass

    def GetOverrideRelease(self, gid, idx):
        pass

    def GetOverrideGrip(self, gid, idx):
        pass

    def GetHomingDirection(self, gid, idx):
        pass

    def GetReferenceDirection(self, gid, idx):
        pass

    def SetReleaseLimit(self, gid, idx, limit):
        pass

    def SetNoPartLimit(self, gid, idx, limit):
        pass

    def SetForce(self, gid, idx, force):
        pass

    def SetOverrideRelease(self, gid, idx, velocity):
        pass

    def SetOverrideGrip(self, gid, idx, velocity):
        pass

    def SetHomingDirection(self, gid, idx, direction):
        pass

    def SetReferenceDirection(self, gid, idx, direction):
        pass

    def SetLEDPreset(self, gid, idx, preset):
        pass

    def SetLEDAnimation(self, gid, idx, animation):
        pass

    def SetLEDColor(self, gid, idx, color):
        pass

    def SetLEDSpeed(self, gid, idx, speed):
        pass


def main():
    rospy.init_node("fake_wrcr_gripper")

    server = ThreadedSimpleXMLRCPServer(("localhost", 44221))
    server.register_instance(FakeWSCR200Gripper())
    server.register_introspection_functions()

    server_thread = threading.Thread(target=server.serve_forever)
    # Exit the server thread when the main thread terminates
    server_thread.daemon = True
    server_thread.start()

    rospy.spin()
    server.shutdown()
    server.server_close()


if __name__ == '__main__':
    main()
