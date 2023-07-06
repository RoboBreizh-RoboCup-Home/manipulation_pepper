import time
import qi
import sys
import math
import rospy
from manipulation_pepper.srv import EmptySrv,EmptySrvResponse

class Motion():
    def __init__(self, app):
        app.start()
        session = app.session
        self.motion_service = session.service("ALMotion")
        self.motion_service.setStiffnesses("Head", 1.0)
        rospy.init_node('point_in_front')
        rospy.Service('/robobreizh/manipulation/point_in_front', EmptySrv, self.animate)
        rospy.spin()
    def arrDegreeToRad(self,angles:list):
        res = []
        for angle in angles:
            res.append(math.radians(angle))
        return res

    def animate(self,req):

        # Example showing multiple trajectories
        names = ["LShoulderPitch","LShoulderRoll","LElbowYaw","LElbowRoll","LWristYaw"]

        LShoulderPitchAngleLists = self.arrDegreeToRad([0.0]) # -119.5 / 119.5
        LShoulderRollAngleLists = self.arrDegreeToRad([0.0]) # -89.5 / -0.5
        LElbowYawAngleLists = self.arrDegreeToRad([0.0]) # -119.5 / 119.5
        LElbowRollAngleLists = self.arrDegreeToRad([0.0]) # 0.5 / -89.5
        LWristYawAngleLists = self.arrDegreeToRad([90.0]) # -104.5 / 104.5

        angleLists = [LShoulderPitchAngleLists,LShoulderRollAngleLists,LElbowYawAngleLists,LElbowRollAngleLists,LWristYawAngleLists]

        # set times for joints every seconds
        LShoulderPitchTimeLists = [5.0]
        LShoulderRollTimeLists = [1.0]
        LElbowYawTimeLists = [1.0]
        LElbowRollTimeLists =[1.0]
        LWristYawTimeLists = [1.0]
        yawTimeLists = [1.0]
        pitchTimeLists = [1.0]
        timeLists = [LShoulderPitchTimeLists,LShoulderRollTimeLists,LElbowYawTimeLists,LElbowRollTimeLists,LWristYawTimeLists]

        isAbsolute = True
        self.motion_service.angleInterpolation(names, angleLists, timeLists, isAbsolute)

        # self.motion_service.wakeUp()
        return EmptySrvResponse()




if __name__ == "__main__":
    try:
        connection_url = "tcp://127.0.0.1:9559"
        app = qi.Application(
            ["SoundProcessingModule", "--qi-url=" + connection_url])
    except RuntimeError:
        print("Can't connect to Naoqi at ip \"" + ip + "\" on port " + str(port) + ".\n"
              "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)
    motion = Motion(app)

