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
        rospy.init_node('look_up')
        rospy.Service('/robobreizh/manipulation/look_up', EmptySrv, self.animate)
        rospy.spin()

    def animate(self,req):

        # Example showing multiple trajectories
        names      = ["HeadPitch","HeadYaw"]
        angleHeadPitch = math.radians(-20.0)
        pitchAngleLists = [angleHeadPitch]
        angleLists = [pitchAngleLists,[0.0]]

        # set times for joints every seconds
        pitchTimeLists = [1.0]
        timeLists = [pitchTimeLists,[1.0]]
        isAbsolute = True
        self.motion_service.angleInterpolation(names, angleLists, timeLists, isAbsolute)
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

