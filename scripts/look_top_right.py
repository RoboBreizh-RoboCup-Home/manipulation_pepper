import time
import qi
import sys
import math


class Motion():
    def __init__(self, app):
        app.start()
        session = app.session
        self.motion_service = session.service("ALMotion")
        self.motion_service.setStiffnesses("Head", 1.0)

    def animate(self):

        # Example showing multiple trajectories
        names      = ["HeadPitch","HeadYaw"]
        angleHeadPitch = math.radians(-35.0)
        angleHeadYaw= math.radians(-35.0)
        pitchAngleLists = [angleHeadPitch]
        angleLists = [pitchAngleLists,[angleHeadYaw]]

        # set times for joints every seconds
        pitchTimeLists = [1.0]
        timeLists = [pitchTimeLists,[1.0]]
        isAbsolute = True
        print(angleLists)
        print(timeLists)
        self.motion_service.angleInterpolation(names, angleLists, timeLists, isAbsolute)


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
    motion.animate()

