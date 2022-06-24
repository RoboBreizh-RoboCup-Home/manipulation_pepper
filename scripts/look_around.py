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

    def setYawPose(self,angleHeadYaw,increment):
        angleLists = []
        while angleHeadYaw < 119.5:
            angleLists.append(math.radians(angleHeadYaw))
            angleHeadYaw += increment
        while angleHeadYaw > -119.5:
            angleLists.append(math.radians(angleHeadYaw))
            angleHeadYaw -= increment
        while angleHeadYaw <= 0.0:
            angleLists.append(math.radians(angleHeadYaw))
            angleHeadYaw += increment
        return angleLists

    def arrDegreeToRad(self,angles:list):
        res = []
        for angle in angles:
            res.append(math.radians(angle))
        return res

    def animate(self):

        # Example showing multiple trajectories
        names      = ["HeadYaw", "HeadPitch"]
        angleHeadPitch = math.radians(-35.0)
        angleHeadYaw = 0.0
        # yawAngleLists = self.setYawPose(angleHeadYaw,50.0)
        yawAngleLists = self.arrDegreeToRad([-90.0,0.0,90.0,0.0])
        pitchAngleLists = [angleHeadPitch]
        angleLists = [yawAngleLists,pitchAngleLists]

        # set times for joints every seconds
        yawTimeLists = [1.0,5.0,10.0,15.0]
        pitchTimeLists = [1.0]
        timeLists = [yawTimeLists,pitchTimeLists]
        isAbsolute = True
        print(angleLists)
        print(timeLists)
        self.motion_service.angleInterpolation(names, angleLists, timeLists, isAbsolute)

        # self.motion_service.wakeUp()




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

