import time
import qi
import sys
import math
import naoqi_bridge_msgs.msg._JointAnglesWithSpeed
import message_filters

class Motion():
    def __init__(self, app):
        app.start()
        session = app.session
        self.motion_service = session.service("ALMotion")
        self.posture = session.service("ALRobotPosture")
        self.motion_service.setStiffnesses("Head", 1.0)

    def animate(self):
        # Example showing a single target angle for one joint
        # Interpolates the head yaw to 1.0 radian in 1.0 second
        names      = "HeadYaw"
        angleLists = math.radians(50.0)
        timeLists  = 1.0
        isAbsolute = True
        self.motion_service.angleInterpolation(names, angleLists, timeLists, isAbsolute)

        time.sleep(1.0)

        # Example showing a single trajectory for one joint
        # Interpolates the head yaw to 1.0 radian and back to zero in 2.0 seconds
        names      = "HeadYaw"
        #              2 angles
        angleLists = [math.radians(30.0), 0.0]
        #              2 times
        timeLists  = [1.0, 2.0]
        isAbsolute = True
        self.motion_service.angleInterpolation(names, angleLists, timeLists, isAbsolute)

        time.sleep(1.0)

        # Example showing multiple trajectories
        names      = ["HeadYaw", "HeadPitch"]
        angleLists = [math.radians(30.0), math.radians(30.0)]
        timeLists  = [1.0, 1.2]
        isAbsolute = True
        self.motion_service.angleInterpolation(names, angleLists, timeLists, isAbsolute)

        # Example showing multiple trajectories
        # Interpolates the head yaw to 1.0 radian and back to zero in 2.0 seconds
        # while interpolating HeadPitch up and down over a longer period.
        names  = ["HeadYaw","HeadPitch"]
        # Each joint can have lists of different lengths, but the number of
        # angles and the number of times must be the same for each joint.
        # Here, the second joint ("HeadPitch") has three angles, and
        # three corresponding times.
        angleLists  = [[math.radians(50.0), 0.0],
                       [math.radians(-30.0), math.radians(30.0), 0.0]]
        timeLists   = [[1.0, 2.0], [ 1.0, 2.0, 3.0]]
        isAbsolute  = True
        self.motion_service.angleInterpolation(names, angleLists, timeLists, isAbsolute)

        self.posture.goToPosture("StandZero",1.0)


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

