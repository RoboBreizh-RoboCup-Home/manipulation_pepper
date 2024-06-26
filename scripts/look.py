import time
import qi
import sys
import numpy as np
import rospy
from robobreizh_msgs.srv import head_position, head_positionResponse


class Motion():
    def __init__(self, app):
        app.start()
        session = app.session
        self.motion_service = session.service("ALMotion")
        rospy.init_node('look_up')
        rospy.Service('/robobreizh/manipulation/look', head_position, self.animate)
        rospy.spin()

    def animate(self, req):
        # Example showing multiple trajectories
        names = ["HeadPitch", "HeadYaw"]
        rad_head_pitch = np.deg2rad(req.head_pitch_angle).tolist()
        rad_head_yaw = np.deg2rad(req.head_yaw_angle).tolist()
        angleLists = [rad_head_pitch, rad_head_yaw]

        # set times for joints every seconds
        timeLists = [req.head_pitch_time, req.head_yaw_time]
        isAbsolute = True
        self.motion_service.angleInterpolation(names, angleLists, timeLists, isAbsolute)
        return (head_positionResponse())


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
