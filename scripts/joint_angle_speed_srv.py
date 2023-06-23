import qi
import sys
import numpy as np
import rospy
from robobreizh_msgs.srv import joint_position, joint_positionResponse


class Motion():
    def __init__(self, app):
        app.start()
        session = app.session
        self.motion_service = session.service("ALMotion")
        rospy.init_node('look_up')
        rospy.Service('/robobreizh/manipulation/joint_angle_speed_srv', joint_position, self.animate)
        rospy.spin()

    def animate(self, req):
        # Example showing multiple trajectories
        names = req.joint_names
        angle_list = []
        for angle_list_req in req.angle_lists.row:
                angle_list.append(np.deg2rad(angle_list_req.col).tolist())

        time_list = []
        for time_list_req in req.time_lists.row:
                time_list.append(time_list_req.col)


        # set times for joints every seconds
        isAbsolute = True
        print(names)
        print(angle_list)
        print(time_list)
        self.motion_service.angleInterpolation(names,angle_list ,time_list , isAbsolute)
        return (joint_positionResponse())


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
