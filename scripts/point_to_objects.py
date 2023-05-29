import time
import qi
import sys
import math
import rospy
from math import degrees, acos
import math

#import ros msg and srv
from robobreizh_msgs.srv import PointToObject
from manipulation_pepper.srv import EmptySrvResponse

class PointToObjectMotion():
    def __init__(self, app):
        app.start()
        session = app.session
        self.motion_service = session.service("ALMotion")
        self.motion_service.setStiffnesses("Head", 1.0)
        rospy.init_node('pointObjectPosition')
        rospy.Service('/robobreizh/manipulation/pointObjectPosition', PointToObject, self.animate)
        rospy.spin()
        
    def arrDegreeToRad(self,angles:list):
        res = []
        for angle in angles:
            res.append(math.radians(angle))
        return res

    def animate(self,PointToObject):
        
        distance = PointToObject.distance
        
        point_x = PointToObject.point_x
        point_y = PointToObject.point_y
        point_z = PointToObject.point_z

        cos = math.acos(point_z/distance)
        
        rospy.loginfo("cos-1 - Chair :" + str(cos))

        angle = degrees(cos)
        
        rospy.loginfo("Distance - Chair :" + str(distance))
        rospy.loginfo("Point z - Chair :" + str(point_z))
        rospy.loginfo("Angle - Chair :" + str(angle))
                
        # Example showing multiple trajectories
        names = ["RShoulderPitch","RShoulderRoll","RElbowYaw","RElbowRoll","RWristYaw"]

        RShoulderPitchAngleLists = self.arrDegreeToRad([0.0]) # -119.5 / 119.5
        RShoulderRollAngleLists = self.arrDegreeToRad([float(angle*-1)]) # -89.5 / -0.5
        RElbowYawAngleLists = self.arrDegreeToRad([0.0]) # -119.5 / 119.5
        RElbowRollAngleLists = self.arrDegreeToRad([0.0]) # 0.5 / -89.5
        RWristYawAngleLists = self.arrDegreeToRad([90.0]) # -104.5 / 104.5

        angleLists = [RShoulderPitchAngleLists,RShoulderRollAngleLists,RElbowYawAngleLists,RElbowRollAngleLists,RWristYawAngleLists]

        # set times for joints every seconds
        RShoulderPitchTimeLists = [5.0]
        RShoulderRollTimeLists = [3.0]
        RElbowYawTimeLists = [1.0]
        RElbowRollTimeLists =[1.0]
        RWristYawTimeLists = [1.0]
        yawTimeLists = [1.0]
        pitchTimeLists = [1.0]
        timeLists = [RShoulderPitchTimeLists,RShoulderRollTimeLists,RElbowYawTimeLists,RElbowRollTimeLists,RWristYawTimeLists]

        isAbsolute = True
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
    motion = PointToObjectMotion(app)

