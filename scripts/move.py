import qi
import sys


class Motion():
    def __init__(self, app):
        app.start()
        session = app.session
        self.animation_player_service = session.service("ALAnimationPlayer")

    def animate(self,name:str):
        future = self.animation_player_service.run(name, _async=True)
        # wait the end of the animation
        future.value()



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
    motion.animate("animations/Stand/Gestures/Explain_10")
    # motion.animate("animations/Stand/BodyTalk/BodyTalk_4")

