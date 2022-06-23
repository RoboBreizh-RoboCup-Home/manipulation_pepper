import rospkg
import math
import numpy as np

#########################################################################################
# TERMINAL color
#########################################################################################
W = '\033[0m'  # white (normal)
R = '\033[31m'  # red
G = '\033[32m'  # green
O = '\033[33m'  # orange
B = '\033[34m'  # blue
P = '\033[35m'  # purple
CYAN = '\033[96m'

def get_pkg_path():
    rp = rospkg.RosPack()
    return(rp.get_path('manipulation_pepper'))
