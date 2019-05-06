import rospy
from moveit_python import PlanningSceneInterface as Scene
from grasp_bolt import BoltController as C
from fetch_api import Arm, Head, Torso

c = C()
scene = Scene('/base_link')
TABLE_HEIGHT = 0.7
scene.addBox('table', 2.0, 1.0, TABLE_HEIGHT, 0.2, 0.0, TABLE_HEIGHT/2)
arm, head, torso = Arm(), Head(), Torso()

print('SETTING UP')
torso.set_height(0.4)
head.pan_tilt(0, 1)

target = c.transform((480/2, 640/2, 700, 0)
print(target)
if arm.check_pose(target) != None:
    print('VALID TARGET. MAKE SURE ARM ENVIRONMENT IS CLEAR!')

import time
for i in range(5):
    print('MOVING IN %d' % i+1)
    time.sleep(1)

arm.move_to_pose(target)
