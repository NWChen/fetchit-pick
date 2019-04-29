import rospy
import sys, select, termios, tty
from fetch_api import Base, Torso, Arm, Gripper, Head
from math import pi

rospy.init_node('teleop')
while not rospy.Time.now():
    pass
print 'READY'

torso_height = 0.4
head_pan, head_tilt = 0, 0

def move_torso(distance):
    global torso_height
    torso.set_height(torso_height + distance)
    torso_height += distance


def move_head(pan, tilt):
    global head_pan, head_tilt
    head_pan += pan
    head_tilt += tilt
    head.pan_tilt(pan, tilt)

if __name__ == '__main__':
    base, torso, arm, gripper, head = Base(), Torso(), Arm(), Gripper(), Head()

    bindings = {
        # Base
        'w': ('Base', lambda distance: base.go_forward(distance)),
        's': ('Base', lambda distance: base.go_forward(-distance)),
        'a': ('Base', lambda angle: base.turn(angle)),
        'd': ('Base', lambda angle: base.turn(-angle)),

        # Torso
        't': ('Torso', lambda dist: move_torso(dist)),
        'g': ('Torso', lambda dist: move_torso(-dist)),

        # Head
        'q': ('Head', lambda pan: move_head(-pan, 0)),
        'e': ('Head', lambda pan: move_head(pan, 0)),
        'r': ('Head', lambda tilt: move_head(0, -tilt)),
        'f': ('Head', lambda tilt: move_head(0, tilt)),
        
    }

    settings = termios.tcgetattr(sys.stdin)
    x, break_key = '', '\x03'

    while (x != break_key):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        x = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

        print x
        if x in bindings.keys():
            (label, cmd) = bindings[x]
            print 'KEY: ', x
            print '%s: %s' % (x, str(label))
            print cmd
            cmd(0.1)
        else:
            print('Invalid key entered.')

    print 'Stopping.'
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
