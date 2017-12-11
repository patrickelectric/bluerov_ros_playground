import argparse
import bluerov_node as BR
import rospy

parser = argparse.ArgumentParser()
parser.add_argument("--device", "-a", help="Set the mavlink device: 'udp:192.168.2.2:14550', '/dev/ttyACM0'", type=str, default='udp:localhost:14550')
args = parser.parse_args()
if args.device:
    print("Set input adress to %s" % args.device)

try:
    rospy.init_node('user_node', log_level=rospy.DEBUG)
except rospy.ROSInterruptException as error:
    print('pubs error with ROS: ', error)
    exit(1)

bluerov = BR.BlueRov(device=args.device)

while not rospy.is_shutdown():
    bluerov.publish()