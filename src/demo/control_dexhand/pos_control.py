#!/usr/bin/env python3
import rospy
from kuavo_msgs.msg import dexhandCommand
import time
import signal
import sys

def signal_handler(sig, frame):
    print("\nCtrl+C detected, exiting...")
    sys.exit(0)

def main():
    signal.signal(signal.SIGINT, signal_handler)
    
    rospy.init_node('dexhand_test')
    
    # Create publishers for each topic
    dual_pub = rospy.Publisher('/dexhand/command', dexhandCommand, queue_size=10)
    left_pub = rospy.Publisher('/dexhand/left/command', dexhandCommand, queue_size=10)
    right_pub = rospy.Publisher('/dexhand/right/command', dexhandCommand, queue_size=10)

    # Wait for publishers to be ready
    rospy.sleep(1)

    # Create command message
    cmd = dexhandCommand()
    cmd.control_mode = dexhandCommand.POSITION_CONTROL
    cmd.data = [0, 0, 0, 0, 0, 0]  # Open position

    # Test sequence
    while not rospy.is_shutdown():
        try:
            # Open position
            cmd.data = [0]*12
            
            print("Publishing open position to all hands")
            dual_pub.publish(cmd)
            rospy.sleep(2)

            # Close position 
            cmd.data = [50, 100, 100, 100, 100, 100]
            
            print("Publishing close position to left hand")
            left_pub.publish(cmd)
            rospy.sleep(2)

            print("Publishing close position to right hand") 
            right_pub.publish(cmd)
            rospy.sleep(2)

        except KeyboardInterrupt:
            print("\nCtrl+C detected, exiting...")
            sys.exit(0)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
