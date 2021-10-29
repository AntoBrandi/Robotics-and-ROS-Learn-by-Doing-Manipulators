#!/usr/bin/env python
import rospy
from std_msgs.msg import String

if __name__ == '__main__':
    try:
        # Inizialize a ROS node called talker
        rospy.init_node('talker', anonymous=True)

        # register a publisher on the topic /chatter that will publish String messages
        pub = rospy.Publisher('chatter', String, queue_size=10)
        
        # Define the frequency for publishing the messages
        # The rate is expressed in Hertz
        rate = rospy.Rate(10) # 10hz
        count = 0

        # Keep going publishing messages until the ROS communication is alive
        while not rospy.is_shutdown():
            hello_str = "hello world %d" % count
            rospy.loginfo(hello_str)
            pub.publish(hello_str)
            count += 1
            # wait the desired rate before publishing the next message
            rate.sleep()
    except rospy.ROSInterruptException:
        pass