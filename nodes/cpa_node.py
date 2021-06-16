#!/usr/bin/env python
'''
Stephen Wissow
Center for Coastal and Ocean Mapping 
University of New Hampshire
Copyright 2021

License: BSD - Clause 2
'''

# Python3, please. Developed on Ubuntu 20.04 for ROS Noetic.
# This code is not thread safe. It is required that all callbacks
# in this node are executed in a single thread. This is supported
# by ROS's default behavior of executing all callbacks, across all
# nodes, within a single thread.

from math import pi
import rospy
from cpa.vessel import Vessel
from nav_msgs.msg import Odometry
from marine_msgs.msg import Contact
from tf.transformations import euler_from_quaternion

class CpaNode():
    def __init__(self):
        rospy.init_node('cpa')
        rospy.Subscriber('/ben/odom', Odometry, self.odom_callback, queue_size=10)
        rospy.Subscriber('/ben/sensors/ais/contact', Contact, self.contact_callback, queue_size=10)

        # odom_callbackup will update this
        self.ben = None

        # TODO
        # rospy.Publisher('/ben/cpa')

    def odom_callback(self, odom_msg):
        '''
        Update instance state.
        '''
        rospy.loginfo(rospy.get_caller_id() + "O")#I heard Odometry message %s", data)
        _roll, _pitch, cog_rad = euler_from_quaternion(odom_msg.pose.pose.orientation)
        cog_deg = (cog_rad / (2 * pi))* 360.0
        self.ben = Vessel(
            # As of 2021-06-13, the Vessel class does not use length for anything
            length = None,
            x = odom_msg.pose.pose.position.x,
            y = odom_msg.pose.pose.position.y,
            # Convert from Vector3 to scalar
            speed = odom_msg.twist.twist.linear.length(),
            heading = cog_deg,
        )

    def contact_callback(self, data):
        # STUB
        rospy.loginfo(rospy.get_caller_id() + "I found Contact source %s", data)
        # TODO
        # (Need to decide message type...maybe just a position? Or position with time?)

    # Not yet working; unused.
    def _scan_for_sources(self):
        '''
        Subscribe to all topics that publish Contact (AIS) messages.
        '''
        sources = dict()
        topics = rospy.get_published_topics()
        # topic: [name, type]
        for topic in topics:
            if topic[1] == "marine_msgs/Contact":
                rospy.loginfo(rospy.get_caller_id() + f'DEBUG: scan_for_sources found a source of marine_msgs/Contact called {topic[0]}')
                sources[topic[0]] = rospy.Subscriber(topic[0], Contact, self.contact_callback, queue_size=10)

    def run(self):
        # vessel = Vessel()

        while not rospy.is_shutdown():
            # scan_for_sources()
            rospy.loginfo(rospy.get_caller_id() + ".")
            rospy.sleep(1)

        # idea from ais_listener():
        # while not rospy.is_shutdown():...

        # idea from AsvSim.run():
        # rospy.Subscriber('/ben/odom', Odometry, odom_callback, queue_size=10)
        # rospy.Timer(rospy.Duration.from_sec(0.05), self.update)
        # clock_timer = threading.Timer(self.wallclock_time_step,self.update_clock)
        # clock_timer.start()
        # rospy.spin()

# modeled off ais_node.py:
if __name__ == '__main__':
    try:
        cpa_node = CpaNode()
        cpa_node.run()
    except rospy.ROSInterruptException:
        pass
