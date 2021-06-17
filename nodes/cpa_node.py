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


import rospy
import tf2_ros
import project11
import numpy as np
from math import pi
from cpa.vessel import Vessel
from nav_msgs.msg import Odometry
from marine_msgs.msg import Contact
from tf.transformations import euler_from_quaternion
from tf2_geometry_msgs import do_transform_pose, PoseStamped

class CpaNode():
    def __init__(self):
        rospy.init_node('cpa')
        rospy.Subscriber('/ben/odom', Odometry, self.odom_callback, queue_size=10)
        rospy.Subscriber('/ben/sensors/ais/contact', Contact, self.contact_callback, queue_size=10)

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(tfBuffer)

        # odom_callbackup will update this
        self.ben = None

        # TODO
        # rospy.Publisher('/ben/cpa')

    def convert_latlon_to_map(self, lat, lon, time):
        '''
        Convert from latitude and longitude to vehicle's base_link reference frame.

        Intended to be used both for the vehicle's own position information and also that of
        AIS Contacts.
        '''
        
        # Convert reference frames from latitude/longitude to earth-centered, earth-fixed
        ecef_x, ecef_y, ecef_z = project11.wgs84.toECEFfromDegrees(lat, lon)
        
        # QUESTION: What exactly does this do?
        Pbase = PoseStamped()
        Pbase.pose.position.x = ecef_x
        Pbase.pose.position.y = ecef_y
        Pbase.pose.position.z = ecef_z

        # Query conversion type from ECEF to base_link
        # TODO: convert hardcoded /ben/map to param from server, to generalize across vehicles
        try:
            ecef_to_base = self.tfBuffer.lookup_transform("ben/map", 'earth', time)
        except Exception as e:
            return




    def odom_callback(self, odom_msg):
        '''
        Update instance state.
        '''
        rospy.loginfo(rospy.get_caller_id() + "O")#I heard Odometry message %s", data)
        o = odom_msg.pose.pose.orientation
        _roll, _pitch, cog_rad = euler_from_quaternion([o.x, o.y, o.z, o.w])
        cog_deg = (cog_rad / (2 * pi))* 360.0
        self.ben = Vessel(
            # As of 2021-06-13, the Vessel class does not use length for anything
            length = None,
            x = odom_msg.pose.pose.position.x,
            y = odom_msg.pose.pose.position.y,
            # Convert from Vector3 to scalar
            speed = np.linalg.norm(odom_msg.twist.twist.linear),
            heading = cog_deg,
        )

    def contact_callback(self, contact):
        rospy.loginfo(rospy.get_caller_id() + "I found Contact source %s", contact)
        loa = contact.dimension_to_bow + contact.dimension_to_stern
        lat = contact.position.latitude
        lon = contact.position.longitude
        sog = contact.sog
        cog = contact.cog
        other_vessel = Vessel(
            length = loa,
            x = lon,
            y = lat,
            speed = sog,
            heading = cog,
        )
        cpa = self.ben.cpa(other_vessel)
        # TODO
        # publish CPA!
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
