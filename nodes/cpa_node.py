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
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf2_geometry_msgs import do_transform_pose, PoseStamped

class CpaNode():
    def __init__(self):
        rospy.init_node('cpa')
        rospy.Subscriber('/ben/odom', Odometry, self.odom_callback, queue_size=10)
        rospy.Subscriber('/ben/sensors/ais/contact', Contact, self.contact_callback, queue_size=10)

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        # odom_callbackup will update this
        self.us_odom = None

        # TODO
        # rospy.Publisher('/ben/cpa')

    def odom_callback(self, odom_msg):
        '''
        Save BEN's Odometry message to shared state for future access by contact_callback.
        '''
        rospy.loginfo(rospy.get_caller_id() + " odom_callback starting")
        
        self.odom = odom_msg

    def odom_to_vessel(self, odom_msg):

        o = odom_msg.pose.pose.orientation

        _roll_rad, _pitch_rad, cog_rad = euler_from_quaternion(
            [
                o.x,
                o.y,
                o.z,
                o.w,
            ]
        )
        cog_deg = (cog_rad / (2 * pi)) * 360.0
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        speed = np.linalg.norm(odom_msg.twist.twist.linear)

        v = Vessel(
            # As of 2021-06-13, the Vessel class does not use length for anything
            length = None,
            x = x,
            y = y,
            speed = speed,
            # COG is less noisy, and more appropriate for CPA, than heading
            heading = cog_deg,
        )

        return v

    def contact_callback(self, contact):
        '''
        Main workhorse.

        Convert's AIS Contact's position from 
        '''
        rospy.loginfo(rospy.get_caller_id() + f" contact_callback starting. COG = {contact.cog}, SOG = {contact.sog}.")

        # Abort if we don't yet know our own position
        if self.odom is None:
            rospy.loginfo(rospy.get_caller_id() + " Own position unknown.")
            return

        # I. Convert AIS Contact to same reference frame as our vessel

        # Convert Contact's position's reference frame from AIS's latitude/longitude to earth-centered, earth-fixed (ECEF)
        ecef_x, ecef_y, ecef_z = project11.wgs84.toECEFfromDegrees(
            contact.position.latitude,
            contact.position.longitude,
        )
        
        # Combine converted position with course over ground
        # TODO: does course over ground itself need to be converted? Currently using raw.
        Pbase = PoseStamped()
        Pbase.pose.position.x = ecef_x
        Pbase.pose.position.y = ecef_y
        Pbase.pose.position.z = ecef_z
        Pbase.pose.orientation = quaternion_from_euler(
            # Roll and pitch are irrelevant for CPA (and unavailable from Contact)
            0,
            0,
            contact.cog,
        )

        # Query conversion type from ECEF ('earth') to base_link
        try:
            ecef_to_odom = self.tfBuffer.lookup_transform(
                self.odom.header.frame_id,
                'earth',
                self.odom.header.stamp,
            )
        except Exception as e:
            rospy.loginfo(rospy.get_caller_id() + " Failed to query transform.")
            return
        
        # Convert other vessel's pose from ECEF to the reference frame used by our vessel's Odometry message
        odom = do_transform_pose(Pbase, ecef_to_odom)

        # Set other vessel's speed over ground
        # TODO: does Contact's sog (m/s) need to be converted? Currently using raw. Odometry is in meters...
        # ...So is Odometry's twist.twist.linear in meters, also?
        # TODO: Is this the correct way to set speed over ground? Does do_transform_pose(Pbase, ecef_to_odom) create
        # a normalized (magnitude 1) twist.twist.linear Vector3 in the cog direction (so that I can just scale it by sog)?
        rospy.loginfo(rospy.get_caller_id() + f" odom.twist.twist.linear after transform: {odom.twist.twist.linear}")
        odom.twist.twist.linear = odom.twist.twist.linear * contact.sog
        
        # Set speed over ground (Contact.sog and Odometry is m/s)

        # II. Calculate CPA

        # Our vessel
        us = self.odom_to_vessel(self.odom)
        them = self.odom_to_vessel(odom)

        # CPA
        (
            time_at_cpa,
            their_position_at_cpa,
            our_position_at_cpa,
            range_to_them_at_cpa,
            bearing_to_them_at_cpa,
        ) = us.cpa(them)

        # III. Publish CPA
        # TODO

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
