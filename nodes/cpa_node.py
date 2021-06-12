#!/usr/bin/env python
'''
Stephen Wissow
Center for Coastal and Ocean Mapping 
University of New Hampshire
Copyright 2021

License: BSD - Clause 2
'''

import rospy
from . import Vessel
from nav_msgs.msg import Odometry
from marine_msgs.msg import Contact

def odom_callback(data):
    # STUB
    rospy.loginfo(rospy.get_caller_id() + "I heard Odometry message %s", data.data)

def contact_callback(data):
    # STUB
    rospy.loginfo(rospy.get_caller_id() + "I found Contact source %s", data.data)

# from ais_manager.cpp:
# void AISManager::scanForSources()
# {
#   ros::NodeHandle nh;

#   ros::master::V_TopicInfo topic_info;
#   ros::master::getTopics(topic_info);

#   for(const auto t: topic_info)
#     if (t.datatype == "marine_msgs/Contact")
#       if (m_sources.find(t.name) == m_sources.end())
#       {
#         m_sources[t.name] = nh.subscribe(t.name, 10, &AISManager::contactCallback, this);
#         m_ui->sourcesListWidget->addItem(t.name.c_str());
#       }
# }

def scan_for_sources():
    sources = dict()
    topics = rospy.get_published_topics()
    # topic: [name, type]
    for topic in topics:
        if topic[1] == "marine_msgs/Contact":
            sources[topic[0]] = rospy.Subscriber(topic[0], Contact, contact_callback, queue_size=10)

def cpa_listener():
    rospy.init_node('cpa')
    # rospy.Subscriber('/base/ais/contacts', Contact, contact_callback, queue_size=10)
    scan_for_sources()

    rospy.Subscriber('/ben/odom', Odometry, odom_callback, queue_size=10)
    # rospy.Publisher('/ben/cpa')

# modeled off ais_node.py:
if __name__ == '__main__':
    try:
        cpa_listener()
    except rospy.ROSInterruptException:
        pass
