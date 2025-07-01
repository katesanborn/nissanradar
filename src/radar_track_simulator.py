#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PointStamped
import random
import time

class RadarTrackSimulator:
    def __init__(self):
        rospy.init_node('radar_track_simulator', anonymous=True)

        self.publishers = {}

        for side in ['L', 'R']:
            for ring in range(1, 11):
                for track in range(1, 7):
                    topic = f"/car/radar/track_{side}{ring}_{track}"
                    self.publishers[topic] = rospy.Publisher(topic, PointStamped, queue_size=10)

        self.rate = rospy.Rate(10)  # 10 Hz
        self.run_simulation()

    def run_simulation(self):
        count = 0
        while not rospy.is_shutdown():
            for topic, pub in self.publishers.items():
                point_msg = PointStamped()
                point_msg.header.stamp = rospy.Time.now()
                # point_msg.point.x = random.uniform(-10.0, 10.0)
                # point_msg.point.y = random.uniform(-5.0, 5.0)
                # point_msg.point.z = 0.0  # Could also randomize if desired
                point_msg.point.x = count
                point_msg.point.y = count
                point_msg.point.z = count
                count += 1

                pub.publish(point_msg)

            self.rate.sleep()

if __name__ == '__main__':
    try:
        RadarTrackSimulator()
    except rospy.ROSInterruptException:
        pass