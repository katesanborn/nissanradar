#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PointStamped
import curses
import threading

# Coordinate ranges
LAT_MIN, LAT_MAX = -8192, 8192
LONG_MIN, LONG_MAX = -4096, 4096

# Topics to subscribe
topics = [
    'car/radar/track_L1_1', 'car/radar/track_L1_2', 'car/radar/track_L1_3',
    'car/radar/track_L1_4', 'car/radar/track_L1_5', 'car/radar/track_L1_6',
    'car/radar/track_R1_1', 'car/radar/track_R1_2', 'car/radar/track_R1_3',
    'car/radar/track_R1_4', 'car/radar/track_R1_5', 'car/radar/track_R1_6',
    
    'car/radar/track_L2_1', 'car/radar/track_L2_2', 'car/radar/track_L2_3',
    'car/radar/track_L2_4', 'car/radar/track_L2_5', 'car/radar/track_L2_6',
    'car/radar/track_R2_1', 'car/radar/track_R2_2', 'car/radar/track_R2_3',
    'car/radar/track_R2_4', 'car/radar/track_R2_5', 'car/radar/track_R2_6',
    
    'car/radar/track_L3_1', 'car/radar/track_L3_2', 'car/radar/track_L3_3',
    'car/radar/track_L3_4', 'car/radar/track_L3_5', 'car/radar/track_L3_6',
    'car/radar/track_R3_1', 'car/radar/track_R3_2', 'car/radar/track_R3_3',
    'car/radar/track_R3_4', 'car/radar/track_R3_5', 'car/radar/track_R3_6',
    
    'car/radar/track_L4_1', 'car/radar/track_L4_2', 'car/radar/track_L4_3',
    'car/radar/track_L4_4', 'car/radar/track_L4_5', 'car/radar/track_L4_6',
    'car/radar/track_R4_1', 'car/radar/track_R4_2', 'car/radar/track_R4_3',
    'car/radar/track_R4_4', 'car/radar/track_R4_5', 'car/radar/track_R4_6',
    
    'car/radar/track_L5_1', 'car/radar/track_L5_2', 'car/radar/track_L5_3',
    'car/radar/track_L5_4', 'car/radar/track_L5_5', 'car/radar/track_L5_6',
    'car/radar/track_R5_1', 'car/radar/track_R5_2', 'car/radar/track_R5_3',
    'car/radar/track_R5_4', 'car/radar/track_R5_5', 'car/radar/track_R5_6',
    
    'car/radar/track_L6_1', 'car/radar/track_L6_2', 'car/radar/track_L6_3',
    'car/radar/track_L6_4', 'car/radar/track_L6_5', 'car/radar/track_L6_6',
    'car/radar/track_R6_1', 'car/radar/track_R6_2', 'car/radar/track_R6_3',
    'car/radar/track_R6_4', 'car/radar/track_R6_5', 'car/radar/track_R6_6',
    
    'car/radar/track_L7_1', 'car/radar/track_L7_2', 'car/radar/track_L7_3',
    'car/radar/track_L7_4', 'car/radar/track_L7_5', 'car/radar/track_L7_6',
    'car/radar/track_R7_1', 'car/radar/track_R7_2', 'car/radar/track_R7_3',
    'car/radar/track_R7_4', 'car/radar/track_R7_5', 'car/radar/track_R7_6',
    
    'car/radar/track_L8_1', 'car/radar/track_L8_2', 'car/radar/track_L8_3',
    'car/radar/track_L8_4', 'car/radar/track_L8_5', 'car/radar/track_L8_6',
    'car/radar/track_R8_1', 'car/radar/track_R8_2', 'car/radar/track_R8_3',
    'car/radar/track_R8_4', 'car/radar/track_R8_5', 'car/radar/track_R8_6',
    
    'car/radar/track_L9_1', 'car/radar/track_L9_2', 'car/radar/track_L9_3',
    'car/radar/track_L9_4', 'car/radar/track_L9_5', 'car/radar/track_L9_6',
    'car/radar/track_R9_1', 'car/radar/track_R9_2', 'car/radar/track_R9_3',
    'car/radar/track_R9_4', 'car/radar/track_R9_5', 'car/radar/track_R9_6',
    
    'car/radar/track_L10_1', 'car/radar/track_L10_2', 'car/radar/track_L10_3',
    'car/radar/track_L10_4', 'car/radar/track_L10_5', 'car/radar/track_L10_6',
    'car/radar/track_R10_1', 'car/radar/track_R10_2', 'car/radar/track_R10_3',
    'car/radar/track_R10_4', 'car/radar/track_R10_5', 'car/radar/track_R10_6',
]

track_points = {}
lock = threading.Lock()

def scale(value, min_src, max_src, min_dst, max_dst):
    value = max(min_src, min(max_src, value))
    return int((value - min_src) / (max_src - min_src) * (max_dst - min_dst)) + min_dst

def make_callback(topic):
    parts = topic.split('/')
    last_part = parts[-1]
    label_parts = last_part.split('_')
    label = label_parts[1] + '.' + label_parts[2]

    def callback(msg):
        lat = msg.point.y
        long = msg.point.x
        if 'L' in topic:
            lat = -lat
        with lock:
            track_points[label] = (lat, long)
    return callback

def ros_listener():
    for topic in topics:
        rospy.Subscriber(topic, PointStamped, make_callback(topic))
    rospy.spin()

def curses_display(stdscr):
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(100)

    curses.start_color()
    curses.use_default_colors()

    # Color definitions
    curses.init_pair(1, curses.COLOR_RED, -1)    # Car
    curses.init_pair(2, curses.COLOR_GREEN, -1)  # Left radar
    curses.init_pair(3, curses.COLOR_BLUE, -1)   # Right radar

    while not rospy.is_shutdown():
        stdscr.erase()
        height, width = stdscr.getmaxyx()
        stdscr.border()

        center_x = width // 2
        center_y = height // 2

        try:
            stdscr.attron(curses.color_pair(1))
            stdscr.addch(center_y, center_x, 'X')
            stdscr.attroff(curses.color_pair(1))
        except curses.error:
            pass

        with lock:
            for label, (lat, long) in track_points.items():
                gx = scale(lat, LAT_MIN, LAT_MAX, 0, width - 3)
                gy = scale(long, LONG_MIN, LONG_MAX, 0, height - 3)
                x = gx + 1
                y = gy + 1

                if (x, y) == (center_x, center_y):
                    continue

                try:
                    color = curses.color_pair(2 if label.startswith('L') else 3)
                    stdscr.attron(color)
                    stdscr.addstr(y, x, "+")
                    stdscr.attroff(color)
                except curses.error:
                    pass

        stdscr.refresh()

if __name__ == '__main__':
    rospy.init_node('curses_grid_viewer', anonymous=True)

    ros_thread = threading.Thread(target=ros_listener)
    ros_thread.daemon = True
    ros_thread.start()

    curses.wrapper(curses_display)