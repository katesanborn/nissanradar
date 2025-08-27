#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PointStamped
import curses
import threading

# Coordinate ranges
LAT_MIN, LAT_MAX = -8192, 8192
LONG_MIN, LONG_MAX = 0, 512

# Generate topic list (normal + mystery)
topics = []
for side in ["L", "R"]:
    for num in range(1, 11):
        for sub in range(1, 7):
            topics.append(f'car/radar/track_{side}{num}_{sub}')
            topics.append(f'car/radar/track_{side}{num}_{sub}_mystery')

track_points = {}   # label -> (lat, long)
raw_data = {}       # store normal + mystery separately
lock = threading.Lock()

def scale(value, min_src, max_src, min_dst, max_dst):
    value = max(min_src, min(max_src, value))
    return int((value - min_src) / (max_src - min_src) * (max_dst - min_dst)) + min_dst

def make_callback(topic):
    parts = topic.split('/')
    last_part = parts[-1]
    label_parts = last_part.split('_')
    label = label_parts[1] + '.' + label_parts[2]

    is_mystery = topic.endswith("_mystery")

    def callback(msg):
        with lock:
            if label not in raw_data:
                raw_data[label] = {}

            if is_mystery:
                raw_data[label]["mystery_y"] = msg.point.y
            else:
                raw_data[label]["lat_y"] = msg.point.y
                raw_data[label]["lat_x"] = msg.point.x
                raw_data[label]["side"] = label_parts[1][0]  # L or R

            # If we have both normal and mystery, compute final lat/long
            if "lat_y" in raw_data[label] and "mystery_y" in raw_data[label]:
                lat = raw_data[label]["lat_y"]
                long = raw_data[label]["mystery_y"]

                if raw_data[label]["side"] == "L":
                    lat = -lat

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

    curses.init_pair(1, curses.COLOR_RED, -1)    # Car
    curses.init_pair(2, curses.COLOR_GREEN, -1)  # Left radar
    curses.init_pair(3, curses.COLOR_CYAN, -1)   # Right radar
    curses.init_pair(4, curses.COLOR_WHITE, -1)  # Center line

    while not rospy.is_shutdown():
        stdscr.erase()
        height, width = stdscr.getmaxyx()
        stdscr.border()

        center_x = width // 2
        center_y = height // 2

        # Dotted center line
        for y in range(1, height - 1, 2):
            try:
                stdscr.attron(curses.color_pair(4))
                stdscr.addch(y, center_x, '|')
                stdscr.attroff(curses.color_pair(4))
            except curses.error:
                pass

        # Car at center
        try:
            stdscr.attron(curses.color_pair(1))
            stdscr.addch(center_y, center_x, 'X')
            stdscr.attroff(curses.color_pair(1))
        except curses.error:
            pass

        # Track points
        with lock:
            for label, (lat, long) in track_points.items():
                gx = scale(lat, LAT_MIN, LAT_MAX, 0, width - 3)
                gy = scale(long, LONG_MIN, LONG_MAX, 0, height - 3)
                x = gx + 1
                y = gy + 1

                if (x == center_x) or ((x, y) == (center_x, center_y)):
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
