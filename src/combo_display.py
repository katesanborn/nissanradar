#!/usr/bin/env python3
import rospy
import curses
from geometry_msgs.msg import PointStamped

tracks_data = {}

def track_callback(msg, track_name):
    # Handle both normal and mystery tracks
    if track_name.endswith("_mystery"):
        base_name = track_name.replace("_mystery", "")
        x, y = msg.point.x, msg.point.y
        # Store mystery values under separate key
        data = tracks_data.get(base_name, {})
        data["mystery"] = (x, y)
        tracks_data[base_name] = data
    else:
        x, y, z = msg.point.x, msg.point.y, msg.point.z
        data = tracks_data.get(track_name, {})
        data["track"] = (x, y, z)
        tracks_data[track_name] = data

def draw_box(stdscr, top, left, height, width, title=""):
    if height < 3 or width < 4:
        return
    stdscr.addstr(top, left, "+" + "-"*(width-2) + "+")
    stdscr.addstr(top + height - 1, left, "+" + "-"*(width-2) + "+")
    for y in range(top+1, top + height -1):
        stdscr.addstr(y, left, "|")
        stdscr.addstr(y, left + width -1, "|")
    if title:
        title_str = f"[ {title} ]"
        max_title_len = width - 4
        stdscr.addstr(top, left + 2, title_str[:max_title_len])

def curses_main(stdscr):
    curses.curs_set(0)
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_GREEN, -1)
    GREEN_PAIR = curses.color_pair(1)

    stdscr.nodelay(True)
    stdscr.timeout(200)
    height, width = stdscr.getmaxyx()

    while not rospy.is_shutdown():
        if curses.is_term_resized(height, width):
            height, width = stdscr.getmaxyx()
            curses.resize_term(height, width)
            stdscr.erase()

        stdscr.erase()
        height, width = stdscr.getmaxyx()

        mid_x = width // 2
        stdscr.addstr(0, 2, "L: Left Radar Tracks")
        stdscr.addstr(0, mid_x + 2, "R: Right Radar Tracks")

        box_height = 8
        spacing = 1
        total_box_height = box_height + spacing
        max_vertical = (height - 1) // total_box_height
        need_columns = 10 > max_vertical
        column_count = 2 if need_columns else 1
        side_width = (mid_x - 4)
        col_width = side_width // column_count

        def draw_group_boxes(groups, side_base_x, max_side_x):
            for i, (group_name, lines) in enumerate(groups):
                col = i // max_vertical if need_columns else 0
                row = i % max_vertical
                top = 1 + row * total_box_height
                left = side_base_x + col * col_width
                if top + box_height > height or left + col_width > max_side_x:
                    continue
                min_box_w = max(len(f"[ {group_name} ]") + 4, 30)
                box_w = max(min_box_w, min(max(len(line_text) for _, line_text, _ in lines) + 4, col_width - 2))
                draw_box(stdscr, top, left, box_height, box_w, title=group_name)
                for j, (track_name, line_text, is_valid) in enumerate(lines):
                    attr = GREEN_PAIR if is_valid else curses.A_NORMAL
                    stdscr.addstr(top + 1 + j, left + 2, line_text[:box_w - 4], attr)

        left_groups = []
        right_groups = []
        for track_num in range(1, 11):
            left_lines = []
            right_lines = []
            for subtrack in range(1, 7):
                for side, lines in [("L", left_lines), ("R", right_lines)]:
                    base_name = f"track_{side}{track_num}_{subtrack}"
                    display_name = f"{side}{track_num}_{subtrack}"
                    data = tracks_data.get(base_name, {})
                    # Normal track values
                    if "track" in data:
                        x, y, z = data["track"]
                    else:
                        x, y, z = 0, 0, 0
                    # Mystery values
                    if "mystery" in data:
                        a, b = data["mystery"]
                    else:
                        a, b = 0, 0
                    line_text = (
                        f"{display_name}: DB={x:5.0f}, DA={y:4.0f}, F={z:1.0f} "
                        f"?A={a:3.0f}, ?B={b:5.0f}"
                    )
                    lines.append((base_name, line_text, int(z) == 1))
            left_groups.append((f"L{track_num}", left_lines))
            right_groups.append((f"R{track_num}", right_lines))

        draw_group_boxes(left_groups, 2, mid_x - 2)
        draw_group_boxes(right_groups, mid_x + 2, width - 2)

        stdscr.refresh()
        try:
            key = stdscr.getch()
            if key == ord('q'):
                break
        except curses.error:
            pass

def main():
    rospy.init_node('radar_display_node')

    # Subscribe to both normal and mystery tracks
    track_topics = []
    for side in ["L", "R"]:
        for num in range(1, 11):
            for sub in range(1, 7):
                track_topics.append(f"car/radar/track_{side}{num}_{sub}")
                track_topics.append(f"car/radar/track_{side}{num}_{sub}_mystery")

    for topic in track_topics:
        rospy.Subscriber(topic, PointStamped, track_callback, callback_args=topic.split('/')[-1])

    curses.wrapper(curses_main)

if __name__ == '__main__':
    main()
