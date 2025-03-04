#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped

def navigate_to_room(room_id):
    # 定義房間的目標點（根據地圖坐標）
    room_goals = {
        1: {"x": -6.31, "y": 0.00, "z": 0.0, "qw": 1.0},
        2: {"x": -6.11, "y": 3.09, "z": -0.78, "qw": 0.62},
        3: {"x": -1.87, "y": 0.88, "z": 0.99, "qw": 0.40},
        4: {"x": 1.21, "y": 4.00, "z": -0.10, "qw": 0.99},
        5: {"x": 4.60, "y": 1.53, "z": 0.05, "qw": 0.99},
        6: {"x": 5.68, "y": -1.40, "z": -0.09, "qw": 0.99},
    }

    # 獲取目標房間的位置
    goal = room_goals.get(room_id, None)
    if not goal:
        rospy.logwarn("無效的房間編號，請重新輸入！")
        return

    # 發送導航目標
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.sleep(1)  # 確保 Publisher 準備就緒
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = goal["x"]
    pose.pose.position.y = goal["y"]
    pose.pose.orientation.z = goal["z"]
    pose.pose.orientation.w = goal["qw"]
    pub.publish(pose)
    rospy.loginfo(f"導航到房間 {room_id}：x={goal['x']}, y={goal['y']}")

if __name__ == "__main__":
    rospy.init_node('room_navigation')
    while not rospy.is_shutdown():
        try:
            room_id = int(input("輸入房間序號（1-6）："))
            navigate_to_room(room_id)
        except ValueError:
            rospy.logwarn("請輸入有效的數字！")

