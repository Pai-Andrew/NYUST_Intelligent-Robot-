#!/usr/bin/env python

# 導入必要的 ROS 和消息類別
import rospy
from std_msgs.msg import String

# 初始化節點
rospy.init_node('python_hello_node')

# 主迴圈：只要節點未被關閉，就會持續執行
while not rospy.is_shutdown():
    # 打印訊息到 ROS 日誌
    rospy.loginfo('Hello World')
    
    # 休眠 1 秒，以避免訊息過於頻繁
    rospy.sleep(1)

