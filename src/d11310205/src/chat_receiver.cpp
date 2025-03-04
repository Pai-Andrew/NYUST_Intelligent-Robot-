#include "ros/ros.h"              // 引入 ROS 的核心標頭文件，包含 ROS 的基本功能
#include "std_msgs/String.h"       // 引入 ROS 的標準字串消息類型

// 定義回調函數，當收到話題訊息時會觸發該函數
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    // 使用 ROS_INFO 打印接收到的訊息到 ROS 日誌
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
    // 初始化 ROS 節點，將節點名稱設置為 "listener"
    ros::init(argc, argv, "chat_receiver");

    // 創建一個 NodeHandle 物件，用來與 ROS 系統通訊
    ros::NodeHandle n;

    // 創建一個 Subscriber 物件，訂閱 "chatter" 話題，並設置隊列大小為 1000
    // 當接收到 "chatter" 話題的消息時，會呼叫 chatterCallback 函數處理
    ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

    // 進入循環，等待回調函數處理接收到的訊息
    ros::spin();

    return 0;
}

