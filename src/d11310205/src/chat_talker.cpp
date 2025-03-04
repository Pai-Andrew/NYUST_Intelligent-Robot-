#include "ros/ros.h"            // 引入 ROS 的核心標頭文件，包含 ROS 的基本功能
#include "std_msgs/String.h"     // 引入 ROS 的標準字串消息類型
#include <sstream>               // 引入 C++ 的 stringstream 類，用於格式化字串

int main(int argc, char **argv) 
{
    // 初始化 ROS 節點，"talker" 是節點名稱
    ros::init(argc, argv, "chat_talker");

    // 創建一個 NodeHandle 對象，用於與 ROS 系統進行通訊
    ros::NodeHandle n;

    // 創建一個 Publisher 物件，發布 std_msgs::String 類型的消息到 "chatter" 話題，隊列大小為 1000
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

    // 設置循環頻率為 10Hz
    ros::Rate loop_rate(1);

    int count = 0;  // 初始化計數器變數

    // 當 ROS 系統正常運行時，進入循環
    while (ros::ok()) 
    {
        // 創建一個 std_msgs::String 類型的消息
        std_msgs::String msg;

        // 使用 stringstream 格式化字串
        std::stringstream ss;
        ss << "Hello,ROS! count " << count;

        // 將格式化的字串賦值給 msg 的 data 成員
        msg.data = ss.str();

        // 打印消息到 ROS 日誌
        ROS_INFO("%s", msg.data.c_str());

        // 將消息發布到 "chatter" 話題
        chatter_pub.publish(msg);

        // 處理一次回調函數（此範例中沒有訂閱者，所以此處沒有實際效果）
        ros::spinOnce();

        // 根據設置的頻率休眠，使循環每秒執行 10 次
        loop_rate.sleep();

        // 計數器增量，下一次會發布 "hello world" 字串加上新的計數值
        ++count;
    }

    return 0; // 結束程式
}

