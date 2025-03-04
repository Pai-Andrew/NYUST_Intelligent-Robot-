#include "ros/ros.h"  // 引入 ROS 的基本標頭文件，包含 ROS 的核心功能

// 主函數入口
int main(int argc, char *argv[]) 
{
    // 初始化 ROS 節點，argc 和 argv 是傳遞給節點的參數
    // "hello_world_node" 是這個節點的名稱，這樣 ROS 系統就能識別該節點
    ros::init(argc, argv, "hello_world_node");  

    // 創建一個 NodeHandle 節點處理器
    // NodeHandle 是與 ROS 系統交互的接口，負責管理節點的通訊，例如訂閱和發布話題等
    ros::NodeHandle handler;  

    // 使用 ROS_INFO 日誌輸出 "hello world"
    // ROS_INFO 是 ROS 提供的日誌功能，用於在控制台輸出訊息
    ROS_INFO("hello world");  

    return 0;  // 結束程式
}

