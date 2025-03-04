#include "ros/ros.h"  // 引入 ROS 的基本功能庫
#include "turtlesim/Pose.h"  // 引入 turtlesim 提供的 Pose 消息類型，用於獲取烏龜的位置
#include "turtlesim/TeleportAbsolute.h"  // 引入 TeleportAbsolute 服務，用於瞬間移動烏龜

// 設定畫布的邊界值
const float X_MIN = 0.5;
const float X_MAX = 10.5;
const float Y_MIN = 0.5;
const float Y_MAX = 10.5;

// 定義畫布的中心點座標
const float CENTER_X = 5.5;
const float CENTER_Y = 5.5;

// 宣告一個服務客戶端，用於呼叫 teleport_absolute 服務
ros::ServiceClient teleport_client;

// 位置回調函數，當烏龜的位置更新時會被呼叫
void poseCallback(const turtlesim::Pose::ConstPtr& msg) {
    // 檢查烏龜的位置是否超出邊界
    if (msg->x <= X_MIN || msg->x >= X_MAX || msg->y <= Y_MIN || msg->y >= Y_MAX) {
        ROS_INFO("撞到邊界！將烏龜移動到中心點。");

        // 創建服務請求物件，設定目標位置為中心點
        turtlesim::TeleportAbsolute srv;
        srv.request.x = CENTER_X;  // 設置 X 坐標為中心點
        srv.request.y = CENTER_Y;  // 設置 Y 坐標為中心點
        srv.request.theta = 0.0;   // 設置方向朝向右方 (0 弧度)

        // 呼叫服務來將烏龜移動到中心點
        if (teleport_client.call(srv)) {
            ROS_INFO("center");  // 成功呼叫服務並移動烏龜
        } else {
            ROS_ERROR("call error");  // 若呼叫服務失敗，顯示錯誤訊息
        }
    }
}

int main(int argc, char **argv) {
    // 初始化 ROS 節點，命名為 "turtle_boundary_check"
    ros::init(argc, argv, "turtle_boundary_check");

    // 創建一個節點處理器，用於與 ROS 系統進行互動
    ros::NodeHandle nh;

    // 訂閱 "/turtle1/pose" 話題，並將消息傳遞給 poseCallback 回調函數
    // 該話題會不斷更新烏龜的當前位置信息
    ros::Subscriber pose_sub = nh.subscribe("/turtle1/pose", 10, poseCallback);

    // 初始化服務客戶端，用於呼叫 "/turtle1/teleport_absolute" 服務
    // 該服務可以瞬間改變烏龜的位置和方向
    teleport_client = nh.serviceClient<turtlesim::TeleportAbsolute>("/turtle1/teleport_absolute");

    // 保持節點運行，等待回調函數執行
    ros::spin();

    return 0;
}

