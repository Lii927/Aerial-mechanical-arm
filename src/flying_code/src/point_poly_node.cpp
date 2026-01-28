#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/PositionTarget.h>
#include <cmath>


// ==========================================
// 1. 全局配置区 (Configuration)
// ==========================================

// --- 飞行目标点 (动捕绝对坐标系) ---
// 你想让飞机飞到哪里，就在这里改
const double TARGET_X = 1.5; 
const double TARGET_Y = 0.0; 
const double TARGET_Z = 1.2; 

// --- 起飞参数 ---
const double TAKEOFF_HEIGHT = 1.0; // 起飞高度 (相对于地面)

// --- 时间参数 ---
const double TIME_TRAJ   = 10.0; // 轨迹飞行时长
const double TIME_HOVER  = 5.0;  // 到达后悬停时长 
const double TIME_WARMUP = 2.0;  // 起飞前预热时长

// ==========================================

// --- 状态枚举 ---
enum TaskState {
    PREPARE,    // 预热：发送位置流，尝试解锁 + Offboard
    TAKEOFF,    // 起飞：从当前位置垂直上升
    TRAJECTORY, // 轨迹：五次多项式规划飞行
    HOLD,       // 悬停：到达目标点后保持
    LAND,       // 降落：自动降落
    FINISHED    // 结束
};

// --- 全局变量 ---
mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
bool has_pose_received = false; 

// --- 回调函数 ---
void state_cb(const mavros_msgs::State::ConstPtr& msg){ current_state = *msg; }
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
    has_pose_received = true;
}

// --- 辅助函数：四元数转Yaw ---
double calculate_yaw(const geometry_msgs::Quaternion& q) {
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

// --- 五次多项式类  ---
class QuinticPoly {
public:
    double a0, a1, a2, a3, a4, a5;
    void compute(double start, double end, double T) {
        if (T <= 0) T = 0.1;
        double T3 = T*T*T, T4 = T3*T, T5 = T4*T;
        a0 = start; a1 = 0; a2 = 0;
        a3 = (20*(end-start)) / (2*T3);
        a4 = (30*(start-end)) / (2*T4);
        a5 = (12*(end-start)) / (2*T5);
    }
    double get_pos(double t) { return a0 + a1*t + a2*t*t + a3*t*t*t + a4*t*t*t*t + a5*t*t*t*t*t; }
    double get_vel(double t) { return a1 + 2*a2*t + 3*a3*t*t + 4*a4*t*t*t + 5*a5*t*t*t*t; }
    double get_acc(double t) { return 2*a2 + 6*a3*t + 12*a4*t*t + 20*a5*t*t*t; }
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "clean_flight_node");
    ros::NodeHandle nh;

    // 1. 初始化 ROS 通讯
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pos_cb);
    ros::Publisher target_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");

    ros::Rate rate(20.0);

    // 2. 安全检查：等待连接 + 等待动捕数据
    ROS_INFO("Waiting for FCU connection...");
    while(ros::ok() && !current_state.connected){ ros::spinOnce(); rate.sleep(); }
    
    ROS_INFO("Waiting for Position Data...");
    while(ros::ok() && !has_pose_received){ ros::spinOnce(); rate.sleep(); }
    ROS_INFO("System Ready. Prepare for Auto Start!");

    // 3. 任务变量初始化
    TaskState current_task = PREPARE;
    QuinticPoly traj_x, traj_y, traj_z;
    
    // 用于记录各个阶段的起始状态
    double start_x=0, start_y=0, start_z=0, start_yaw=0;
    double takeoff_z_target = 0;
    
    // 计时器
    ros::Time task_start_time = ros::Time::now();
    ros::Time last_req_time = ros::Time::now();

    while(ros::ok()){
        ros::spinOnce();

        mavros_msgs::PositionTarget raw_target;
        raw_target.header.stamp = ros::Time::now();
        raw_target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

        switch (current_task) {

            // ------------------------------------------------
            // 阶段 0: 预热 + 自动切 OFFBOARD + 解锁
            // ------------------------------------------------
            case PREPARE: {
                // 持续发送当前位置 (即使不动也要发，为了满足 Offboard 要求)
                raw_target.type_mask = 0b101111111000; // 仅位置 + Yaw
                raw_target.position = current_pose.pose.position;
                raw_target.yaw = calculate_yaw(current_pose.pose.orientation);

                // 预热足够时间后，开始尝试切模式
                if (ros::Time::now() - task_start_time > ros::Duration(TIME_WARMUP)) {
                    if(current_state.mode != "OFFBOARD" && (ros::Time::now() - last_req_time > ros::Duration(5.0))){
                        mavros_msgs::SetMode offb_set_mode;
                        offb_set_mode.request.custom_mode = "OFFBOARD";
                        if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) ROS_INFO("Offboard enabled");
                        last_req_time = ros::Time::now();
                    } 
                    else if(!current_state.armed && (ros::Time::now() - last_req_time > ros::Duration(5.0))){
                        mavros_msgs::CommandBool arm_cmd;
                        arm_cmd.request.value = true;
                        if(arming_client.call(arm_cmd) && arm_cmd.response.success) ROS_INFO("Vehicle armed");
                        last_req_time = ros::Time::now();
                    }
                }

                // 成功起飞条件：模式正确 + 已解锁
                if (current_state.mode == "OFFBOARD" && current_state.armed) {
                    ROS_INFO("Taking Off...");
                    
                    // 【关键点1】在这里读取当前位置作为起飞起点
                    start_x = current_pose.pose.position.x;
                    start_y = current_pose.pose.position.y;
                    start_z = current_pose.pose.position.z;
                    start_yaw = calculate_yaw(current_pose.pose.orientation);
                    takeoff_z_target = start_z + TAKEOFF_HEIGHT; // 目标高度

                    current_task = TAKEOFF;
                }
                break;
            }

            // ------------------------------------------------
            // 阶段 1: 垂直起飞
            // ------------------------------------------------
            case TAKEOFF: {
                raw_target.type_mask = 0b101111111000;
                raw_target.position.x = start_x; // 保持水平位置不变
                raw_target.position.y = start_y;
                raw_target.position.z = takeoff_z_target;
                raw_target.yaw = start_yaw;

                // 判断是否到达高度
                if (std::abs(current_pose.pose.position.z - takeoff_z_target) < 0.1) {
                    ROS_INFO("Takeoff Done. Planning Trajectory...");


                    start_x = current_pose.pose.position.x;
                    start_y = current_pose.pose.position.y;
                    start_z = current_pose.pose.position.z;
                    
                    // 规划五次多项式 (从当前点 -> 全局目标点)
                    traj_x.compute(start_x, TARGET_X, TIME_TRAJ);
                    traj_y.compute(start_y, TARGET_Y, TIME_TRAJ);
                    traj_z.compute(start_z, TARGET_Z, TIME_TRAJ);

                    task_start_time = ros::Time::now(); // 重置计时器
                    current_task = TRAJECTORY;
                }
                break;
            }

            // ------------------------------------------------
            // 阶段 2: 执行轨迹
            // ------------------------------------------------
            case TRAJECTORY: {
                double t = (ros::Time::now() - task_start_time).toSec();
                
                if (t > TIME_TRAJ) {
                    t = TIME_TRAJ; // 防止过冲
                    if (current_task != HOLD) {
                        ROS_INFO("Trajectory Finished. Hovering...");
                        task_start_time = ros::Time::now(); // 重置计时器用于悬停倒计时
                        current_task = HOLD;
                    }
                }

                raw_target.type_mask = 0; // 全状态控制 P+V+A
                raw_target.position.x = traj_x.get_pos(t);
                raw_target.position.y = traj_y.get_pos(t);
                raw_target.position.z = traj_z.get_pos(t);
                raw_target.velocity.x = traj_x.get_vel(t);
                raw_target.velocity.y = traj_y.get_vel(t);
                raw_target.velocity.z = traj_z.get_vel(t);
                raw_target.acceleration_or_force.x = traj_x.get_acc(t);
                raw_target.acceleration_or_force.y = traj_y.get_acc(t);
                raw_target.acceleration_or_force.z = traj_z.get_acc(t);
                raw_target.yaw = start_yaw; // 保持机头朝向不变
                break;
            }

            // ------------------------------------------------
            // 阶段 3: 悬停保持
            // ------------------------------------------------
            case HOLD: {
                double t_hold = (ros::Time::now() - task_start_time).toSec();
                
                // 保持在轨迹终点
                raw_target.type_mask = 0b101111111000;
                raw_target.position.x = TARGET_X;
                raw_target.position.y = TARGET_Y;
                raw_target.position.z = TARGET_Z;
                raw_target.yaw = start_yaw;

                if (t_hold > TIME_HOVER) {
                    ROS_INFO("Hover time up. Landing...");
                    current_task = LAND;
                }
                break;
            }

            // ------------------------------------------------
            // 阶段 4: 降落
            // ------------------------------------------------
            case LAND: {
                mavros_msgs::CommandTOL land_cmd;
                if (land_client.call(land_cmd) && land_cmd.response.success) {
                    ROS_INFO("Auto Land Triggered.");
                    current_task = FINISHED;
                } else {
                    // 如果失败，继续保持悬停，1秒后重试
                    raw_target.position.x = TARGET_X;
                    raw_target.position.y = TARGET_Y;
                    raw_target.position.z = TARGET_Z;
                    raw_target.yaw = start_yaw;
                    ros::Duration(1.0).sleep(); 
                }
                break;
            }

            case FINISHED:
                break;
        }

        // 安全发布：除了结束状态，其他时候都要发指令
        if (current_task != FINISHED && current_task != LAND) {
            target_pub.publish(raw_target);
        }

        rate.sleep();
    }
    return 0;
}