#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <geometry_msgs/Vector3.h>
#include <mavros_msgs/PositionTarget.h>
#include <cmath>
#include <vector>

// ==========================================
// 1. 全局配置区 (Configuration)
// ==========================================

// --- 定义航点结构体 ---
struct Waypoint {
    double x;
    double y;
    double z;
    double fly_time;  // 飞到该点所需的规划时间
    double hold_time; // 到达该点后的停留时间
};

// --- 【关键】任务列表 ---

const std::vector<Waypoint> MISSION_WAYPOINTS = {
//  { x, y, z, 规划飞行时间, 规划停留时间 }
    { 1.5,  0.0,  3.2,  2.0,  3.0 }, 
    { 1.0,  1.5,  2.0,   3.0,  5.0 }, 
    { 0.0,  0.0,  1.0,   3.0,  2.0 }  
};

// --- 判定参数 ---
const double ACCEPT_RADIUS = 0.10; // 位置判定阈值
const double TIMEOUT_EXTRA = 10.0;  // 超时保护

// --- 起飞参数 ---
const double TAKEOFF_HEIGHT = 1.0; 
const double TIME_WARMUP = 2.0;

// ==========================================

enum TaskState {
    PREPARE,    
    TAKEOFF,    
    MOVING,     // 飞向航点
    HOVER,      // 悬停
    LAND,       
    FINISHED    
};

mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
bool has_pose_received = false; 

void state_cb(const mavros_msgs::State::ConstPtr& msg){ current_state = *msg; }
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
    has_pose_received = true;
}

double calculate_yaw(const geometry_msgs::Quaternion& q) {
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

// 计算欧氏距离
double get_distance(double x, double y, double z) {
    return sqrt(pow(current_pose.pose.position.x - x, 2) +
                pow(current_pose.pose.position.y - y, 2) +
                pow(current_pose.pose.position.z - z, 2));
}

// 五次多项式类
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
    ros::init(argc, argv, "waypoint_hold_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pos_cb);
    ros::Publisher target_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");

    ros::Rate rate(20.0);

    // 等待连接
    while(ros::ok() && !current_state.connected){ ros::spinOnce(); rate.sleep(); }
    while(ros::ok() && !has_pose_received){ ros::spinOnce(); rate.sleep(); }
    ROS_INFO("System Ready. Waiting for Auto Start...");

    // 变量初始化
    TaskState current_task = PREPARE;
    QuinticPoly traj_x, traj_y, traj_z;
    
    double start_x=0, start_y=0, start_z=0, start_yaw=0;
    double takeoff_z_target = 0;
    
    int wp_index = 0; 
    int total_wps = MISSION_WAYPOINTS.size();
    
    ros::Time state_start_time = ros::Time::now(); // 状态计时器
    ros::Time last_req_time = ros::Time::now();

    while(ros::ok()){
        ros::spinOnce();

        mavros_msgs::PositionTarget raw_target;
        raw_target.header.stamp = ros::Time::now();
        raw_target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

        switch (current_task) {

            // ------------------------------------------------
            // 阶段 0: 预热 + 自动启动
            // ------------------------------------------------
            case PREPARE: {
                raw_target.type_mask = 0b101111111000; 
                raw_target.position = current_pose.pose.position;
                raw_target.yaw = calculate_yaw(current_pose.pose.orientation);

                if (ros::Time::now() - state_start_time > ros::Duration(TIME_WARMUP)) {
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

                if (current_state.mode == "OFFBOARD" && current_state.armed) {
                    ROS_INFO("Taking Off...");
                    start_x = current_pose.pose.position.x;
                    start_y = current_pose.pose.position.y;
                    start_z = current_pose.pose.position.z;
                    start_yaw = calculate_yaw(current_pose.pose.orientation);
                    takeoff_z_target = start_z + TAKEOFF_HEIGHT;
                    current_task = TAKEOFF;
                }
                break;
            }




            // ------------------------------------------------
            // 阶段 1: 垂直起飞
            // ------------------------------------------------
            case TAKEOFF: {
                raw_target.type_mask = 0b101111111000;
                raw_target.position.x = start_x;
                raw_target.position.y = start_y;
                raw_target.position.z = takeoff_z_target;
                raw_target.yaw = start_yaw;

                // 起飞完成判断
                if (std::abs(current_pose.pose.position.z - takeoff_z_target) < 0.15) {
                    ROS_INFO("Takeoff Done. Starting Mission...");

                    if (total_wps > 0) {
                        // 准备进入第一个 MOVING 状态
                        wp_index = 0;
                        
                        start_x = current_pose.pose.position.x;
                        start_y = current_pose.pose.position.y;
                        start_z = current_pose.pose.position.z;

                        traj_x.compute(start_x, MISSION_WAYPOINTS[wp_index].x, MISSION_WAYPOINTS[wp_index].fly_time);
                        traj_y.compute(start_y, MISSION_WAYPOINTS[wp_index].y, MISSION_WAYPOINTS[wp_index].fly_time);
                        traj_z.compute(start_z, MISSION_WAYPOINTS[wp_index].z, MISSION_WAYPOINTS[wp_index].fly_time);
                        
                        state_start_time = ros::Time::now();
                        current_task = MOVING;
                    } else {
                        current_task = LAND; // 没有航点直接降落
                    }
                }
                break;
            }

            // ------------------------------------------------
            // 阶段 2: 飞向航点 (MOVING)
            // ------------------------------------------------
            case MOVING: {
                double t = (ros::Time::now() - state_start_time).toSec();
                double planned_duration = MISSION_WAYPOINTS[wp_index].fly_time;
                
                // 1. 如果时间没到，继续计算轨迹
                if (t <= planned_duration) {
                    raw_target.type_mask = 0; // P+V+A
                    raw_target.position.x = traj_x.get_pos(t);
                    raw_target.position.y = traj_y.get_pos(t);
                    raw_target.position.z = traj_z.get_pos(t);
                    raw_target.velocity.x = traj_x.get_vel(t);
                    raw_target.velocity.y = traj_y.get_vel(t);
                    raw_target.velocity.z = traj_z.get_vel(t);
                    raw_target.acceleration_or_force.x = traj_x.get_acc(t);
                    raw_target.acceleration_or_force.y = traj_y.get_acc(t);
                    raw_target.acceleration_or_force.z = traj_z.get_acc(t);
                } 
                // 2. 如果时间到了，检查位置误差
                else {
                    // 保持发送最终目标点
                    raw_target.type_mask = 0b101111111000;
                    raw_target.position.x = MISSION_WAYPOINTS[wp_index].x;
                    raw_target.position.y = MISSION_WAYPOINTS[wp_index].y;
                    raw_target.position.z = MISSION_WAYPOINTS[wp_index].z;
                    
                    // 计算当前误差
                    double dist = get_distance(MISSION_WAYPOINTS[wp_index].x, 
                                               MISSION_WAYPOINTS[wp_index].y, 
                                               MISSION_WAYPOINTS[wp_index].z);
                    
                    bool is_reached = (dist < ACCEPT_RADIUS);
                    bool is_timeout = (t > planned_duration + TIMEOUT_EXTRA);

                    // 到达 或者 超时 -> 进入停留模式
                    if (is_reached || is_timeout) {
                        if (is_reached) ROS_INFO("Target Reached (Dist: %.2fm). Holding for %.1fs...", dist, MISSION_WAYPOINTS[wp_index].hold_time);
                        else            ROS_WARN("Target Timeout (Dist: %.2fm). Force Holding...", dist);
                        
                        // 重置计时器，用于计算停留时间
                        state_start_time = ros::Time::now();
                        current_task = HOVER;
                    }
                }
                
                raw_target.yaw = start_yaw; 
                break;
            }

            // ------------------------------------------------
            // 阶段 3: 航点停留 (HOVER)
            // ------------------------------------------------
            case HOVER: {
                double t_hold = (ros::Time::now() - state_start_time).toSec();
                double required_hold = MISSION_WAYPOINTS[wp_index].hold_time;

                // 锁死位置
                raw_target.type_mask = 0b101111111000;
                raw_target.position.x = MISSION_WAYPOINTS[wp_index].x;
                raw_target.position.y = MISSION_WAYPOINTS[wp_index].y;
                raw_target.position.z = MISSION_WAYPOINTS[wp_index].z;
                raw_target.yaw = start_yaw;

                // 停留时间结束
                if (t_hold > required_hold) {
                    // 检查是否还有下一个点
                    if (wp_index < total_wps - 1) {
                        wp_index++; // 进位
                        ROS_INFO("Moving to Waypoint %d...", wp_index);

                        // 规划下一段轨迹 (从当前位置出发)
                        start_x = current_pose.pose.position.x;
                        start_y = current_pose.pose.position.y;
                        start_z = current_pose.pose.position.z;

                        traj_x.compute(start_x, MISSION_WAYPOINTS[wp_index].x, MISSION_WAYPOINTS[wp_index].fly_time);
                        traj_y.compute(start_y, MISSION_WAYPOINTS[wp_index].y, MISSION_WAYPOINTS[wp_index].fly_time);
                        traj_z.compute(start_z, MISSION_WAYPOINTS[wp_index].z, MISSION_WAYPOINTS[wp_index].fly_time);

                        state_start_time = ros::Time::now();
                        current_task = MOVING; // 切回飞行模式
                    } else {
                        // 全部飞完
                        ROS_INFO("All Mission Completed. Landing...");
                        current_task = LAND;
                    }
                }
                break;
            }

            case LAND: {
                mavros_msgs::CommandTOL land_cmd;
                if (land_client.call(land_cmd) && land_cmd.response.success) {
                    ROS_INFO("Auto Land Triggered.");
                    current_task = FINISHED;
                } else {
                    ros::Duration(1.0).sleep(); 
                }
                break;
            }

            case FINISHED:
                break;
        }

        if (current_task != FINISHED && current_task != LAND) {
            target_pub.publish(raw_target);
        }

        rate.sleep();
    }
    return 0;
}
