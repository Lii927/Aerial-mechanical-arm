#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h> 
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h> // 降落服务
#include <mavros_msgs/PositionTarget.h>
#include <cmath>

// ==========================================
// 1. 全局配置区 (实机飞行参数)
// ==========================================
// 目标位置 (动捕坐标系)
const double TARGET_X = 1.5; 
const double TARGET_Y = 0.0;
const double TARGET_Z = 1.2;

// 起飞高度
const double TAKEOFF_H = 1.0;

// PID 限制 (实机要比仿真保守)
const double MAX_VEL_XY = 1.0;   // 最大水平速度 1m/s
const double MAX_VEL_Z  = 0.8;   // 最大垂直速度
const double MAX_ACC_XY = 2.0;   // 最大水平加速度 2m/s^2
const double MAX_ACC_Z  = 2.0;   // 最大垂直加速度

const double TIME_HOVER_PID = 10.0; // PID控制保持时间

// ==========================================

// --- 工具函数：限幅 ---
double clamp(double value, double min, double max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

// --- 通用 PID 控制器类 ---
class PID {
public:
    double kp, ki, kd;
    double error_sum = 0;
    double error_last = 0;
    double max_output; 

    PID(double p, double i, double d, double max_v) 
        : kp(p), ki(i), kd(d), max_output(max_v) {}

    void reset() { error_sum = 0; error_last = 0; }

    double calculate(double target, double current, double dt) {
        double error = target - current;
        
        // 积分项 (带抗饱和)
        error_sum += error * dt;
        error_sum = clamp(error_sum, -3.0, 3.0); // 实机积分限幅要小一点

        // 微分项
        double derivative = (error - error_last) / dt;
        error_last = error;

        // 计算输出
        double output = kp * error + ki * error_sum + kd * derivative;
        return clamp(output, -max_output, max_output);
    }
};

// --- 全局变量 ---
mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::TwistStamped current_vel; 
bool has_pose = false;
bool has_vel = false;

void state_cb(const mavros_msgs::State::ConstPtr& msg){ current_state = *msg; }
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){ current_pose = *msg; has_pose = true; }
void vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){ current_vel = *msg; has_vel = true; }

// --- 状态机 ---
enum State { 
    PREPARE,    // 预热 + 自动解锁
    TAKEOFF,    // 位置控制起飞
    PID_FLY,    // 串级PID控制飞行
    LAND,       // 降落
    FINISHED    // 结束
};

// --- 辅助：四元数转Yaw ---
double calculate_yaw(const geometry_msgs::Quaternion& q) {
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "real_pid_node");
    ros::NodeHandle nh;

    // 订阅与发布
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pos_cb);
    ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_local", 10, vel_cb);

    ros::Publisher target_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
    ros::Publisher debug_acc_pub = nh.advertise<geometry_msgs::Vector3>("my_debug/ideal_accel", 10);

    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");

    ros::Rate rate(20.0);
    double dt = 1.0 / 20.0;

    // --- 串级 PID 初始化 ---
    // 实机飞行建议参数 (比仿真稍微柔和一点)
    // 外环 (位置 -> 速度)
    PID pos_pid_x(1.0, 0.0, 0.0, MAX_VEL_XY);
    PID pos_pid_y(1.0, 0.0, 0.0, MAX_VEL_XY);
    PID pos_pid_z(1.0, 0.0, 0.0, MAX_VEL_Z); 

    // 内环 (速度 -> 加速度) P值适当降低防震荡，给一点I消除稳态误差
    PID vel_pid_x(1.5, 0.05, 0.02, MAX_ACC_XY);
    PID vel_pid_y(1.5, 0.05, 0.02, MAX_ACC_XY);
    PID vel_pid_z(2.0, 0.10, 0.02, MAX_ACC_Z);

    // 等待连接和数据
    ROS_INFO("Waiting for FCU & Mocap...");
    while(ros::ok() && (!current_state.connected || !has_pose || !has_vel)){ 
        ros::spinOnce(); rate.sleep(); 
    }
    ROS_INFO("System Ready. Prepare for Auto Start!");

    // 状态机变量
    State current_task = PREPARE;
    ros::Time state_start_time = ros::Time::now();
    ros::Time last_req = ros::Time::now();

    // 记录起飞点
    double start_x = 0, start_y = 0, start_z = 0, start_yaw = 0;
    double takeoff_target_z = 0;
    bool is_pid_initialized = false;

    while(ros::ok()){
        ros::spinOnce();

        mavros_msgs::PositionTarget raw_target;
        raw_target.header.stamp = ros::Time::now();
        raw_target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

        switch (current_task) {
            
            // ==========================================
            // 阶段 0: 预热与自动启动
            // ==========================================
            case PREPARE: {
                // 持续发送当前位置，建立数据流
                raw_target.type_mask = 0b101111111000; // 仅位置 + Yaw
                raw_target.position = current_pose.pose.position;
                raw_target.yaw = calculate_yaw(current_pose.pose.orientation);

                // 2秒后尝试解锁
                if (ros::Time::now() - state_start_time > ros::Duration(2.0)) {
                    if(current_state.mode != "OFFBOARD" && (ros::Time::now() - last_req > ros::Duration(5.0))){
                        mavros_msgs::SetMode offb_set_mode;
                        offb_set_mode.request.custom_mode = "OFFBOARD";
                        if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) ROS_INFO("Offboard enabled");
                        last_req = ros::Time::now();
                    } 
                    else if(!current_state.armed && (ros::Time::now() - last_req > ros::Duration(5.0))){
                        mavros_msgs::CommandBool arm_cmd;
                        arm_cmd.request.value = true;
                        if(arming_client.call(arm_cmd) && arm_cmd.response.success) ROS_INFO("Vehicle armed");
                        last_req = ros::Time::now();
                    }
                }

                // 成功启动
                if (current_state.mode == "OFFBOARD" && current_state.armed) {
                    ROS_INFO("Taking Off (Position Control)...");
                    start_x = current_pose.pose.position.x;
                    start_y = current_pose.pose.position.y;
                    start_z = current_pose.pose.position.z;
                    start_yaw = calculate_yaw(current_pose.pose.orientation);
                    takeoff_target_z = start_z + TAKEOFF_H;
                    
                    current_task = TAKEOFF;
                }
                break;
            }

            // ==========================================
            // 阶段 1: 垂直起飞 (使用位置控制更安全)
            // ==========================================
            case TAKEOFF: {
                raw_target.type_mask = 0b101111111000;
                raw_target.position.x = start_x; 
                raw_target.position.y = start_y;
                raw_target.position.z = takeoff_target_z;
                raw_target.yaw = start_yaw;

                if (std::abs(current_pose.pose.position.z - takeoff_target_z) < 0.2) {
                    ROS_INFO("Takeoff Done! Engaging Cascaded PID Control...");
                    current_task = PID_FLY;
                    state_start_time = ros::Time::now();
                    is_pid_initialized = false; // 准备重置PID
                }
                break;
            }

            // ==========================================
            // 阶段 2: 串级 PID 控制飞行
            // ==========================================
            case PID_FLY: {
                // 1. 初始化 (只执行一次)
                if (!is_pid_initialized) {
                    pos_pid_x.reset(); vel_pid_x.reset();
                    pos_pid_y.reset(); vel_pid_y.reset();
                    pos_pid_z.reset(); vel_pid_z.reset();
                    is_pid_initialized = true;
                    ROS_INFO("PID Reset Done. Moving to Target...");
                }

                double t_fly = (ros::Time::now() - state_start_time).toSec();
                
                // 2. PID 计算
                // --- 外环 (位置 -> 速度) ---
                double target_vel_x = pos_pid_x.calculate(TARGET_X, current_pose.pose.position.x, dt);
                double target_vel_y = pos_pid_y.calculate(TARGET_Y, current_pose.pose.position.y, dt);
                double target_vel_z = pos_pid_z.calculate(TARGET_Z, current_pose.pose.position.z, dt);

                // --- 内环 (速度 -> 加速度) ---
                // 注意：这里用的是 TwistStamped 里的真实速度
                double target_acc_x = vel_pid_x.calculate(target_vel_x, current_vel.twist.linear.x, dt);
                double target_acc_y = vel_pid_y.calculate(target_vel_y, current_vel.twist.linear.y, dt);
                double target_acc_z = vel_pid_z.calculate(target_vel_z, current_vel.twist.linear.z, dt);

                // 3. 填充指令 (全状态前馈)
                raw_target.type_mask = 0; // P+V+A 全开
                
                // 位置：告诉飞控你要去哪 (作为辅助参考)
                raw_target.position.x = TARGET_X;
                raw_target.position.y = TARGET_Y;
                raw_target.position.z = TARGET_Z;

                // 速度：PID算出来的期望速度 (作为前馈)
                raw_target.velocity.x = target_vel_x;
                raw_target.velocity.y = target_vel_y;
                raw_target.velocity.z = target_vel_z;

                // 加速度：PID算出来的期望加速度 (这是主力控制量)
                raw_target.acceleration_or_force.x = target_acc_x;
                raw_target.acceleration_or_force.y = target_acc_y;
                raw_target.acceleration_or_force.z = target_acc_z;

                raw_target.yaw = start_yaw;

                // 4. Debug 发布
                geometry_msgs::Vector3 debug_msg;
                debug_msg.x = target_acc_x; 
                debug_msg.y = target_acc_y;
                debug_msg.z = target_acc_z;
                debug_acc_pub.publish(debug_msg);

                // 5. 计时结束降落
                if (t_fly > TIME_HOVER_PID) {
                    ROS_INFO("PID Flight Time Up. Landing...");
                    current_task = LAND;
                }
                break;
            }

            // ==========================================
            // 阶段 3: 自动降落
            // ==========================================
            case LAND: {
                mavros_msgs::CommandTOL land_cmd;
                if (land_client.call(land_cmd) && land_cmd.response.success) {
                    ROS_INFO("Auto Land Triggered.");
                    current_task = FINISHED;
                } else {
                    ROS_WARN_THROTTLE(1, "Land failed, retrying...");
                }
                break;
            }

            case FINISHED:
                break;
        }

        // 安全发布
        if (current_task != FINISHED && current_task != LAND) {
            target_pub.publish(raw_target);
        }

        // 遥控器介入保护
        if (current_task != PREPARE && current_task != FINISHED && current_state.mode != "OFFBOARD") {
             ROS_WARN("Manual Interrupt! Loop Aborted.");
             current_task = FINISHED; 
        }

        rate.sleep();
    }
    return 0;
}