#include <ros/ros.h>
#include <can_msgs/ecu.h>
#include "common.hpp"
#include <linux/can.h>

// 包含所有自定义消息头文件
#include <pix_driver/DriveStatusFb.h>
#include <pix_driver/BrakeStatusFb.h>
#include <pix_driver/SteerStatusFb.h>
#include <pix_driver/WorkStatusFb.h>
#include <pix_driver/BatteryStatusFb.h>
#include <pix_driver/VehicleStatusFb.h>
#include <pix_driver/VehicleFltStatusFb.h>
#include <pix_driver/ChassisWheelRpmFb.h>
#include <pix_driver/ChassisOdoFb.h>
#include <pix_driver/ChassisWheelTirePressFb.h>
#include <pix_driver/ChassisWheelAngleFb.h>
#include <pix_driver/ChassisWheelTorqueFb.h>

#include <boost/asio.hpp>
#include <thread>
#include <mutex>
#include <endian.h>

using boost::asio::ip::udp;

// GLOBAL 系统参数设置
const double max_steer_angle = 26.0;  // 最大转角读数26°



template<typename T>
T clamp(T value, T min_value, T max_value) {
    return std::max(min_value, std::min(value, max_value));
}


class CANBridgeNode {
public:
    CANBridgeNode(ros::NodeHandle& nh) :
        nh_(nh),
        pnh_("~"),
        socket_(io_context_),
        receiver_socket_(io_context_),
        target_endpoint_(boost::asio::ip::address::from_string("192.168.100.110"), 4001){

        pnh_.param<bool>("debug", _debug_mode, false);
        
        last_ecu_.shift = can_msgs::ecu::SHIFT_N;

        // 初始化UDP发送套接字
        socket_.open(udp::v4());

        // 初始化UDP接收套接字
        receiver_socket_.open(udp::v4());
        receiver_socket_.bind(udp::endpoint(udp::v4(), 8001));

        // 初始化所有发布者
        drive_sta_pub_        = nh_.advertise<pix_driver::DriveStatusFb>("/can/drive_status", 10);
        brake_sta_pub_        = nh_.advertise<pix_driver::BrakeStatusFb>("/can/brake_status", 10);
        steer_sta_pub_        = nh_.advertise<pix_driver::SteerStatusFb>("/can/steer_status", 10);
        vehicle_work_sta_pub_ = nh_.advertise<pix_driver::WorkStatusFb>("/can/vehicle_work_status", 10);
        power_sta_pub_        = nh_.advertise<pix_driver::BatteryStatusFb>("/can/power_status", 10);
        vehicle_sta_pub_      = nh_.advertise<pix_driver::VehicleStatusFb>("/can/vehicle_status", 10);
        vehicle_flt_sta_pub_  = nh_.advertise<pix_driver::VehicleFltStatusFb>("/can/vehicle_fault_status", 10);
        wheel_rpm_pub_        = nh_.advertise<pix_driver::ChassisWheelRpmFb>("/can/wheel_rpm", 10);
        odo_pub_              = nh_.advertise<pix_driver::ChassisOdoFb>("/can/odometer", 10);
        tire_press_pub_       = nh_.advertise<pix_driver::ChassisWheelTirePressFb>("/can/tire_pressure", 10);
        wheel_angle_pub_      = nh_.advertise<pix_driver::ChassisWheelAngleFb>("/can/wheel_angle", 10);
        wheel_torque_pub_     = nh_.advertise<pix_driver::ChassisWheelTorqueFb>("/can/wheel_torque", 10);

        // 启动接收线程
        receiver_thread_ = std::thread(&CANBridgeNode::udpReceiver, this);

        // ROS订阅和定时器
        ecu_sub_ = nh_.subscribe("/ecu", 1, &CANBridgeNode::ecuCallback, this);
        timer_ = nh_.createTimer(ros::Duration(0.05), &CANBridgeNode::timerCallback, this);
        // timer_pub_status_ = nh_.createTimer(ros::Duration(1.0), &CANBridgeNode::pubStatus, this);  // 电池消息, 里程消息, 胎压消息, 扭矩反馈消息, 错误消息, 车辆状态消息, 工作状态消息
    }

    ~CANBridgeNode() {
        receiver_thread_.join();
    }

private:
    void ecuCallback(const can_msgs::ecu::ConstPtr& msg) {
        last_cmd_time_ = ros::Time::now();
        last_ecu_ = *msg;
    }

    void timerCallback(const ros::TimerEvent& e) {
        if (this->current_gear_ == pix_driver::DriveStatusFb::GearStatusNoUse) {
            ROS_INFO_THROTTLE(1, "[can driver] Gear status not initialized!");
            return;
        }

        // 超时处理（1s）
        if ((ros::Time::now() - last_cmd_time_).toSec() > 1.0 || last_ecu_.shift == last_ecu_.SHIFT_N) {
            this->stop_robot();
            return;
        }

        can_frame drive_frame   = createDriveCtrlFrame(last_ecu_);
        can_frame brake_frame   = createBrakeCtrlFrame(last_ecu_);
        can_frame steer_frame   = createSteerCtrlFrame(last_ecu_);
        can_frame vehicle_frame = createVehicleCtrlFrame(last_ecu_);

        // 发送UDP报文
        send_frame(drive_frame);
        send_frame(brake_frame);
        send_frame(steer_frame);
        send_frame(vehicle_frame);
    }

    void stop_robot() {
        can_msgs::ecu ecu_msg;
        ecu_msg.shift = ecu_msg.SHIFT_N;
        last_ecu_.motor = 0.0;
        // ecu_msg.brake = true;
        can_frame drive_frame   = createDriveCtrlFrame(last_ecu_);
        can_frame brake_frame   = createBrakeCtrlFrame(last_ecu_);
        can_frame steer_frame   = createSteerCtrlFrame(last_ecu_);
        can_frame vehicle_frame = createVehicleCtrlFrame(last_ecu_);

        // 发送UDP报文
        send_frame(drive_frame);
        send_frame(brake_frame);
        send_frame(steer_frame);
        send_frame(vehicle_frame);
    }

    void send_frame(const can_frame &frame) {
        std::lock_guard<std::mutex> lock(mutex_);
        uint8_t can_packet[13] = {0};
        // 1.帧信息（1字节）：FF=1(扩展帧), RTR=0, UDP=1, DLC=8
        can_packet[0] = 0x08;  // data长度8字节
        can_packet[1] = (uint8_t)((frame.can_id >> 24) & 0xFF);
        can_packet[2] = (uint8_t)((frame.can_id >> 16) & 0xFF);
        can_packet[3] = (uint8_t)((frame.can_id >> 8) & 0xFF);
        can_packet[4] = (uint8_t)(frame.can_id & 0xFF);
        memcpy(&can_packet[5], frame.data, 8);
        ROS_WARN("[can driver] Send: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x", can_packet[0], can_packet[1], can_packet[2], can_packet[3], can_packet[4], can_packet[5], can_packet[6], can_packet[7], can_packet[8], can_packet[9], can_packet[10], can_packet[11], can_packet[12]);

        socket_.send_to(boost::asio::buffer(can_packet, 13), target_endpoint_);
    }

    void udpReceiver() {
        uint8_t buffer[128];
        while (ros::ok()) {
            try {
                udp::endpoint remote_ep;
                size_t len = receiver_socket_.receive_from(boost::asio::buffer(buffer), remote_ep);

                // 检查数据包长度
                if (len < 13) {  // CAN帧应为13字节
                    ROS_WARN_THROTTLE(1, "[can driver] Invalid CAN frame length: %zu (expected 13)", len);
                    continue;
                }

                // 解析CAN ID (1~4字节)
                uint32_t can_id = be32toh(*reinterpret_cast<uint32_t*>(buffer+1));
                // 获取数据段（最后8字节）
                const uint8_t* data = buffer + 5;

                // 根据CAN ID处理不同报文
                switch (can_id) {
                    case 0x530: // 驱动状态反馈
                        processDriveStatus(data);
                        break;
                    case 0x531: // 制动状态反馈
                        processBrakeStatus(data);
                        break;
                    case 0x532: // 转向状态反馈
                        processSteerStatus(data);
                        break;
                    case 0x534: // 车辆工作状态反馈
                        processVehicleWorkStatus(data);
                        break;
                    case 0x535: // 动力电源状态反馈
                        processPowerStatus(data);
                        break;
                    case 0x536: // 车辆车身及灯光反馈
                        processVehicleStatus(data);
                        break;
                    case 0x537: // 车辆警告和故障
                        processFaultStatus(data);
                        break;
                    case 0x539: // 车轮转速反馈
                        processWheelRpm(data);
                        break;
                    case 0x53A: // 行驶里程
                        processOdometer(data);
                        break;
                    case 0x540: // 胎压反馈
                        processTirePressure(data);
                        break;
                    case 0x541: // 车轮转角反馈
                        processWheelAngle(data);
                        break;
                    case 0x542: // 车轮扭矩反馈
                        processWheelTorque(data);
                        break;
                    default:
                        ROS_DEBUG_THROTTLE(1, "[can driver] Unknown CAN ID: 0x%X", can_id);
                        break;
                }
            }
            catch (const std::exception& e) {
                ROS_ERROR("接收错误: %s", e.what());
            }
        }
    }

    // 处理驱动状态反馈 (0x530)
    void processDriveStatus(const uint8_t* data) {
        pix_driver::DriveStatusFb msg;
        msg.header.stamp = ros::Time::now();

        // 使用BitBuffer工具类解析各个信号
        msg.drive_enable_status = PixDriver::readBit(data, 0);
        msg.drive_overrun_warning = PixDriver::readBit(data, 1);
        msg.drive_mode = PixDriver::readUInt(data, 2, 2);
        msg.gear_status = PixDriver::readUInt(data, 4, 2);
        this->current_gear_ = msg.gear_status;

        msg.drive_4wd_mode = PixDriver::readUInt(data, 6, 2);

        // 车速 (16位有符号，分辨率0.01 m/s)
        int16_t speed_raw = PixDriver::readInt(data, 8, 16);
        msg.speed = speed_raw * 0.01f;
        this->current_speed_ = msg.speed;

        // 油门 (10位无符号，分辨率0.1%)
        uint16_t throttle_raw = PixDriver::readUInt(data, 24, 10);
        msg.throttle = throttle_raw * 0.1f;

        // 加速度 (16位有符号，分辨率0.01 m/s²)
        int16_t accel_raw = PixDriver::readInt(data, 40, 16);
        msg.acceleration = accel_raw * 0.01f;

        drive_sta_pub_.publish(msg);

        // if (this->_debug_mode) {
        //     ROS_INFO_THROTTLE(1, "[can driver] 0x530 Drive Status: drive_enable_status=%d, drive_overrun_warning=%d, drive_mode=%d, gear_status=%d, drive_4wd_mode=%d, speed=%.2f, throttle=%.1f, acceleration=%.2f", msg.drive_enable_status, msg.drive_overrun_warning, msg.drive_mode, msg.gear_status, msg.drive_4wd_mode, msg.speed, msg.throttle, msg.acceleration);
        // }
    }

    // 处理制动状态反馈 (0x531)
    void processBrakeStatus(const uint8_t* data) {
        pix_driver::BrakeStatusFb msg;
        msg.header.stamp = ros::Time::now();

        msg.brake_enable_status = PixDriver::readBit(data, 0);
        msg.brake_lamp_status = PixDriver::readBit(data, 2);
        msg.epb_status = PixDriver::readUInt(data, 4, 2);
        this->current_p_status_ = msg.epb_status;

        // 制动踏板值 (10位无符号，分辨率0.1%)
        uint16_t brake_pedal_raw = PixDriver::readUInt(data, 8, 10);
        msg.brake_pedal = brake_pedal_raw * 0.1f;

        msg.aeb_enable_status = PixDriver::readBit(data, 20);
        msg.aeb_trigger_status = PixDriver::readBit(data, 22);

        // 制动压力 (8位无符号，分辨率1 bar)
        uint8_t brake_pressure_raw = PixDriver::readUInt(data, 24, 8);
        msg.brake_pressure = static_cast<float>(brake_pressure_raw);

        brake_sta_pub_.publish(msg);

        // if (this->_debug_mode) {
        //     ROS_INFO_THROTTLE(1, "[can driver] 0x531 Brake Status: brake_enable_status=%d, brake_lamp_status=%d, epb_status=%d, brake_pedal=%.1f, aeb_enable_status=%d, aeb_trigger_status=%d, brake_pressure=%.2f",
        //     msg.brake_enable_status, msg.brake_lamp_status, msg.epb_status, msg.brake_pedal, msg.aeb_enable_status, msg.aeb_trigger_status, msg.brake_pressure
        //     );
        // }
    }

    // 处理转向状态反馈 (0x532)
    void processSteerStatus(const uint8_t* data) {
        pix_driver::SteerStatusFb msg;
        msg.header.stamp = ros::Time::now();

        msg.steer_enable_status = PixDriver::readBit(data, 0);
        msg.steer_overrun_warning = PixDriver::readBit(data, 1);
        msg.steer_work_mode = PixDriver::readUInt(data, 2, 2);
        msg.steer_mode = PixDriver::readUInt(data, 4, 4);

        // 前转向角度 (16位有符号，分辨率1度)
        int16_t steer_angle_front_raw = PixDriver::readInt(data, 8, 16);
        msg.steer_angle_front = static_cast<float>(steer_angle_front_raw);

        // 后转向角度 (16位有符号，分辨率1度)
        int16_t steer_angle_rear_raw = PixDriver::readInt(data, 24, 16);
        msg.steer_angle_rear = static_cast<float>(steer_angle_rear_raw);

        // 转向角速度 (8位无符号，分辨率2 deg/s)
        uint8_t steer_speed_raw = PixDriver::readUInt(data, 40, 8);
        msg.steer_angle_speed = steer_speed_raw * 2.0f;

        steer_sta_pub_.publish(msg);

        // if (this->_debug_mode) {
        //     ROS_INFO_THROTTLE(1, "[can driver] Steer Status: steer_enable_status=%d, steer_overrun_warning=%d, steer_work_mode=%d, steer_mode=%d, steer_angle_front=%.1f, steer_angle_rear=%.1f, steer_angle_speed=%.1f",
        //     msg.steer_enable_status, msg.steer_overrun_warning, msg.steer_work_mode, msg.steer_mode, msg.steer_angle_front, msg.steer_angle_rear, msg.steer_angle_speed
        //     );
        // }
    }

    // 处理车辆工作状态反馈 (0x534)
    void processVehicleWorkStatus(const uint8_t* data) {
        pix_driver::WorkStatusFb msg;
        msg.header.stamp = ros::Time::now();

        msg.drive_mode = PixDriver::readUInt(data, 0, 2);
        msg.power_status = PixDriver::readUInt(data, 2, 2);
        msg.dc_status = PixDriver::readUInt(data, 4, 2);
        msg.limit_status = PixDriver::readBit(data, 8);
        msg.power_limit_status = PixDriver::readBit(data, 9);
        msg.eco_status = PixDriver::readUInt(data, 10, 2);

        // 速度限制值 (16位无符号，分辨率0.1 m/s)
        uint16_t speed_limit_raw = PixDriver::readUInt(data, 16, 16);
        msg.chassis_speed_limit = speed_limit_raw * 0.1f;

        // 低压蓄电池电压 (8位无符号，分辨率0.1 V)
        uint8_t low_power_volt_raw = PixDriver::readUInt(data, 32, 8);
        msg.chassis_low_power_vol = low_power_volt_raw * 0.1f;

        msg.e_stop_status = PixDriver::readUInt(data, 40, 4);
        msg.front_crash_status = PixDriver::readBit(data, 44);
        msg.back_crash_status = PixDriver::readBit(data, 45);
        msg.left_crash_status = PixDriver::readBit(data, 46);
        msg.right_crash_status = PixDriver::readBit(data, 47);

        vehicle_work_sta_pub_.publish(msg);

        // if (this->_debug_mode) {
        //     ROS_INFO_THROTTLE(1,
        //         "[can driver] 0x534 Vehicle Work Status: drive_mode=%d, "
        //         "power_status=%d, "
        //         "dc_status=%d, "
        //         "limit_status=%d, "
        //         "power_limit_status=%d, "
        //         "eco_status=%d, "
        //         "chassis_speed_limit=%.1f, "
        //         "chassis_low_power_vol=%.1f, "
        //         "e_stop_status=%d, "
        //         "front_crash_status=%d, "
        //         "back_crash_status=%d, "
        //         "left_crash_status=%d, "
        //         "right_crash_status=%d",
        //         msg.drive_mode,
        //         msg.power_status,
        //         msg.dc_status,
        //         msg.limit_status,
        //         msg.power_limit_status,
        //         msg.eco_status,
        //         msg.chassis_speed_limit,
        //         msg.chassis_low_power_vol,
        //         msg.e_stop_status,
        //         msg.front_crash_status,
        //         msg.back_crash_status,
        //         msg.left_crash_status,
        //         msg.right_crash_status
        //     );
        // }
    }

    // 处理动力电源状态反馈 (0x535)
    void processPowerStatus(const uint8_t* data) {
        pix_driver::BatteryStatusFb msg;
        msg.header.stamp = ros::Time::now();

        msg.charge_status = PixDriver::readUInt(data, 4, 2);
        msg.charge_sock_status = PixDriver::readBit(data, 6);

        // 动力电池电量 (8位无符号，分辨率1%)
        uint8_t soc_raw = PixDriver::readUInt(data, 8, 8);
        msg.power_soc = static_cast<float>(soc_raw);

        // 动力电池电压 (16位无符号，分辨率0.1 V)
        uint16_t voltage_raw = PixDriver::readUInt(data, 16, 16);
        msg.power_voltage = voltage_raw * 0.1f;

        // 动力电池电流 (16位有符号，分辨率0.1 A，偏移-1000)
        int16_t current_raw = PixDriver::readInt(data, 32, 16);
        msg.power_current = current_raw * 0.1f - 1000.0f;

        // BMS最高单体温度 (8位有符号，分辨率1 °C，偏移-40)
        int8_t temp_raw = static_cast<int8_t>(PixDriver::readUInt(data, 48, 8));
        msg.bms_max_temp = static_cast<float>(temp_raw) - 40.0f;

        power_sta_pub_.publish(msg);

        // if (this->_debug_mode) {
        //     ROS_INFO_THROTTLE(1, "[can driver] 0x535 Power Status: charge_status=%d, charge_sock_status=%d, power_soc=%.1f, power_voltage=%.1f, power_current=%.1f, bms_max_temp=%.1f",
        //         msg.charge_status,
        //         msg.charge_sock_status,
        //         msg.power_soc,
        //         msg.power_voltage,
        //         msg.power_current,
        //         msg.bms_max_temp
        //     );
        // }
    }

    // 处理车辆车身及灯光反馈 (0x536)
    void processVehicleStatus(const uint8_t* data) {
        pix_driver::VehicleStatusFb msg;
        msg.header.stamp = ros::Time::now();

        // 灯光状态
        msg.pos_lamp_status = PixDriver::readBit(data, 0);
        msg.head_lamp_status = PixDriver::readBit(data, 1);
        msg.left_lamp_status = PixDriver::readBit(data, 2);
        msg.right_lamp_status = PixDriver::readBit(data, 3);
        msg.high_beam_status = PixDriver::readBit(data, 4);
        msg.fog_lamp_status = PixDriver::readBit(data, 5);
        msg.hazard_war_lamp_status = PixDriver::readBit(data, 6);
        msg.body_lamp_status = PixDriver::readBit(data, 7);
        msg.read_lamp_status = PixDriver::readBit(data, 8);

        // 车门状态
        msg.door_status = PixDriver::readUInt(data, 16, 4);

        // 雨刮状态
        msg.wipers_status = PixDriver::readUInt(data, 20, 2);

        // 安全带状态
        msg.safety_belt1 = PixDriver::readUInt(data, 24, 2);
        msg.safety_belt2 = PixDriver::readUInt(data, 26, 2);
        msg.safety_belt3 = PixDriver::readUInt(data, 28, 2);
        msg.safety_belt4 = PixDriver::readUInt(data, 30, 2);

        vehicle_sta_pub_.publish(msg);

        // if (this->_debug_mode) {
        //     ROS_INFO_THROTTLE(1, "[can driver] 0x536 Vehicle Status: pos_lamp_status=%d, head_lamp_status=%d, left_lamp_status=%d, right_lamp_status=%d, high_beam_status=%d, fog_lamp_status=%d, hazard_war_lamp_status=%d, body_lamp_status=%d, read_lamp_status=%d, door_status=%d, wipers_status=%d, safety_belt1=%d, safety_belt2=%d, safety_belt3=%d, safety_belt4=%d",
        //         msg.pos_lamp_status,
        //         msg.head_lamp_status,
        //         msg.left_lamp_status,
        //         msg.right_lamp_status,
        //         msg.high_beam_status,
        //         msg.fog_lamp_status,
        //         msg.hazard_war_lamp_status,
        //         msg.body_lamp_status,
        //         msg.read_lamp_status,
        //         msg.door_status,
        //         msg.wipers_status,
        //         msg.safety_belt1,
        //         msg.safety_belt2,
        //         msg.safety_belt3,
        //         msg.safety_belt4
        //     );
        // }
    }

    // 处理车辆警告和故障 (0x537)
    void processFaultStatus(const uint8_t* data) {
        pix_driver::VehicleFltStatusFb msg;
        msg.header.stamp = ros::Time::now();

        // 过温状态
        msg.motor_over_temp_status = PixDriver::readBit(data, 0);
        msg.bms_over_temp_status = PixDriver::readBit(data, 1);
        msg.brake_over_temp_status = PixDriver::readBit(data, 2);
        msg.steer_over_temp_status = PixDriver::readBit(data, 3);
        msg.under_volt_status = PixDriver::readBit(data, 4);

        // 故障等级
        msg.sys_fault = PixDriver::readUInt(data, 8, 4);
        msg.brake_fault = PixDriver::readUInt(data, 12, 4);
        msg.parking_fault = PixDriver::readUInt(data, 16, 4);
        msg.steer_front_fault = PixDriver::readUInt(data, 20, 4);
        msg.steer_back_fault = PixDriver::readUInt(data, 24, 4);
        msg.motor_lf_fault = PixDriver::readUInt(data, 28, 4);
        msg.motor_rf_fault = PixDriver::readUInt(data, 32, 4);
        msg.motor_lr_fault = PixDriver::readUInt(data, 36, 4);
        msg.motor_rr_fault = PixDriver::readUInt(data, 40, 4);
        msg.bms_fault = PixDriver::readUInt(data, 44, 4);
        msg.dc_fault = PixDriver::readUInt(data, 48, 4);
        msg.fault_reserve1 = PixDriver::readUInt(data, 52, 4);

        vehicle_flt_sta_pub_.publish(msg);

        // if (this->_debug_mode) {
        //     ROS_INFO_THROTTLE(1, "[can driver] 0x537 Vehicle Fault Status: motor_over_temp_status=%d, bms_over_temp_status=%d, brake_over_temp_status=%d, steer_over_temp_status=%d, under_volt_status=%d, sys_fault=%d, brake_fault=%d, parking_fault=%d, steer_front_fault=%d, steer_back_fault=%d, motor_lf_fault=%d, motor_rf_fault=%d, motor_lr_fault=%d, motor_rr_fault=%d, bms_fault=%d, dc_fault=%d, fault_reserve1=%d",
        //         msg.motor_over_temp_status,
        //         msg.bms_over_temp_status,
        //         msg.brake_over_temp_status,
        //         msg.steer_over_temp_status,
        //         msg.under_volt_status,
        //         msg.sys_fault,
        //         msg.brake_fault,
        //         msg.parking_fault,
        //         msg.steer_front_fault,
        //         msg.steer_back_fault,
        //         msg.motor_lf_fault,
        //         msg.motor_rf_fault,
        //         msg.motor_lr_fault,
        //         msg.motor_rr_fault,
        //         msg.bms_fault,
        //         msg.dc_fault,
        //         msg.fault_reserve1
        //     );
        // }
    }

    // 处理车轮转速反馈 (0x539)
    void processWheelRpm(const uint8_t* data) {
        pix_driver::ChassisWheelRpmFb msg;
        msg.header.stamp = ros::Time::now();

        // 车轮转速 (16位有符号，分辨率1 rpm)
        int16_t rpm_lf = PixDriver::readInt(data, 0, 16);
        int16_t rpm_rf = PixDriver::readInt(data, 16, 16);
        int16_t rpm_lr = PixDriver::readInt(data, 32, 16);
        int16_t rpm_rr = PixDriver::readInt(data, 48, 16);

        msg.wheel_rpm_lf = static_cast<float>(rpm_lf);
        msg.wheel_rpm_rf = static_cast<float>(rpm_rf);
        msg.wheel_rpm_lr = static_cast<float>(rpm_lr);
        msg.wheel_rpm_rr = static_cast<float>(rpm_rr);

        wheel_rpm_pub_.publish(msg);

        // if (this->_debug_mode) {
        //     ROS_INFO_THROTTLE(1, "[can driver] 0x539 Wheel RPM: wheel_rpm_lf=%d, wheel_rpm_rf=%d, wheel_rpm_lr=%d, wheel_rpm_rr=%d",
        //         rpm_lf,
        //         rpm_rf,
        //         rpm_lr,
        //         rpm_rr
        //     );
        // }
    }

    // 处理行驶里程 (0x53A)
    void processOdometer(const uint8_t* data) {
        pix_driver::ChassisOdoFb msg;
        msg.header.stamp = ros::Time::now();

        // 总里程 (24位无符号，分辨率1 km)
        uint32_t total_odo_raw = PixDriver::readUInt(data, 0, 24);
        msg.total_odo = static_cast<float>(total_odo_raw);

        // 单次里程 (24位无符号，分辨率0.01 km)
        uint32_t trip_odo_raw = PixDriver::readUInt(data, 24, 24);
        msg.trip_odo = trip_odo_raw * 0.01f;

        // 剩余里程 (16位无符号，分辨率1 km)
        uint16_t remain_range_raw = PixDriver::readUInt(data, 48, 16);
        msg.remaining_range = static_cast<float>(remain_range_raw);

        odo_pub_.publish(msg);

        // if (this->_debug_mode) {
        //     ROS_INFO_THROTTLE(1, "[can driver] 0x53A Odometer: total_odo=%d, trip_odo=%d, remain_range=%d",
        //         total_odo_raw,
        //         trip_odo_raw,
        //         remain_range_raw
        //     );
        // }
    }

    // 处理胎压反馈 (0x540)
    void processTirePressure(const uint8_t* data) {
        pix_driver::ChassisWheelTirePressFb msg;
        msg.header.stamp = ros::Time::now();

        // 胎压 (12位无符号，分辨率0.01 bar)
        uint16_t press_lf = PixDriver::readUInt(data, 0, 12);
        uint16_t press_rf = PixDriver::readUInt(data, 16, 12);
        uint16_t press_lr = PixDriver::readUInt(data, 32, 12);
        uint16_t press_rr = PixDriver::readUInt(data, 48, 12);

        msg.tire_press_lf = press_lf * 0.01f;
        msg.tire_press_rf = press_rf * 0.01f;
        msg.tire_press_lr = press_lr * 0.01f;
        msg.tire_press_rr = press_rr * 0.01f;

        tire_press_pub_.publish(msg);

        // if (this->_debug_mode) {
        //     ROS_INFO_THROTTLE(1, "[can driver] 0x540 Tire Pressure: tire_press_lf=%d, tire_press_rf=%d, tire_press_lr=%d, tire_press_rr=%d",
        //         press_lf,
        //         press_rf,
        //         press_lr,
        //         press_rr
        //     );
        // }
    }

    // 处理车轮转角反馈 (0x541)
    void processWheelAngle(const uint8_t* data) {
        pix_driver::ChassisWheelAngleFb msg;
        msg.header.stamp = ros::Time::now();

        // 车轮转角 (12位有符号，分辨率0.1度)
        int16_t angle_lf = PixDriver::readInt(data, 0, 12);
        int16_t angle_rf = PixDriver::readInt(data, 16, 12);
        int16_t angle_lr = PixDriver::readInt(data, 32, 12);
        int16_t angle_rr = PixDriver::readInt(data, 48, 12);

        msg.wheel_angle_lf = angle_lf * 0.1f;
        msg.wheel_angle_rf = angle_rf * 0.1f;
        msg.wheel_angle_lr = angle_lr * 0.1f;
        msg.wheel_angle_rr = angle_rr * 0.1f;

        wheel_angle_pub_.publish(msg);

        // if (this->_debug_mode) {
        //     ROS_INFO_THROTTLE(1, "[can driver] 0x541 Wheel Angle: wheel_angle_lf=%d, wheel_angle_rf=%d, wheel_angle_lr=%d, wheel_angle_rr=%d",
        //         angle_lf,
        //         angle_rf,
        //         angle_lr,
        //         angle_rr
        //     );
        // }
    }

    // 处理车轮扭矩反馈 (0x542)
    void processWheelTorque(const uint8_t* data) {
        pix_driver::ChassisWheelTorqueFb msg;
        msg.header.stamp = ros::Time::now();

        // 车轮扭矩 (16位有符号，分辨率0.1 Nm)
        int16_t torque_lf = PixDriver::readInt(data, 0, 16);
        int16_t torque_rf = PixDriver::readInt(data, 16, 16);
        int16_t torque_lr = PixDriver::readInt(data, 32, 16);
        int16_t torque_rr = PixDriver::readInt(data, 48, 16);

        msg.wheel_torque_lf = torque_lf * 0.1f;
        msg.wheel_torque_rf = torque_rf * 0.1f;
        msg.wheel_torque_lr = torque_lr * 0.1f;
        msg.wheel_torque_rr = torque_rr * 0.1f;

        wheel_torque_pub_.publish(msg);

        // if (this->_debug_mode) {
        //     ROS_INFO_THROTTLE(1, "[can driver] 0x542 Wheel Torque: wheel_torque_lf=%d, wheel_torque_rf=%d, wheel_torque_lr=%d, wheel_torque_rr=%d",
        //         torque_lf,
        //         torque_rf,
        //         torque_lr,
        //         torque_rr
        //     );
        // }
    }

    /**************************
     *         功能函数
     ***************************/

    // 驱动控制报文 (0x130)
    can_frame createDriveCtrlFrame(const can_msgs::ecu& ecu_msg) {
        can_frame frame;
        frame.can_id = 0x130;
        frame.can_dlc = 8;
        memset(frame.data, 0, 8);

        float speed = std::max(0.0f, std::min(ecu_msg.motor, 50.0f));  // 限制速度范围 0-50 m/s
        uint16_t speed_ctrl = static_cast<uint16_t>(speed / 0.01f);    // 速度控制 (16位无符号，分辨率0.01 m/s)
        uint8_t target_gear = 0;


        // 驱动使能
        PixDriver::writeBit(frame.data, 0, 1);

        // 驱动模式控制 - 默认为速度控制模式(0)
        uint8_t drive_mode = 0; // 0:speed ctrl mode  1:throttle ctrl mode
        PixDriver::writeUInt(frame.data, 2, 2, drive_mode);

        // 档位控制
        if ((this->current_gear_ == pix_driver::DriveStatusFb::GearStatusD && ecu_msg.shift == ecu_msg.SHIFT_R) or (this->current_gear_ == pix_driver::DriveStatusFb::GearStatusR && ecu_msg.shift == ecu_msg.SHIFT_D)) {
            // 跨挡位换挡  --禁止执行  --减速停车
            if (fabs(this->current_speed_) < 0.01) {
                target_gear = pix_driver::DriveStatusFb::GearStatusN;
            }else{
                target_gear = this->current_gear_;
            }
            speed_ctrl = 0;
        }else{
            switch(ecu_msg.shift) {
                case can_msgs::ecu::SHIFT_D: target_gear = pix_driver::DriveStatusFb::GearStatusD; break;
                case can_msgs::ecu::SHIFT_N: target_gear = pix_driver::DriveStatusFb::GearStatusN; break;
                case can_msgs::ecu::SHIFT_R: target_gear = pix_driver::DriveStatusFb::GearStatusR; break;
                default: target_gear = pix_driver::DriveStatusFb::GearStatusD; // 默认为N档
            }
        }
        PixDriver::writeUInt(frame.data, 4, 2, target_gear);

        // 四驱两驱模式 - 0:4wd 1:Rwd 2:Fwd  --默认为4wd
        PixDriver::writeUInt(frame.data, 6, 2, 0);

        // 当不一致时，不设置速度
        if (this->current_gear_ != ecu_msg.shift) {
            speed_ctrl = 0;
        }
        PixDriver::writeUInt(frame.data, 8, 16, speed_ctrl);

        // 循环计数 (4位，0-15循环)
        static uint8_t drive_life = 0;
        PixDriver::writeUInt(frame.data, 48, 4, drive_life);
        drive_life = (drive_life + 1) % 16;

        // 校验和 (8位，byte0 xor byte1 xor...byte6)
        uint8_t checksum = 0;
        for(int i = 0; i < 7; i++) {
            checksum ^= frame.data[i];
        }
        PixDriver::writeUInt(frame.data, 56, 8, checksum);

        // ROS_WARN("[can driver] 0x131: %02x %02x %02x %02x %02x %02x %02x %02x", frame.data[0], frame.data[1], frame.data[2], frame.data[3], frame.data[4], frame.data[5], frame.data[6], frame.data[7]);

        return frame;
    }

    // 制动控制报文 (0x131)
    can_frame createBrakeCtrlFrame(const can_msgs::ecu& ecu_msg) {
        can_frame frame;
        frame.can_id = 0x131;
        frame.can_dlc = 8;
        memset(frame.data, 0, 8);

        // 制动使能 (紧急停车或速度控制时使能)  --TODO::常规状态下是否使能制动?
        bool brake_enable = true;
        PixDriver::writeBit(frame.data, 0, brake_enable);

        // 刹车灯控制 (紧急停车时开启)
        PixDriver::writeBit(frame.data, 1, ecu_msg.brake);

        // AEB使能 (预留，设为0)
        PixDriver::writeBit(frame.data, 4, 0);


        // 刹车控制 (10位无符号，分辨率0.1%)
        // 紧急停车时使用最大制动力，否则根据速度比率计算
        uint16_t brake_ctrl = uint16_t(1000*ecu_msg.brake);
        brake_ctrl = std::min(brake_ctrl, static_cast<uint16_t>(1000)); // 限制最大值
        // 驻车且无速度状态下可以不开启刹车
        if (this->current_p_status_ == pix_driver::BrakeStatusFb::EpbStatusBrake && fabs(this->current_speed_) < 0.1f) {
            brake_ctrl = 0;
        }
        PixDriver::writeUInt(frame.data, 8, 10, brake_ctrl);

        // 驻车控制
        // 紧急停车或N档时驻车，D/R档时释放
        uint8_t epb_ctrl = 0; // default
        if(ecu_msg.shift == can_msgs::ecu::SHIFT_N) {
            epb_ctrl = 1; // brake
        } else if(ecu_msg.shift == can_msgs::ecu::SHIFT_D || ecu_msg.shift == can_msgs::ecu::SHIFT_R) {
            epb_ctrl = 2; // release
        }
        PixDriver::writeUInt(frame.data, 24, 2, epb_ctrl);

        // 循环计数 (4位，0-15循环)
        static uint8_t brake_life = 0;
        PixDriver::writeUInt(frame.data, 48, 4, brake_life);
        brake_life = (brake_life + 1) % 16;

        // 校验和 (8位，byte0 xor byte1 xor...byte6)
        uint8_t checksum = 0;
        for(int i = 0; i < 7; i++) {
            checksum ^= frame.data[i];
        }
        PixDriver::writeUInt(frame.data, 56, 8, checksum);

        return frame;
    }

    // 转向控制报文 (0x132)
    can_frame createSteerCtrlFrame(const can_msgs::ecu& ecu_msg) {
        can_frame frame;
        frame.can_id = 0x132;
        frame.can_dlc = 8;
        memset(frame.data, 0, 8);

        // 转向使能
        PixDriver::writeBit(frame.data, 0, 1);

        // 转向模式控制 - 默认为前轮转向(0)
        /*
            0:front ackerman
            1:same front and back
            2:front different back
            3:back ackrman
            4:front back
        */
        PixDriver::writeUInt(frame.data, 4, 4, 0);

        // 转向角度控制 (16位有符号，分辨率1度，范围-500~500)
        int16_t steer_angle = clamp<double>(500*ecu_msg.steer/max_steer_angle, -500, 500);
        PixDriver::writeInt(frame.data, 8, 16, steer_angle);   // 前转向角度控制
        // PixDriver::writeInt(frame.data, 24, 16, steer_angle);  // 后转向角度控制

        // 转向角速度控制 (8位无符号，分辨率2 deg/s，范围0-500 deg/s)
        // 使用固定值100 deg/s
        uint8_t steer_speed = 50; // 100 deg/s
        PixDriver::writeUInt(frame.data, 40, 8, steer_speed);

        // 循环计数 (4位，0-15循环)
        static uint8_t steer_life = 0;
        PixDriver::writeUInt(frame.data,48, 4, steer_life);
        steer_life = (steer_life + 1) % 16;

        // 校验和 (8位，byte0 xor byte1 xor...byte6)
        uint8_t checksum = 0;
        for(int i = 0; i < 7; i++) {
            checksum ^= frame.data[i];
        }
        PixDriver::writeUInt(frame.data, 56, 8, checksum);

        return frame;
    }

    // 车身控制报文 (0x133)
    can_frame createVehicleCtrlFrame(const can_msgs::ecu& ecu_msg) {
        can_frame frame;
        frame.can_id = 0x133;
        frame.can_dlc = 8;
        memset(frame.data, 0, 8);

        // 灯光控制
        PixDriver::writeBit(frame.data, 0, ecu_msg.position_light);  // 位置灯
        PixDriver::writeBit(frame.data, 1, ecu_msg.low_beam);        // 近光灯
        PixDriver::writeBit(frame.data, 2, ecu_msg.turn_left_light); // 左转向灯
        PixDriver::writeBit(frame.data, 3, ecu_msg.turn_right_light); // 右转向灯
        PixDriver::writeBit(frame.data, 4, ecu_msg.high_beam);       // 远光灯
        PixDriver::writeBit(frame.data, 5, 0);                       // 雾灯(预留)
        PixDriver::writeBit(frame.data, 6, 0);                       // 氛围灯(预留)
        PixDriver::writeBit(frame.data, 7, ecu_msg.rear_front_light); // 车内灯光

        // 限速控制 (1位)
        bool speed_limit_enable = false; // 默认不限速 0:不限速  1:限速
        PixDriver::writeBit(frame.data, 24, speed_limit_enable);

        // // 速度限制值 (8位无符号，分辨率0.1 m/s，范围1-20 m/s)
        // uint8_t speed_limit_val = 200; // 20 m/s
        // PixDriver::writeUInt(frame.data, 32, 8, speed_limit_val);

        // 校验使能 (1位，预留)  0:校验  1:不校验
        PixDriver::writeBit(frame.data, 48, 0);

        return frame;
    }

    // // 电机扭矩控制报文 (0x135) - 预留功能
    // can_frame createWheelTorqueCtrlFrame(const can_msgs::ecu& ecu_msg) {
    //     can_frame frame;
    //     frame.can_id = 0x135;
    //     frame.can_dlc = 8;
    //     memset(frame.data, 0, 8);

    //     // 左前电机扭矩 (16位有符号，分辨率0.1 Nm，范围-200~200 Nm)
    //     int16_t torque_lf = 0;
    //     PixDriver::writeInt(frame.data, 0, 16, torque_lf);

    //     // 右前电机扭矩 (16位有符号，分辨率0.1 Nm，范围-200~200 Nm)
    //     int16_t torque_rf = 0;
    //     PixDriver::writeInt(frame.data, 16, 16, torque_rf);

    //     // 左后电机扭矩 (16位有符号，分辨率0.1 Nm，范围-200~200 Nm)
    //     int16_t torque_lr = 0;
    //     PixDriver::writeInt(frame.data, 32, 16, torque_lr);

    //     // 右后电机扭矩 (16位有符号，分辨率0.1 Nm，范围-200~200 Nm)
    //     int16_t torque_rr = 0;
    //     PixDriver::writeInt(frame.data, 48, 16, torque_rr);

    //     return frame;
    // }

private:
    ros::NodeHandle nh_, pnh_;
    ros::Subscriber ecu_sub_;
    ros::Timer timer_, timer_pub_status_;
    bool _debug_mode = false;

    // 所有发布者
    ros::Publisher drive_sta_pub_;
    ros::Publisher brake_sta_pub_;
    ros::Publisher steer_sta_pub_;
    ros::Publisher vehicle_work_sta_pub_;
    ros::Publisher power_sta_pub_;
    ros::Publisher vehicle_sta_pub_;
    ros::Publisher vehicle_flt_sta_pub_;
    ros::Publisher wheel_rpm_pub_;
    ros::Publisher odo_pub_;
    ros::Publisher tire_press_pub_;
    ros::Publisher wheel_angle_pub_;
    ros::Publisher wheel_torque_pub_;

    boost::asio::io_context io_context_;
    udp::socket socket_;
    udp::socket receiver_socket_;
    udp::endpoint target_endpoint_;
    std::thread receiver_thread_;

    uint8_t current_gear_ = pix_driver::DriveStatusFb::GearStatusNoUse;
    uint8_t current_p_status_ = -1;  // pix_driver::BrakeStatusFb epb_status
    double current_speed_;  // 当前速度

    std::mutex mutex_;
    can_msgs::ecu last_ecu_;
    ros::Time last_cmd_time_;
};

int main(int argc, char** argv) {
    setlocale(LC_ALL, "zh_CN.UTF-8");
    ros::init(argc, argv, "can_bridge_node");
    ros::NodeHandle nh("~");
    CANBridgeNode node(nh);
    ros::spin();
    return 0;
}