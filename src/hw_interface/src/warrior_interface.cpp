
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <string>

#include "hw_interface/warrior_hardware.hpp"
#include "hw_interface/can_driver.hpp"
PLUGINLIB_EXPORT_CLASS(
    debict::mecanumbot::hardware::WarriorbotHardware,
    hardware_interface::SystemInterface
)

VCI_BOARD_INFO pInfo;//用来获取设备信息。
int count=0;//数据列表中，用来存储列表序号。
VCI_BOARD_INFO pInfo1 [50];
int num=0;

int reclen=0;
VCI_CAN_OBJ rec[3000];//接收缓存，设为3000为佳。

int q1,w1;
int ind=0;

using namespace debict::mecanumbot::hardware;

hardware_interface::return_type WarriorbotHardware::configure(const hardware_interface::HardwareInfo & system_info)
{
    if (configure_default(system_info) != hardware_interface::return_type::OK) {
        return hardware_interface::return_type::ERROR;
    }
    serial_port_name_ = info_.hardware_parameters["serial_port"];
    RCLCPP_INFO(rclcpp::get_logger("WarriorbotHardware"), "Motor id not defined for join %s", serial_port_name_);
    motor_ids_.resize(info_.joints.size());
    imu_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    position_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    velocity_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    velocity_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    velocity_commands_saved_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    for (hardware_interface::ComponentInfo & joint : info_.joints)
    {
        if (joint.parameters["motor_id"].empty()) {
            RCLCPP_FATAL(rclcpp::get_logger("WarriorbotHardware"), "Motor id not defined for join %s", joint.name.c_str());
            return hardware_interface::return_type::ERROR;
        }
        if (joint.command_interfaces.size() != 1) {
            RCLCPP_FATAL(rclcpp::get_logger("WarriorbotHardware"), "Invalid number of command interfaces (expected: 1)");
            return hardware_interface::return_type::ERROR;
        }
        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(rclcpp::get_logger("WarriorbotHardware"), "Invalid joint command interface 0 type (expected: velocity)");
            return hardware_interface::return_type::ERROR;
        }

        if (joint.state_interfaces.size() != 2) {
            RCLCPP_FATAL(rclcpp::get_logger("WarriorbotHardware"), "Invalid number of state interfaces (expected: 2)");
            return hardware_interface::return_type::ERROR;
        }
        if (joint.state_interfaces[0].name != HW_IF_ANGLE) {
            RCLCPP_FATAL(rclcpp::get_logger("WarriorbotHardware"), "Invalid joint state interface 0 type (expected: position)");
            return hardware_interface::return_type::ERROR;
        }
        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
            RCLCPP_FATAL(rclcpp::get_logger("WarriorbotHardware"), "Invalid joint state interface 1 type (expected: velocity)");
            return hardware_interface::return_type::ERROR;
        }
    }

    for (size_t i = 0; i < info_.joints.size(); i++) {
        motor_ids_[i] = (uint8_t)std::stoi(info_.joints[i].parameters["motor_id"]);
        RCLCPP_INFO(rclcpp::get_logger("WarriorbotHardware"), "%s mapped to motor %d", info_.joints[i].name.c_str(), motor_ids_[i]);
    }
    
    status_ = hardware_interface::status::CONFIGURED;
    return hardware_interface::return_type::OK;
}
/* read buffer -> strcut imu_ -> imu_ -> state_interface*/
std::vector<hardware_interface::StateInterface> WarriorbotHardware::export_state_interfaces()
{
    imu_[PITCH] = 1;
    imu_[YAW] = 2;
    imu_[ROLL] = 3;
    RCLCPP_INFO(rclcpp::get_logger("WarriorbotHardware"), "export_state_interfaces");
    /* RM get date from the interface */
    std::vector<hardware_interface::StateInterface> state_interfaces;
    RCLCPP_INFO(rclcpp::get_logger("WarriorbotHardware"), "Adding angle pitch state interface: %s", info_.joints[0].name.c_str());
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[0].name, HW_IF_ANGLE, &imu_[PITCH]));
    RCLCPP_INFO(rclcpp::get_logger("WarriorbotHardware"), "Adding angle roll state interface: %s", info_.joints[1].name.c_str());
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[1].name, HW_IF_ANGLE, &imu_[YAW]));
    RCLCPP_INFO(rclcpp::get_logger("WarriorbotHardware"), "Adding angle yaw state interface: %s", info_.joints[2].name.c_str());
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[2].name, HW_IF_ANGLE, &imu_[ROLL]));
    RCLCPP_INFO(rclcpp::get_logger("WarriorbotHardware"), "Adding angle pitch state interface: %s", info_.joints[3].name.c_str());
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[3].name, HW_IF_ANGLE, &imu_[PITCH]));
    
    /*
    for (size_t i = 0; i < info_.joints.size(); i++) 
    {
        RCLCPP_INFO(rclcpp::get_logger("WarriorbotHardware"), "Adding position state interface: %s", info_.joints[i].name.c_str());
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]
            )
        );
    }
    for (size_t i = 0; i < info_.joints.size(); i++) 
    {
        RCLCPP_INFO(rclcpp::get_logger("WarriorbotHardware"), "Adding velocity state interface: %s", info_.joints[i].name.c_str());
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]
            )
        );
    }
    */

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> WarriorbotHardware::export_command_interfaces()
{
    RCLCPP_INFO(rclcpp::get_logger("WarriorbotHardware"), "export_command_interfaces");

    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++) {
        RCLCPP_INFO(rclcpp::get_logger("WarriorbotHardware"), "Adding velocity command interface: %s", info_.joints[i].name.c_str());
        command_interfaces.emplace_back(
            hardware_interface::CommandInterface(
                info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]
            )
        );
    }
    return command_interfaces;
}

hardware_interface::return_type WarriorbotHardware::start()
{
    RCLCPP_INFO(rclcpp::get_logger("WarriorbotHardware"), "Mecanumbot hardware starting ...");

    /*查找陀螺仪*/
    num=VCI_FindUsbDevice2(pInfo1);
    RCLCPP_INFO(rclcpp::get_logger("WarriorbotHardware"), ">>USBCAN DEVICE NUM:%d...",num);  

    for (size_t i = 0; i < info_.joints.size(); i++) {
        if (std::isnan(position_states_[i])) {
            position_states_[i] = 0.0f;
        }
        if (std::isnan(velocity_states_[i])) {
            velocity_states_[i] = 0.0f;
        }
        if (std::isnan(velocity_commands_[i])) {
            velocity_commands_[i] = 0.0f;
        }
        velocity_commands_saved_[i] = velocity_commands_[i];
    }

     serial_port_ = std::make_shared<MecanumbotSerialPort>();//初始化串口对象

     if (serial_port_->open(serial_port_name_) != return_type::SUCCESS) {
         RCLCPP_INFO(rclcpp::get_logger("WarriorbotHardware"), "Mecanumbot hardware failed to open serial port");
         return hardware_interface::return_type::ERROR;
     }
        if(VCI_OpenDevice(VCI_USBCAN2,0,0)!=1)//打开设备
        {
                RCLCPP_INFO(rclcpp::get_logger("WarriorbotHardware"), "failed to open can port");   
                return hardware_interface::return_type::ERROR;         
        }
        if(VCI_ReadBoardInfo(VCI_USBCAN2,0,&pInfo)==1)//读取设备序列号、版本等信息。
        {
            RCLCPP_INFO(rclcpp::get_logger("WarriorbotHardware"), ">>Get VCI_ReadBoardInfo success!\n");  
            RCLCPP_INFO(rclcpp::get_logger("WarriorbotHardware"), ">>Serial_Num:%c\n", pInfo.str_Serial_Num[0]); 
        }
        else
        {
                printf(">>Get VCI_ReadBoardInfo error!\n");
                return hardware_interface::return_type::ERROR;
        }
                //初始化参数，严格参数二次开发函数库说明书。
        VCI_INIT_CONFIG config;
        config.AccCode=0;
        config.AccMask=0xFFFFFFFF;
        config.Filter=1;//接收所有帧
        config.Timing0=0x00;/*波特率1000 Kbps  0x00  0x14*/
        config.Timing1=0x14;
        config.Mode=0;//正常模式

        if(VCI_InitCAN(VCI_USBCAN2,0,0,&config) != 1)
        {
            RCLCPP_INFO(rclcpp::get_logger("WarriorbotHardware"), ">>Init CAN1 error\n\n"); 
            VCI_CloseDevice(VCI_USBCAN2,0);
        }

        if(VCI_StartCAN(VCI_USBCAN2,0,0)!=1)
        {
                RCLCPP_INFO(rclcpp::get_logger("WarriorbotHardware"), ">>Start CAN1 error\n\n"); 
                VCI_CloseDevice(VCI_USBCAN2,0);
        }

    status_ = hardware_interface::status::STARTED;

    RCLCPP_INFO(rclcpp::get_logger("WarriorbotHardware"), "Mecanumbot hardware started");
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type WarriorbotHardware::stop()
{
    RCLCPP_INFO(rclcpp::get_logger("WarriorbotHardware"), "Mecanumbot hardware stopping ...");

    if (serial_port_->is_open()) {
        serial_port_->close();
        serial_port_.reset();
    }

    status_ = hardware_interface::status::STOPPED;

    RCLCPP_INFO(rclcpp::get_logger("WarriorbotHardware"), "Mecanumbot hardware stopped");
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type WarriorbotHardware::read()
{

    // We currently have an ack response, so read the frames
    std::vector<SerialHdlcFrame> frames;

    serial_port_->read_frames(frames);
    if((reclen=VCI_Receive(VCI_USBCAN2,0,ind,rec,3000,100))>0)//调用接收函数，如果有数据，进行数据处理显示。
    {
        for(q1=0;q1<reclen;q1++)
        {
            position_states_[0] = (rec[q1].Data[0]<<8)|rec[q1].Data[1];
            position_states_[1] = (rec[q1].Data[2]<<8)|rec[q1].Data[3];
            position_states_[2] = (rec[q1].Data[4]<<8)|rec[q1].Data[5];
            position_states_[3] = (rec[q1].Data[6]<<8)|rec[q1].Data[7];
            // RCLCPP_INFO(rclcpp::get_logger("WarriorbotHardware"), "Index:%04d  ",count);count++;
            // RCLCPP_INFO(rclcpp::get_logger("WarriorbotHardware"), "CAN%d RX ID:0x%08X", ind+1, rec[q1].ID);
            // if(rec[q1].ExternFlag==0) RCLCPP_INFO(rclcpp::get_logger("WarriorbotHardware")," Standard ");//帧格式：标准帧
            // if(rec[q1].ExternFlag==1) RCLCPP_INFO(rclcpp::get_logger("WarriorbotHardware")," Extend   ");//帧格式：扩展帧
            // if(rec[q1].RemoteFlag==0) RCLCPP_INFO(rclcpp::get_logger("WarriorbotHardware")," Data   ");//帧类型：数据帧
            // if(rec[q1].RemoteFlag==1) RCLCPP_INFO(rclcpp::get_logger("WarriorbotHardware")," Remote ");//帧类型：远程帧
            // RCLCPP_INFO(rclcpp::get_logger("WarriorbotHardware"),"DLC:0x%02X",rec[q1].DataLen);//帧长度
            // RCLCPP_INFO(rclcpp::get_logger("WarriorbotHardware")," data:0x");//帧长度
            // for(w1 = 0; w1 < rec[q1].DataLen; w1++)
            // {
            //     RCLCPP_INFO(rclcpp::get_logger("WarriorbotHardware")," %02X", rec[q1].Data[w1]);//帧长度
            // }
            // RCLCPP_INFO(rclcpp::get_logger("WarriorbotHardware")," TimeStamp:0x%08X",rec[q1].TimeStamp);//时间标识。
        }
    }
    ind=!ind;//变换通道号，以便下次读取另一通道，交替读取。	
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type WarriorbotHardware::write()
{
    for (size_t i = 0; i < info_.joints.size(); i++) {
        // Only send motor commands if the velocity changed
        if (velocity_commands_[i] != velocity_commands_saved_[i]) {

            RCLCPP_INFO(rclcpp::get_logger("WarriorbotHardware"), "Motor velocity changed: %.5f", velocity_commands_[i]);

            // Generate the motor command message
            uint16_t duty = 0;
            uint8_t message[6];
            message[0] = (uint8_t)DeviceCommand::MotorSetDuty;
            message[1] = 4; // Payload len
            message[2] = motor_ids_[i];
            if (velocity_commands_[i] >= 0.0) {
                duty = (uint16_t)(velocity_commands_[i]);
                message[3] = (uint8_t)DeviceMotorDirection::Forward;
            } else {
                duty = (uint16_t)(-velocity_commands_[i]);
                message[3] = (uint8_t)DeviceMotorDirection::Reverse;
            }
            message[4] = (uint8_t)(duty & 0xFF);
            message[5] = (uint8_t)((duty >> 8) & 0xFF);

            // Send the motor command
            serial_port_->write_frame(message, 6);

            // Store the current velocity
            velocity_commands_saved_[i] = velocity_commands_[i];
        }
    }
    return hardware_interface::return_type::OK;
}
