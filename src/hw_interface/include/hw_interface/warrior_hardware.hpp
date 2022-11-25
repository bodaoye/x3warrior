/*
 * @Author: JFG 73979026+bodaoye@users.noreply.github.com
 * @Date: 2022-11-18 01:22:35
 * @LastEditors: JFG 73979026+bodaoye@users.noreply.github.com
 * @LastEditTime: 2022-11-19 23:44:33
 * @FilePath: \mecanumbot_hardware\include\mecanumbot_hardware\mecanumbot_hardware.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#ifndef __DEBICT_WARRIORBOT_HARDWARE__WARRIORBOT_HARDWARE_H__
#define __DEBICT_WARRIORBOT_HARDWARE__WARRIORBOT_HARDWARE_H__

#include <hardware_interface/base_interface.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/macros.hpp>
#include <vector>

#include "hw_interface/mecanumbot_serial_port.hpp"
#include "hw_interface/mecanumbot_hardware_compiler.hpp"

namespace debict
{
    namespace mecanumbot
    {
        namespace hardware
        {
            constexpr char HW_IF_ANGLE[] = "angle";
            enum class DeviceCommand : uint8_t {
                MotorSetDuty = 0x01,
                MotorBrake   = 0x02,
                MotorStop    = 0x03,
            };

            enum class DeviceMotorDirection : uint8_t {
                None    = 0,
                Forward = 1,
                Reverse = 2,
            };
            enum
            {
                PITCH = 0,
                YAW,
                ROLL
            };
            typedef struct 
            {
                float pitch;
                float yaw;
                float roll;
            }imu_typedef;

            class WarriorbotHardware : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
            {
            public:
                RCLCPP_SHARED_PTR_DEFINITIONS(WarriorbotHardware)

                DEBICT_MECANUMBOT_HARDWARE_PUBLIC
                virtual hardware_interface::return_type configure(const hardware_interface::HardwareInfo & system_info) override;
                
                DEBICT_MECANUMBOT_HARDWARE_PUBLIC
                virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
                
                DEBICT_MECANUMBOT_HARDWARE_PUBLIC
                virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
                
                DEBICT_MECANUMBOT_HARDWARE_PUBLIC
                virtual hardware_interface::return_type start() override;
                
                DEBICT_MECANUMBOT_HARDWARE_PUBLIC
                virtual hardware_interface::return_type stop() override;
                
                DEBICT_MECANUMBOT_HARDWARE_PUBLIC
                virtual hardware_interface::return_type read() override;
                
                DEBICT_MECANUMBOT_HARDWARE_PUBLIC
                virtual hardware_interface::return_type write() override;

            private:
                std::vector<uint8_t> motor_ids_;
                std::vector<double> position_states_;
                std::vector<double> velocity_states_;
                std::vector<double> velocity_commands_;
                std::vector<double> velocity_commands_saved_;
                /*imu*/
                std::vector<double> imu_;
                imu_typedef imu_raw_data;
                std::shared_ptr<MecanumbotSerialPort> serial_port_;
                std::string serial_port_name_;

            };
        }
    }
}

#endif // __DEBICT_MECANUMBOT_HARDWARE__MECANUMBOT_HARDWARE_H__