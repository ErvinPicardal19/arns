// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "arns_base/arns_hardware.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace arns_base
{
   
   ARNSystemHardware::ARNSystemHardware() {
      RCLCPP_INFO(rclcpp::get_logger("ARNSystemHardware"), "ARNSystemHardware created.");
   }

   hardware_interface::CallbackReturn ARNSystemHardware::on_init(const hardware_interface::HardwareInfo & info)
   {
      RCLCPP_INFO(rclcpp::get_logger("ARNSystemHardware"), "Activating ...please wait...");
      if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
      {
         return hardware_interface::CallbackReturn::ERROR;
      }

      // get parameters from ros2_control
      this->cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
      this->cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
      this->cfg_.loop_rate= hardware_interface::stod(info_.hardware_parameters["loop_rate"]);
      this->cfg_.device = info_.hardware_parameters["device"];
      this->cfg_.baud_rate= std::stoi(info_.hardware_parameters["baud_rate"]);
      this->cfg_.timeout_ms= std::stoi(info_.hardware_parameters["timeout_ms"]);
      this->cfg_.enc_counts_per_rev= std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);

      // setup wheels
      this->l_wheel_.setup(this->cfg_.left_wheel_name, this->cfg_.enc_counts_per_rev);
      this->r_wheel_.setup(this->cfg_.right_wheel_name, this->cfg_.enc_counts_per_rev);

      // check joints command and state interface from parameters
      for (const hardware_interface::ComponentInfo &joint : info_.joints)
      {
         // DiffBotSystem has exactly two states and one command interface on each joint
         if (joint.command_interfaces.size() != 1)
         {
            RCLCPP_FATAL(
               rclcpp::get_logger("ARNSystemHardware"),
               "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
               joint.command_interfaces.size());

            return hardware_interface::CallbackReturn::ERROR;
         }

         if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
         {
            RCLCPP_FATAL(
               rclcpp::get_logger("ARNSystemHardware"),
               "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
               joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);

            return hardware_interface::CallbackReturn::ERROR;
         }

         if (joint.state_interfaces.size() != 2)
         {
            RCLCPP_FATAL(
               rclcpp::get_logger("ARNSystemHardware"),
               "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
               joint.state_interfaces.size());

            return hardware_interface::CallbackReturn::ERROR;
         }

         if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
         {
            RCLCPP_FATAL(
               rclcpp::get_logger("ARNSystemHardware"),
               "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
               joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);

            return hardware_interface::CallbackReturn::ERROR;
         }

         if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
         {
            RCLCPP_FATAL(
               rclcpp::get_logger("ARNSystemHardware"),
               "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
               joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
         }
      }

      return hardware_interface::CallbackReturn::SUCCESS;
   }

   std::vector<hardware_interface::StateInterface> ARNSystemHardware::export_state_interfaces()
   {
      std::vector<hardware_interface::StateInterface> state_interfaces;

      // left wheel state interfaces
      state_interfaces.emplace_back(hardware_interface::StateInterface(this->l_wheel_.name, hardware_interface::HW_IF_POSITION, &this->l_wheel_.pos));
      state_interfaces.emplace_back(hardware_interface::StateInterface(this->l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &this->l_wheel_.vel));
      
      // right wheel state interfaces
      state_interfaces.emplace_back(hardware_interface::StateInterface(this->r_wheel_.name, hardware_interface::HW_IF_POSITION, &this->r_wheel_.pos));
      state_interfaces.emplace_back(hardware_interface::StateInterface(this->r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &this->r_wheel_.vel));

      return state_interfaces;
   }

   std::vector<hardware_interface::CommandInterface> ARNSystemHardware::export_command_interfaces()
   {
      std::vector<hardware_interface::CommandInterface> command_interfaces;

      // left and right wheel command interfaces
      command_interfaces.emplace_back(hardware_interface::CommandInterface(this->l_wheel_.name, hardware_interface::HW_IF_VELOCITY, &this->l_wheel_.cmd));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(this->r_wheel_.name, hardware_interface::HW_IF_VELOCITY, &this->r_wheel_.cmd));

      return command_interfaces;
   }

   hardware_interface::CallbackReturn ARNSystemHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
   {

      RCLCPP_INFO(rclcpp::get_logger("ARNSystemHardware"), "Activating ...please wait...");

      comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);

      RCLCPP_INFO(rclcpp::get_logger("ARNSystemHardware"), "Successfully activated!");

      return hardware_interface::CallbackReturn::SUCCESS;
   }

   hardware_interface::CallbackReturn ARNSystemHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
   {

      RCLCPP_INFO(rclcpp::get_logger("ARNSystemHardware"), "Deactivating ...please wait...");

      comms_.disconnect();

      RCLCPP_INFO(rclcpp::get_logger("ARNSystemHardware"), "Successfully deactivated!");

      return hardware_interface::CallbackReturn::SUCCESS;
   }

   hardware_interface::return_type ARNSystemHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
   {

      // add check comms_ later on...

      comms_.read_encoder_values(this->l_wheel_.enc, this->r_wheel_.enc);

      double time_delta = period.seconds();

      double l_wheel_prev_pos = this->l_wheel_.pos;
      this->l_wheel_.pos = this->l_wheel_.calc_enc_angle();
      this->l_wheel_.vel = (this->l_wheel_.pos - l_wheel_prev_pos) / time_delta;
      
      double r_wheel_prev_pos = this->r_wheel_.pos;
      this->r_wheel_.pos = this->r_wheel_.calc_enc_angle();
      this->r_wheel_.vel = (this->r_wheel_.pos - r_wheel_prev_pos) / time_delta;

      return hardware_interface::return_type::OK;
   }

   hardware_interface::return_type arns_base::ARNSystemHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
   {

      int motor_l_counts_per_loop = this->l_wheel_.cmd / this->l_wheel_.rads_per_count / this->cfg_.loop_rate; 

      int motor_r_counts_per_loop = this->r_wheel_.cmd / this->r_wheel_.rads_per_count / this->cfg_.loop_rate;

      // add check comms_ later on...

      comms_.set_motor_values(motor_l_counts_per_loop, motor_r_counts_per_loop);

      return hardware_interface::return_type::OK;
   }

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  arns_base::ARNSystemHardware, hardware_interface::SystemInterface)