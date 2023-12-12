/*********************************************************************
 * Copyright (c) 2019, SoftBank Corp.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************/

#include "vesc_hw_interface/roamer_vesc_hw_interface.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "xr1_hw_interface_node");

  ros::NodeHandle nh, nh_private("~");
  vesc_hw_interface::XR1VescHwInterface xr1_vesc_hw_interface;
  xr1_vesc_hw_interface.init(nh, nh_private);

  controller_manager::ControllerManager controller_manager(&xr1_vesc_hw_interface, nh);

  double update_rate = 100;
  double update_duration = 1.0 / update_rate;
  ros::Rate loop_rate(update_rate);
  ros::AsyncSpinner spinner(1);

  spinner.start();

  while (ros::ok())
  {
    // sends commands
    xr1_vesc_hw_interface.write(xr1_vesc_hw_interface.getTime(), ros::Duration(update_duration));

    // updates the hardware interface control
    controller_manager.update(xr1_vesc_hw_interface.getTime(), ros::Duration(update_duration));

    // gets current states
    xr1_vesc_hw_interface.read(xr1_vesc_hw_interface.getTime(), ros::Duration(update_duration));

    // sleeps
    loop_rate.sleep();
  }

  spinner.stop();

  return 0;
}
