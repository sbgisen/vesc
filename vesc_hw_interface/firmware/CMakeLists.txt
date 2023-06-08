cmake_minimum_required(VERSION 3.0.2)

add_definitions(-DUSB_CON)

generate_arduino_firmware(
  serial_calibration
  SKETCH
  pm_t65w_p.ino
  BOARD
  nano328 # arduino nano old bootloader
  PORT
  /dev/ttyUSB0
)
