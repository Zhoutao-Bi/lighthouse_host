cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(lighthouse_20260714)

target_sources(app PRIVATE
	src/main.c
	src/led/led.c
	src/bt_advertise/bt_advertise.c
	src/bt_scan/bt_scan.c
	src/ppi/ppi.c
	src/ppi/timer.c
	src/ts4231/ts4231.c
	src/ts4231/ts4231_sensors.c
	src/pos/pos.c
	src/robot_pose/robot_pose.c
	src/uart_out/uart_out.c
)

target_include_directories(app PRIVATE
	src/led
	src/bt_advertise
	src/bt_scan
	src/ppi
	src/ts4231
	src/pos
	src/robot_pose
	src/uart_out
)
