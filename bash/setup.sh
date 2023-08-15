#!/usr/bin/bash

echo "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  %%%%%%       %%%%%%%%%%%%%  
 %%%                      %%%%
 %%%                       %%%
  %%%%%%%%%%%%%           %%%%
     %%%%%%%%%%%%%   %%%%%%%  
                %%%   %%%%    
               %%%%     %%%    Space Robotics Laboratory
  %%%%%%%%%%%%%%%        %%%%  Kyushu Institute of Technology

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
 
 Climbing Robots Research Group
 NVDIA JETSON ORIN Set-Up for ROS2 2023

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" 

echo ""
echo "NOTE: This script needs to be run with sudo"
echo "Setting up can0 with 1000000 bitrate..."

# configure jetson orin gpio
busybox devmem 0x0c303000 w 0x400
busybox devmem 0x0c303008 w 0x458
busybox devmem 0x0c303010 w 0x400
busybox devmem 0x0c303018 w 0x458

# load drivers
modprobe can
modprobe can_raw
modprobe mttcan

# set up can bus
ifconfig can0 down
ip link set can0 type can bitrate 1000000
ifconfig can0 txqueuelen 1000
ifconfig can0 up

echo "Setting up serial port ttyTHS0..."

# configure serial port
fuser -k /dev/ttyTHS0 # avoid tty is busy
chmod a+rw /dev/ttyTHS0 # avoid permission denied
chmod 666 /dev/ttyTHS0 # if tty is blocked

echo "Setting USBFS to 1000 to prevent image losses with pylon and USB cameras..."

# configure usbfs_memory_mb
echo 1000 > /sys/module/usbcore/parameters/usbfs_memory_mb
cat /sys/module/usbcore/parameters/usbfs_memory_mb

echo "Setup up finished successfully"
