#!/bin/bash

# fix_usbserial.sh
# Author: Dave Touretzky, 1/22/2012

# In Ubuntu 10.04, the usbserial driver crashes if a serial device is
# unplugged and plugged back in.  To fix this we must unload and
# reload the driver module.  For Tekkotsu robots this affects FTDI
# serial devices such as the iRobot USB-to-serial cable and the
# Dynamixel USB serial controllers, so the FTDI driver must also
# be reloaded.

usbserial_driver=`locate usbserial.ko | grep \`uname -r\``
ftdi_driver=`locate ftdi_sio.ko | grep \`uname -r\``

if [ -z "$usbserial_driver" ]; then
    echo "Could not find usbserial  driver!"
    exit 1
else
    echo Using $usbserial_driver
fi
if [ -z "$ftdi_driver" ]; then
    echo "Could not find ftdi_sio driver!"
    exit 1
else
    echo Using $ftdi_driver
fi

# Do this in a subshell to ignore error if module isn't loaded.
(sudo rmmod ftdi_sio usbserial)

# Insert modules one at a time because usbserial must be loaded first.
sudo insmod $usbserial_driver
sudo insmod $ftdi_driver
echo done
