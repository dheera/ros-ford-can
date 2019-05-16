# ROS Ford reader

This reads a number of variables potential useful for machine learning model training from a Ford car. Tested on a Focus 2015. Untested on other cars (please help).

Requires a proper CANbus interface (e.g. Jetson TX2, Xavier, or USB-CAN adapter) connected to HS-CAN on the OBD port. This will NOT work with ELM327 or other serial "AT command" devices.

## Parameters:

* **channel** (string) -- the CANbus interface to use. Defaults to can0. Refer to the python-can documentation.
* **bustype** (string) -- bus type. Defaults to socketcan\_native. Refer to the python-can documentation.

## Outputs topics:
* **accelerator\_fraction** (std\_msgs/Float32) -- 0.0 to 1.0, how much the accelerator pedal is pressed.
* **brake\_pressure** (std\_msgs/Float32) -- pressue applied to the brakes in hPa.
* **gps/fix** (sensor\_msgs/NavSatFix) -- GPS latitude and longitude. Not very accurate or useful for navigation purposes.
* **gps/heading** (std\_msgs/Float32) -- Heading in degrees as reported by the car. 0.0 is north.
* **ignition\_switch** (std\_msgs/Int8) -- ignition switch position. 0 = lock/off, 1 = key inside, 2 = accessory, 3 = on, 4 = start.
* **rpm** (std\_msgs/Float32) -- engine RPM.
* **speed** (std\_msgs/Float32) -- vehicle speed. Always positive regardless of forward or reverse motion.
* **steering\_wheel\_angle** (std\_msgs/Float32) -- steering wheel angle in degrees. Left is positive.
* **total\_distance** (std\_msgs/Float32) -- total distance travelled over all time by the car in km ("odometer"). Increments in steps of 1 km.

# Disclaimer

Use at your own risk. Only intended for recording data -- not for live deployment on autonomous vehicles. Not responsible for damage to your vehicle.
