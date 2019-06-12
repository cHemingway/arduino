# Arduino

Low-level software for ROV. Takes JSON key:value pairs from [Raspberry Pi](https://github.com/ncl-ROVers/raspberry-pi) and acts on them accordingly.

## Installation

To install this code on an Arduino you first need to run the appropriate setup script (in arduino-setup ) to assign an ID to the Arduino. This ID dictates the functionality as follows:

* Ard_T: Control for Thrusters on main ROV body
* Ard_A: Control for Arm and other outputs on main ROV body
* Ard_M: Control for any Micro ROV devices
* Ard_I: Sending sensor data back up to surface

After running the setup script, flash arduino-main to the device.

## Expected behaviour

### Arduino T

This Arduino is for controlling the main thrusters.

Values in the range 1100 to 1900 will be accepted for Thruster control where 1100 is full reverse, 1500 is stopped, and 1900 is full forward.

Thrusters are given an ID which describes their position on the ROV.

| Pin | JSON ID | Description                                 |
|-----|---------|---------------------------------------------|
| 2   | Thr_FP  | Forward Port Thruster (front right)         |
| 3   | Thr_FS  | Forward Starboard Thruster (front left)     |
| 4   | Thr_AP  | Aft Port Thruster (back left)               |
| 5   | Thr_AS  | Aft Starboard Thruster (back right)         |
| 6   | Thr_TFP | Top Forward Port Thruster (front right)     |
| 7   | Thr_TFS | Top Forward Starboard Thruster (front left) |
| 8   | Thr_TAP | Top Aft Port Thruster (back left)           |
| 9   | Thr_TAS | Top Aft Starboard Thruster (back right)     |




