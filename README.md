# ESP32-Security-System using ESP32 Mail Client Library 

Project description:

The ESP32CAM continuously looks for the door status. If the door is closed, then the ESP32CAM goes into the sleep mode. When the door is opened, ESP wakes up from sleep mode, connects to the Wi-Fi, goes through the camera initializations, and keeps on looking for the motion. If the motion is detected, it then takes a picture and sends it to the entered email address. Otherwise, if the door is closed, the ESP32CAM will go back into sleep mode.

This repository contains two versions of the code one, containing the door sesnor and one without it.

Hardware:

![image](https://user-images.githubusercontent.com/60723743/205772569-7d729052-f0f1-491a-98b1-cc2c40e70f10.png)
  - LILYGO: ESP32 T- Camera 
  - Door Sensor
Door Sensor

Software details:

Install the following libraries for the code to work properly:
  - ESP32 mail client library
  - Blynk and Blynk_Async_ESP32_BT_WF libraries
  - Blynk_Async_GSM_Manager
  - Make a mobile hotspot on laptop or mobile and set its name and password. After that place that    hotspot name and password in the labeled lines of code. 
  - Replace recipient email if needed in the labeled lines of code.
