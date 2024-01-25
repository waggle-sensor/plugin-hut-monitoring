# Hut monitoring plugin
This plugin is desgined to communicate with AHT21 [Datasheet](http://www.aosong.com/en/products-60.html) for temperature and humidity and with MPU6050 [Datasheet](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/) for acceleration. The repository includes the Arduino code that can be flashed to an Arduino Mega to print out sensor readings via a Serial port. Then, this plugin reads the serial data to publish it to the cloud.

# Acknowledgement
[The Arduino code](arduino/main.ino) is written on top of the examples of the 2 sensors. It is shown in the head of the code, but we put a copy of it here to explicitly acknowledge, 

```cpp
/*
   Arduino and MPU6050 Accelerometer and Gyroscope Sensor Tutorial
   by Dejan, https://howtomechatronics.com
*/
...
```
