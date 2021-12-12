# Q-Flight Quadrotor

#### Descirpition
Q-Flight is a fully functional quadrotor system which was based on STM32 F411 & FreeRTOS. It includes the flight controller, ground station, Unity simulations and 3D printing model...

see more in my webpage

https://quix.zqwei-tech.cn/


#### Q-Flight Overview

Q-Flight was the last project I've done in my high school times. It include the remote controller, flight control system based on STM32 F4, and related softwares... Click the video and see what Q-Flight got ! 

![quadrotor](https://quix.zqwei-tech.cn/static/qflight/3.png "quadrotor")

#### Flight Controller

For the Q-Flight controller, a range of sensors was used, for instance the barometer and GPS are combined for the navigation and position control in outdoor, while laser lidar and optical flow sensor was responsible for the indoor positioning. By using Mahony algorithm as the core for calculating the attitude of the quadrotor from accelerometer and gyroscope, thus the internal Strapdown Inertia Navigation System will gain sufficient infomation to control its altitude and position. FreeRTOS is also used to make the whole system run at a higher speed. In addition to the core control system, GNSS and GPRS are also added to send back flight data to the back-end server through socket to monitor the flight even in case of remote control failure. 

![quadrotor](https://quix.zqwei-tech.cn/static/qflight/2.png "quadrotor")

#### Remote Control

The Remote Control use implemented by using wireless USART through LORA protocols which has achieve over  2100km as its valid remote control distance through air. The Q-Flight remote communication protocol supports the control of the quadrotor, transporting the quadrotor's status, change values in Flash, customize PID controllers, transmitting large amount of information, sensor calibration, etc. The protocol received by quadrotor starts with '0xC8' and end with '0xC9' , while the protocol received by the ground staion was start with double  '0xC8' and end with double '0xC9'. The hardware and coding system was proved to be highly reliable during several tests in Q-Flight.

![quadrotor](https://quix.zqwei-tech.cn/static/qflight/1.png "quadrotor")

#### 3D Printing Model

The framework support the whole quadrotor was made from 3D printing and some steel stick to enhance some of the core part. This enables me manufacturing Q-Flight at a very lost cost. But the time it took to make one was too long and the hardness are not as good as the framework we can buy on the internet. It still need to be improved. 

![quadrotor](https://quix.zqwei-tech.cn/static/qflight/4.png "quadrotor")


#### Pyqt5 Based Ground Station Software

The ground station software provides a powerful tool to manage and control Q-Flight. It enables the developer to monitor the real-time status of Q-Flight directly and change the values for PID and other settings in Flash easily. It include the Opengl support the visualization of the attitude of Q-Flight and pyqt-graph to show the variation in diffierent status magnitudes. The GPS and GSM functionalities are currently developing now.

![quadrotor](https://quix.zqwei-tech.cn/static/qflight/ground_station.png "quadrotor")

#### Furture Plans

I will implement remote control through LTE 4G, try to use an extended Kalman Filter and other control algorithm by using a MCU supporting Linux. Also the PCB version of the flight controller is on its way...

![quadrotor](https://quix.zqwei-tech.cn/static/qflight/5.png "quadrotor")

![quadrotor](https://quix.zqwei-tech.cn/static/qflight/6.png "quadrotor")
![quadrotor](https://quix.zqwei-tech.cn/static/qflight/7.png "quadrotor")
![quadrotor](https://quix.zqwei-tech.cn/static/qflight/8.png "quadrotor")
![quadrotor](https://quix.zqwei-tech.cn/static/qflight/9.png "quadrotor")
![quadrotor](https://quix.zqwei-tech.cn/static/qflight/10.png "quadrotor")
![quadrotor](https://quix.zqwei-tech.cn/static/qflight/11.png "quadrotor")

