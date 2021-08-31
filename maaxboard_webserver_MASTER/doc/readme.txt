Overview
========
This program utilizes m7, m4 cores to demonstrate headless webserver on the i.mxrt1170 MCU.
Application program is partitioned in following way:

1. webserver. (m7)
    1. LWIP stack based httpserver serving wifi, led, sensor pages.
    2. wifi softAP or client
    3. Hyperflash as configuration storage    
2. sensor acquisition using i2c. (m4)
    1. [6DOF IMU 3 CLICK](https://www.mikroe.com/6dof-imu-3-click)
    2. [LightRanger 8 CLICK](https://www.mikroe.com/lightranger-8-click)

M7 core project runs Freertos with memory scheme 3. It has 3 running tasks. 
	wifi_task: for initialize wifi connectivity.
	http_srv_task:  httpserver based on lwip stack.
	app_task: for communicating with m4 core, to retrieve sensor values.
   
M4 core project runs Freertos with memory scheme 4. It has 3 running tasks
	IMU_TASK: for polling imu sensor every 10ms;
	LR_TASK: for polling lightranger sensor every 500ms;
	MC_TASK: sending sensor values to M7 core every 200ms through shared memory.

Shared memory usage
This multicore example uses the shared memory section "rpmsg_sh_mem" for data exchange. 
This memory section starts from 0x202c0000 with size of (0x2000).
The shared memory region is defined and the size can be adjustable in the linker file. The shared memory region start address
and the size have to be defined in linker file for each core equally. The shared memory start
address is then exported from the linker to the application.


Hardware requirements
=====================
- Micro USB cable
- MaaXBoard RT board
- 6DOF IMU 3 click board
- LightRanger 8 click board
- pi2 click shield or pi3 click shield
- Personal Computer


Board settings
==============
No special settings are required.

Prepare the Demo
================
1.  Connect a USB cable between the host PC and the OpenSDA USB port on the target board. 
2.  Open a serial terminal with the following settings:
    - 115200 baud rate
    - 8 data bits
    - No parity
    - One stop bit
    - No flow control
3.  Download the program to the target board.
4.  Either press the reset button on your board or launch the debugger in your IDE to begin running the demo.


Running the demo
================
The log below shows the output of the maaxboard_webserver_MASTER demo in the terminal window:
```````````````````````````````````````````
Starting MaaXBoard Webserver DEMO
[i] Trying to load data from mflash.
[i] Saved SSID: ssid, Password: password
[i] Initializing WiFi connection...
MAC Address: XX:XX:XX:XX:XX:XX
[net] Initialized TCP/IP networking stack
WLAN initialized
WLAN FW Version: w8987o-V0, RF878X, FP91, 16.91.10.p200, WPA2_CVE_FIX 1, PVE_FIX 1
[i] Successfully initialized WiFi module
Connecting as client to ssid: ssid with password password
        Connected to following BSS:SSID = [ssid], IP = [192.168.0.13]
[i] Connected to Wi-Fi
ssid: ssid
[!]passphrase: password
 Now join that network on your device and connect to this IP: 192.168.0.13
...
```````````````````````````````````````````

Note:
Lightranger click sensor must installed on slot#1 on click shield, imu click sensor on slot#2 due to physical pin routing.


