#### ArduPilot set

Solder weirs to R3, T3 and G on Ardupilot flight controller. 
![[Pasted image 20240511093117.png]]

Then, set following parameters by Mission Planner software: 

| Name             | Value    |
| ---------------- | -------- |
| SERIAL3_BAUD     | 57       |
| SERIAL3_OPTION   | 0        |
| SERIAL3_PROTOCOL | MAVLink2 |
| SERIAL_PASS2     | 3        |
#### OrangePi - UART 4
Connect weir (RX, TX, GND) from ArduPilot to UART4_RX_M0, UART4_TX_M0 and GND

![[Pasted image 20240511092958.png]]

Connect by SSH and type in terminal 
```
sudo orangepi-config
```

Then >  **System** >**Hardware** > scroll till *uart4-mo*, press <space> on it. Then Save. 

![[Pasted image 20240511095331.png]]


#### Changes in launch file
Open `apm.launch` and type path for UART4
```
<arg name="fcu_url" default="/dev/ttyS4:57600" />
```

To check, launch 
```
roslaunch racing_pkg apm.launch
```

And chec topic 
```
orangepi@orangepi5b:~$ rostopic echo /red/mavros/state
```

The message with data shout be here. 
```
header: 
  seq: 5
  stamp: 
    secs: 1715421288
    nsecs: 505271666
  frame_id: ''
connected: True
armed: False
guided: False
manual_input: True
mode: "STABILIZE"
system_status: 3
---
```