Sensor: Optical FLow 3901-L0X 
https://www.mateksys.com/?portfolio=3901-l0x

Important! 
**Make sure Optical Flow lens to ground >2cm for opflow initialization while FC starting up.**

==**Performed on nuc2**==
Flight controller parameters:

| RNGFND1_TYPE     | 32  |
| ---------------- | --- |
| SERIAL6_PROTOCOL | 23  |
| SERIAL6_BAUD     | 115 |

add to /opt/ros/noetic/share/mavros/launch/apm_pluginlists.yaml 

```
plugin_blacklist:
- distance_sensor
```

add to apm.launch
```
<!-- enable stream for flight controller sensors -->
<node pkg="rosservice" type="rosservice" name="rosservice" args="call /nuc/mavros/set_stream_rate 0 50 true" launch-prefix="bash -c 'sleep 10; $0 $@' " />
```

Range finder data will publish in: (message type sensor_msgs/Range)
`/nuc/mavros/rangefinder/rangefind`