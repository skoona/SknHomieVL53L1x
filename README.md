# SknHomieVL53L1x

<a href="https://homieiot.github.io/">
  <img src="https://homieiot.github.io/img/works-with-homie.png" alt="works with MQTT Homie">
</a>

HomieNode V3 (ESP32) featuring:
- VL53L1x Time of Flight Sensor capturing distance reading every 280 milliseconds once triggered.
- Assumes the distance being measured is one to eight feet.
- Runs in continuous capture mode once triggered for the seconds indicated as  duration, set in the config.
- Accepts an ON command via 'homie/<deviceID>/position/ranging/set -> ON | OFF', which will trigger continuous capture for duration seconds.
- Public API:
    bool isReady()                            Is VL53L1x ready and communicating?
    void setRunDuration(const int seconds)    set a new run duration in seconds
    void startRanging()                       start continuous capture for Duration 
    void stopRanging()                        stop capture



### Homie Config: data/homie/config.json

```
{
  "name": "Time of Flight Ranger",
  "device_id": "L1XRanger",
  "device_stats_interval": 900,  
  "wifi": {
    "ssid": "<wifi-host>",
    "password": "<wifi-password>"
  },
  "mqtt": {
    "host": "<mqtt-hostname-or-ip>",
    "port": 1883,
	"base_topic": "sknSensors/",
    "auth": true,
    "username": "<mqtt-username>",
    "password": "<mqtt-password>"
  },
  "ota": {
    "enabled": true
  },
  "settings": {
      "duration": 40
    }
}
```


### Homie Serial Runtime Log
```
08:39:47.828 > 💡 Firmware Time of Flight Ranger (1.0.0)
08:39:47.828 > 🔌 Booting into normal mode 🔌
08:39:48.103 > {} Stored configuration
08:39:48.105 >   • Hardware device ID: 240ac4594254
08:39:48.109 >   • Device ID: TOFRanger
08:39:48.111 >   • Name: Time of Flight Ranger
08:39:48.114 >   • Device Stats Interval: 300 sec
08:39:48.117 >   • Wi-Fi: 
08:39:48.119 >     ◦ SSID: SFNSS1-24G
08:39:48.121 >     ◦ Password not shown
08:39:48.123 >   • MQTT: 
08:39:48.125 >     ◦ Host: openhab.skoona.net
08:39:48.128 >     ◦ Port: 1883
08:39:48.129 >     ◦ Base topic: sknSensors/
08:39:48.132 >     ◦ Auth? yes
08:39:48.134 >     ◦ Username: openhabian
08:39:48.136 >     ◦ Password not shown
08:39:48.139 >   • OTA: 
08:39:48.140 >     ◦ Enabled? yes
08:39:48.142 >   • Custom settings: 
08:39:48.144 >     ◦ duration: 15 (set)
08:39:48.147 > • Ranging Module:
08:39:48.153 > 〽 Medium distance mode accepted.
08:39:48.156 > 〽 200us timing budget accepted.
08:39:48.159 > ↕ Attempting to connect to Wi-Fi...
08:39:51.140 > ✔ Wi-Fi connected, IP: 10.100.1.244
08:39:51.144 > Triggering WIFI_CONNECTED event...
08:39:51.147 > ↕ Attempting to connect to MQTT...
08:39:51.181 > Sending initial information...
08:39:51.238 > ✔ MQTT ready
08:39:51.240 > Triggering MQTT_READY event...
08:39:51.243 > 〽 Node: TOF Ranger Ready to operate.
08:39:51.246 > Calling setup function...
08:39:51.249 > 〽 Sending statistics...
08:39:51.251 >   • Interval: 305s (300s including 5s grace time)
08:39:51.255 >   • Wi-Fi signal quality: 90%
08:39:51.258 >   • Uptime: 4s
08:39:51.284 > 📢 Calling broadcast handler...
08:39:51.287 > Received broadcast level alert: OH3 Offline
08:39:51.291 > 📢 Calling broadcast handler...
08:39:51.294 > Received broadcast level LWT: HomieMonitor Offline!
08:40:08.152 > 〽 Start continuous ranging @ 280 ms accepted.
08:40:08.378 > 〽 range: 288 mm 	status: range valid	signal: 71.92 MCPS	ambient: 0.10 MCPS
08:40:08.661 > 〽 range: 297 mm 	status: range valid	signal: 14.41 MCPS	ambient: 0.02 MCPS
08:40:08.946 > 〽 range: 297 mm 	status: range valid	signal: 14.40 MCPS	ambient: 0.02 MCPS
08:40:09.211 > 〽 range: 297 mm 	status: range valid	signal: 14.45 MCPS	ambient: 0.02 MCPS
08:40:09.510 > 〽 range: 298 mm 	status: range valid	signal: 14.51 MCPS	ambient: 0.02 MCPS
08:40:09.796 > 〽 range: 298 mm 	status: range valid	signal: 14.49 MCPS	ambient: 0.02 MCPS
08:40:10.059 > 〽 range: 298 mm 	status: range valid	signal: 14.41 MCPS	ambient: 0.02 MCPS
08:40:10.334 > 〽 range: 297 mm 	status: range valid	signal: 14.44 MCPS	ambient: 0.02 MCPS
08:40:10.626 > 〽 range: 298 mm 	status: range valid	signal: 14.46 MCPS	ambient: 0.02 MCPS
08:40:10.894 > 〽 range: 298 mm 	status: range valid	signal: 14.49 MCPS	ambient: 0.02 MCPS
08:40:11.182 > 〽 range: 298 mm 	status: range valid	signal: 14.46 MCPS	ambient: 0.02 MCPS
08:40:11.454 > 〽 range: 297 mm 	status: range valid	signal: 14.48 MCPS	ambient: 0.02 MCPS
08:40:11.734 > 〽 range: 298 mm 	status: range valid	signal: 14.55 MCPS	ambient: 0.02 MCPS
08:40:12.016 > 〽 range: 297 mm 	status: range valid	signal: 14.60 MCPS	ambient: 0.02 MCPS
08:40:12.292 > 〽 range: 298 mm 	status: range valid	signal: 14.59 MCPS	ambient: 0.02 MCPS

...

08:40:24.897 > 〽 range: 299 mm 	status: range valid	signal: 14.54 MCPS	ambient: 0.02 MCPS
08:40:25.174 > 〽 range: 299 mm 	status: range valid	signal: 14.55 MCPS	ambient: 0.02 MCPS
08:40:25.450 > 〽 range: 299 mm 	status: range valid	signal: 14.47 MCPS	ambient: 0.02 MCPS
08:40:25.759 > 〽 range: 299 mm 	status: range valid	signal: 14.52 MCPS	ambient: 0.02 MCPS
08:40:26.020 > 〽 range: 299 mm 	status: range valid	signal: 14.55 MCPS	ambient: 0.02 MCPS
08:40:26.301 > 〽 range: 299 mm 	status: range valid	signal: 14.48 MCPS	ambient: 0.02 MCPS
08:40:26.575 > 〽 range: 299 mm 	status: range valid	signal: 14.46 MCPS	ambient: 0.03 MCPS
08:40:26.878 > 〽 range: 299 mm 	status: range valid	signal: 14.51 MCPS	ambient: 0.02 MCPS
08:40:27.141 > 〽 range: 299 mm 	status: range valid	signal: 14.47 MCPS	ambient: 0.02 MCPS
08:40:27.416 > 〽 range: 299 mm 	status: range valid	signal: 14.44 MCPS	ambient: 0.02 MCPS
08:40:27.712 > 〽 range: 298 mm 	status: range valid	signal: 14.59 MCPS	ambient: 0.02 MCPS
08:40:27.993 > 〽 range: 299 mm 	status: range valid	signal: 14.48 MCPS	ambient: 0.02 MCPS
08:40:28.150 > 〽 Stopping continuous ranging accepted.
08:40:48.148 > 〽 Start continuous ranging @ 280 ms accepted.
08:40:48.374 > 〽 range: 299 mm 	status: range valid	signal: 14.49 MCPS	ambient: 0.02 MCPS
08:40:48.672 > 〽 range: 298 mm 	status: range valid	signal: 14.44 MCPS	ambient: 0.02 MCPS
08:40:48.942 > 〽 range: 298 mm 	status: range valid	signal: 14.52 MCPS	ambient: 0.02 MCPS
08:40:49.211 > 〽 range: 298 mm 	status: range valid	signal: 14.55 MCPS	ambient: 0.02 MCPS
08:40:49.495 > 〽 range: 297 mm 	status: range valid	signal: 14.50 MCPS	ambient: 0.02 MCPS
08:40:49.786 > 〽 range: 298 mm 	status: range valid	signal: 14.57 MCPS	ambient: 0.02 MCPS
08:40:50.054 > 〽 range: 298 mm 	status: range valid	signal: 14.58 MCPS	ambient: 0.02 MCPS
08:40:50.349 > 〽 range: 298 mm 	status: range valid	signal: 14.64 MCPS	ambient: 0.02 MCPS
08:40:50.620 > 〽 range: 297 mm 	status: range valid	signal: 14.51 MCPS	ambient: 0.02 MCPS

```
