# SE14 robot dashboard

`dashboard.html` shows the robot's MQTT data live: state and why it stopped,
wheel speeds, line sensors, ultrasonic, IMU, Wi-Fi, faults, an event log, and
START / STOP / RESUME / CANCEL / RESET buttons. It needs no internet.

## One-time setup: let browsers talk to Mosquitto

A web page can only use MQTT over WebSockets. Add a second listener to
`C:\Program Files\mosquitto\mosquitto.conf` (Administrator PowerShell):

    Add-Content "C:\Program Files\mosquitto\mosquitto.conf" "`nlistener 9001 0.0.0.0`nprotocol websockets"

## Every time

1. Start Mosquitto as usual (Administrator PowerShell):

       net stop mosquitto
       & "C:\Program Files\mosquitto\mosquitto.exe" -c "C:\Program Files\mosquitto\mosquitto.conf" -v

   It should print "Opening websockets listen socket on port 9001".
2. Double-click `dashboard.html` (opens in Chrome / Edge).
3. Broker = your laptop's IP from `ipconfig`, port `9001`, Connect.
   The page remembers them.

MQTT Explorer still works at the same time (port 1883).
