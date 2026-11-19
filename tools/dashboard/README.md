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

## Barcodes (2026-11-18)

Everything is on `dashboard.html`:

- **Barcodes and junctions** card: the letter just read (big tile), what it will do at
  the next junction, the mapping (send it before START), read counts and the picture of
  the last failed read.
- **Barcode scan test** card (link at the top): with the robot READY, it drives itself
  straight over a barcode (distance = barcode length + 10 cm, speed 50 mm/s) and the
  wave shows what the left sensor saw. "Live sensor" streams the wave without moving.

`barcode_test.html` is the older stand-alone version of the scan test (same commands).
It is no longer needed.
