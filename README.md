# Arduino-UAVTalk-Blinkenlights
Read telemetry from a QuadCopter FlightControl CC3D via UAVTalk protocol

The communication between CC3D and Arduino is copied from minoposd project (https://code.google.com/p/minoposd/) - Thanks a lot!

## Hardware

We are using a Arduino Nano, so I guess the source will run on any other Arduino as well.

### Wiring

Configure the main port of CC3D to send telemetry on 57600 baud (default setting) and connect the green/yellow cable (in my case) to the RX/TX port of the arduino. We powered the arduino directly with the 3S LiPo from the copter (raw VIn).
