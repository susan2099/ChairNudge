# ChairNudge

An ESP32 + MPU-6050 on the side of a chair cushion. Sit for 45 minutes straight and it beeps
(and pings your phone) until you get up. A 2-minute break resets the timer.

CS244P Internet of Things, UCI, Fall 2026, Team 14.

## How it works

The ESP32 detects sitting by how much the cushion tilts the board, then sends the state to
Grafana Cloud over HTTPS. Grafana does the dashboard and phone alert. No server of our own.

## Running it

1. Copy `include/secrets.example.h` to `include/secrets.h` and fill in Wi-Fi and Grafana info.
2. `POST ... -> 204` means it's working.
