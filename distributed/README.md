# Homeostat II -- Distributed Units

A room-scale physical recreation of W. Ross Ashby's homeostat (1947).

Four equation nodes in the corners of a room. One sensor node watching for motion. Five ESP32-S3 Superminis talking to each other via ESP-NOW. Each corner is a unit of the homeostat -- owning one state variable, one row of the coupling matrix, making its own decision about when it is out of viable range.

The room finds equilibrium, or it doesn't.

---

## Files

| File | Description |
|---|---|
| `homeostat_equation_node.ino` | Flash to each corner node. Change `DEVICE_ID` (1-4) only. |
| `homeostat_sensor_node.ino` | Flash to the sensor node. Never change `DEVICE_ID` (always 0). |
| `homeostat_distributed_context.md` | Full technical context -- wiring, parameters, known issues, conceptual notes. |

---

## Quick start

1. Flash `homeostat_equation_node.ino` to four Superminis, changing `DEVICE_ID` to 1, 2, 3, 4
2. Flash `homeostat_sensor_node.ino` to one Supermini (PIR on GPIO 6)
3. Place equation nodes in corners
4. Power up -- nodes boot staggered, find each other via ESP-NOW broadcast
5. Walk in front of the PIR -- the room responds

## Hardware

- 5x ESP32-S3 Supermini
- 4x WS2812 LED strips, 16 LEDs each (GPIO 4 per node)
- 1x HC-SR501 PIR sensor (GPIO 6 on sensor node)
- External 5V supply recommended for LED strips at full brightness

## Key dependencies

- FastLED (equation nodes)
- Arduino core for ESP32 3.x
- Board: ESP32S3 Dev Module, USB Mode: Hardware CDC and JTAG

---

*Homeostat II is part of an ongoing practice-based research project. Built in collaboration with T. Zafiropulos and Claude (Anthropic).*
