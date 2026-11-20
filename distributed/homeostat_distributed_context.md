# Distributed Homeostat – Project Context

## What this is

A room-scale physical recreation of W. Ross Ashby’s homeostat (1947), distributed across five ESP32-S3 Supermini nodes via ESP-NOW. Four equation nodes sit in the corners of a room; one sensor node watches for motion and perturbs the system.

Built for educational and installation purposes. The goal is to make Ashby’s ideas about ultrastability visible, tangible, and spatial.

-----

## Architecture

### Five nodes total

|Node         |File                         |Role                                         |
|-------------|-----------------------------|---------------------------------------------|
|Sensor (ID 0)|`homeostat_sensor_node.ino`  |Watches room via PIR, broadcasts perturbation|
|Unit 1 (ID 1)|`homeostat_equation_node.ino`|Owns x[0], corner 1, receives perturbation   |
|Unit 2 (ID 2)|`homeostat_equation_node.ino`|Owns x[1], corner 2                          |
|Unit 3 (ID 3)|`homeostat_equation_node.ino`|Owns x[2], corner 3                          |
|Unit 4 (ID 4)|`homeostat_equation_node.ino`|Owns x[3], corner 4                          |

### Ashby mapping

|Ashby concept          |This implementation                                 |
|-----------------------|----------------------------------------------------|
|Unit                   |One equation node in one corner                     |
|Needle (state variable)|`x_own` – the float each node owns                  |
|Trough / integrator    |The ESP32 running dx/dt every 60ms                  |
|Coupling (k matrix)    |Each node owns one row; received x values from peers|
|Uniselector (S2)       |`fireUniselector()` – fires per node independently  |
|Environment            |Sensor node – Ashby’s hand pushing the needle       |
|Viable range           |`BOUND` – exceeding triggers S2                     |

### Key design principle (T. Zafiropulos)

The system does not heal by erasing the disturbance. It heals by changing the relations among its parts. No `x_own = 0` reset on S2 fire – the system recovers through its new dynamics.

-----

## ESP-NOW comms

- All nodes broadcast to `FF:FF:FF:FF:FF:FF`
- Two message types, both `__attribute__((packed))`:
  - `msg_type=1` – KnockMessage (perturbation from sensor node)
  - `msg_type=2` – StateMessage (x value from equation node)
- Node 1 is the only equation node that absorbs sensor perturbations
- All nodes receive and store peer x values for integration
- Immediate broadcast after button/perturbation, before S2 can reset

-----

## Equation node – `homeostat_equation_node.ino`

### Flash instructions

- Change only `DEVICE_ID` (1, 2, 3, or 4) between flashes
- `NODE_INDEX = DEVICE_ID - 1` – DO NOT CHANGE

### Key parameters

|Parameter         |Value     |Effect                             |
|------------------|----------|-----------------------------------|
|`BOUND`           |1.0       |Viable range. Exceeding triggers S2|
|`DT`              |0.06      |Integration timestep               |
|`K_MAX`           |0.5       |Max coupling coefficient           |
|`DIAGONAL_DAMPING`|0.08      |Ceiling for randomised self-damping|
|`PERTURB_MIN/MAX` |5.0 / 10.0|Random perturbation strength range |
|`DISPLAY_SCALE`   |1.0       |Maps x magnitude to LED display    |

### Display logic (16x WS2812, GPIO 4)

|Visual                |Meaning                          |
|----------------------|---------------------------------|
|All green             |x near zero, at rest             |
|Green dropping to red |x positive, moving away from zero|
|Green dropping to blue|x negative                       |
|Yellow flash (all 16) |Perturbation received            |
|White flash (all 16)  |S2 reorganisation event          |

### S2 behaviour

- Fires when `abs(x_own) > BOUND`
- Randomises entire k row using `randomFloat(-K_MAX, K_MAX)`
- Diagonal randomised within `(-DIAGONAL_DAMPING, -DIAGONAL_DAMPING * 0.1)` – no two organisations identical
- Does NOT reset x_own to zero
- White flash signals the event visually
- Sequential: one node reconfigures at a time, returns immediately

### Wiring (equation node)

|GPIO|Connection                            |
|----|--------------------------------------|
|4   |WS2812 strip data                     |
|0   |Boot button (local perturbation input)|

-----

## Sensor node – `homeostat_sensor_node.ino`

### What it does

Watches the room via HC-SR501 PIR sensor. On motion detection (rising edge), broadcasts a KnockMessage with random strength to all nodes. Node 1 absorbs it.

This node is the environment – not part of the simulation.

### Key parameters

|Parameter            |Value     |Effect                      |
|---------------------|----------|----------------------------|
|`PIR_PIN`            |6         |GPIO for PIR signal out     |
|`PERTURB_MIN/MAX`    |5.0 / 10.0|Random perturbation strength|
|`PERTURB_COOLDOWN_MS`|1500      |Min ms between broadcasts   |

### HC-SR501 notes

- VCC: 5V (not 3.3V)
- Signal out: 3.3V safe for ESP32
- Jumper: set to L (single trigger) for clean rising edges
- Has ~30-60 second warmup on power on
- Two pots: sensitivity and hold time – turn hold time to minimum
- Onboard LED blinks on trigger

### Wiring (sensor node)

|GPIO|Connection         |
|----|-------------------|
|6   |HC-SR501 signal out|

### Serial debug output

- `[pir] HIGH -- motion detected` on rising edge
- `[pir] LOW -- clear` on falling edge
- `[sensor] Perturbation broadcast -- strength: X` when cooldown allows

-----

## Known issues / history

- ESP-NOW broadcast on S3 with newer Arduino core (3.x) requires `__attribute__((packed))` on message structs – without it struct padding causes len=8 instead of len=6 and message processing fails
- ESP-NOW send callback signature changed in core 3.x: use `wifi_tx_info_t*` not `uint8_t*`
- ESP-NOW receive callback: use `esp_now_recv_info*` not `uint8_t* mac`
- GPIO 2 unreliable on S3 Supermini for PIR – use GPIO 6
- MAC address prints as 00:00:00:00:00:00 if printed before `WiFi.mode(WIFI_STA)` / after `esp_wifi_set_channel` – print after `WiFi.disconnect()`
- GRB colour order on WS2812 strips with this hardware

-----

## Arduino IDE settings

- Board: ESP32S3 Dev Module (or ESP32-S3 Supermini)
- USB Mode: Hardware CDC and JTAG (must be enabled – easy to forget)
- Serial monitor: 115200 baud
- Serial plotter: shows all four x values as scrolling lines

## Libraries

- FastLED (equation nodes only)
- No additional libraries for sensor node

-----

## Conceptual notes

- The sensor node is the **environment**, not the uniselector. S2 lives inside each unit.
- Distributed S2: each corner decides for itself when it is out of viable range and hunts independently. The room finds equilibrium, or it doesn’t.
- ESP-NOW latency between nodes is a form of noise in the physics – a feature, not a bug.
- The hunting/oscillation phase is not a malfunction. It is the system searching for viability.
- “Who is the agent?” – Sui Wan, T. Zafiropulos, Claude (instance), the room.