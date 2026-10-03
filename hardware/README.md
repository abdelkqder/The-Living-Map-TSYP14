# Hardware — ESP32-only stack

## Platform decision

**ESP32 DevKit V1 per robot · Arduino Nano per beacon · ESP32 as ONA · laptop Python as Command Post.**

The lighter ESP32-only stack was chosen over the Raspberry Pi 4 + Arduino Mega option.

| Criterion | ESP32-only (chosen) | RPi 4 + Mega |
|-----------|---------------------|--------------|
| Cost | ~340 TND/robot | ~580 TND/robot |
| Firmware language | C++/MicroPython | Python-native on RPi |
| Compute headroom | Sufficient for A* + LoRa at this arena scale | Large margin |
| Local availability | All parts at CoThings Tunis | RPi 4 stock inconsistent |
| Integration complexity | Single MCU per robot | RPi↔Mega serial bridge |

`communication/lora_transport.py` (currently a stub) now has a concrete target:
SX1278 Ra-02 LoRa via ESP32 SPI on the GPIOs listed below.

---

## Corrected GPIO pin map — ESP32 DevKit V1 (per robot)

> ⚠ **Two conflicts existed in planning notes — both are resolved here.**
> - Conflict A: GPIO16 assigned to both HC-SR04 Front ECHO **and** LoRa SX1278 RST
>   → LoRa RST moved to **GPIO12**
> - Conflict B: GPIO34/35 assigned to both Encoder A PCNT channels **and** KY-026/MQ-2 sensors
>   → sensors routed through a **PCF8574 I2C GPIO expander** on the existing I2C bus

| Peripheral              | Signal           | GPIO              | Notes                                     |
|-------------------------|------------------|-------------------|-------------------------------------------|
| L298N Motor A           | IN1 / IN2        | 25 / 26           |                                           |
| L298N Motor B           | IN3 / IN4        | 32 / 33           |                                           |
| L298N Enable            | ENA / ENB        | 14 / 27           | PWM via LEDC                              |
| Encoder A               | Ch.A / Ch.B      | 34 / 35           | PCNT hardware peripheral                  |
| Encoder B               | Ch.A / Ch.B      | 36 / 39           | PCNT hardware peripheral                  |
| MPU-6050                | SDA / SCL        | 21 / 22           | I2C master                                |
| HC-SR04 Front           | TRIG / ECHO      | 4 / **16**        | 3.3 V version — no level-shift needed     |
| HC-SR04 Right           | TRIG / ECHO      | 17 / 5            | 3.3 V version — no level-shift needed     |
| **KY-026 Flame**        | **DO**           | **PCF8574 P0**    | **via I2C — was GPIO34, conflict fixed**  |
| **MQ-2 Gas**            | **DO (÷1k/2k)**  | **PCF8574 P1**    | **via I2C — was GPIO35, conflict fixed**  |
| LoRa SX1278 Ra-02       | SCK/MISO/MOSI    | 18 / 19 / 23      | SPI                                       |
| LoRa SX1278 Ra-02       | NSS / DIO0       | 15 / 2            |                                           |
| **LoRa SX1278 Ra-02**   | **RST**          | **12**            | **was GPIO16 — conflict fixed**           |
| SG90 Servo              | PWM              | 13                |                                           |
| **PCF8574 expander**    | **SDA / SCL**    | **21 / 22**       | **I2C addr 0x20 · A0=A1=A2=GND**         |

### Why PCF8574 for KY-026 and MQ-2?

The ESP32 DevKit V1 is fully saturated after assigning motors, PCNT encoders
(GPIO34–39, input-only), I2C (MPU-6050), SPI (LoRa), and the two ultrasonic
sensors. The PCF8574 shares the existing I2C bus (GPIO21/22, already serving
MPU-6050 at 0x68) and costs ~8 TND per robot. Both KY-026 DO and MQ-2 DO
(through a 1 kΩ/2 kΩ voltage divider) are simple digital signals with no
timing requirements — I2C polling at the sensing step is sufficient.

### Why GPIO12 for LoRa RST?

GPIO12 is the de-facto RST pin on commercial ESP32-LoRa boards (TTGO LoRa32
etc.). It is boot-strapping sensitive during the ≈10 ms power-on window only
— LoRa RST is asserted only by firmware, never by external hardware at
startup, so this is not a concern in practice.

---

## Power tree

```
2S LiPo 7.4 V 2000 mAh
  → 1N5822 (polarity diode) → 5 A fuse → power switch
       ├─ L298N H-bridge  (7.4 V direct — motors)
       └─ LM2596 buck  → 5.0 V
              ├─ MQ-2 heater  (~150 mA)
              ├─ SG90 servo
              └─ AMS1117-3.3 LDO  → 3.3 V
                     ├─ ESP32 DevKit V1        (~240 mA peak, WiFi on)
                     ├─ LoRa SX1278 Ra-02      (~120 mA TX burst)
                     ├─ MPU-6050 + PCF8574     (I2C bus, GPIO21/22)
                     └─ HC-SR04 ×2             (3.3 V versions)

All GNDs → single common node
  (LiPo−, L298N GND, LM2596 GND, AMS1117 GND, all sensors and MCUs).
```

---

## Bill of materials — full system (2 robots + 4 beacons + 1 ONA)

> Prices estimated from CoThings Tunis, September 2026.

| Item                                      | Qty | Unit (TND) | Total (TND) |
|-------------------------------------------|-----|-----------|-------------|
| ESP32 DevKit V1                           | 5   | 22        | 110         |
| Arduino Nano (beacon MCU)                 | 4   | 15        | 60          |
| L298N H-bridge module                     | 2   | 8         | 16          |
| MPU-6050 module                           | 2   | 7         | 14          |
| HC-SR04 ultrasonic 3.3 V                  | 4   | 4         | 16          |
| KY-026 flame sensor                       | 2   | 3         | 6           |
| MQ-2 gas sensor                           | 2   | 6         | 12          |
| LoRa SX1278 Ra-02 module                  | 7   | 35        | 245         |
| **PCF8574 I2C GPIO expander**             | **2** | **8** | **16**      |
| 2S LiPo 7.4 V 2000 mAh                   | 2   | 45        | 90          |
| LM2596 buck module                        | 2   | 5         | 10          |
| AMS1117-3.3 LDO                           | 4   | 1         | 4           |
| SG90 servo                                | 2   | 5         | 10          |
| 2× AA batteries (beacon power)            | 2 packs | 3   | 6           |
| Miscellaneous (wire, PCB, connectors,     |     |           |             |
| resistors, capacitors)                    | —   | —         | 60          |
| **Total**                                 |     |           | **~675 TND** |

> Previously estimated ~559 TND. The +116 TND delta is primarily the 2 additional
> LoRa modules (ONA needs one too, now counted) and the 2× PCF8574 (+16 TND)
> that resolve the GPIO34/35 conflict.

---

## Phase 2 validation sequence

Build outward from the simulation, not inward from full assembly.

1. **Power rail verification** — confirm 5.0 V and 3.3 V rails under load before connecting any MCU
2. **ESP32 ↔ ESP32 LoRa link** — Writer transmits, ONA receives — verify packet count and RSSI
3. **Single beacon** — Arduino Nano + LoRa Ra-02 stores one `BeaconMessage`, broadcasts at 1 Hz, ONA decodes
4. **Motor + encoder** — closed-loop distance test; verify PCNT counts match expected displacement
5. **Sensor stack** — confirm PCF8574 reads KY-026 and MQ-2 DO lines correctly while MPU-6050 I2C runs simultaneously
6. **First full mission** — arena ≥ 4 × 4 m, one FIRE beacon, Writer deploys, Executor verifies
