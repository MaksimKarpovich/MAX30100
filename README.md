# MAX30100

This repository contains the solution for Pulse Oximeter and Heart-Rate Sensor IC MAX30100. It can be used to measure: 

- Heartbeat (beats per minute)
- [SpO<sub>2</sub>](https://en.wikipedia.org/wiki/Oxygen_saturation) (peripheral oxygen saturation, %)

## Connection

![Connection](Pictures/Connection.png)

|Group        |Name|Description                                 |
|-------------|----|--------------------------------------------|
|Power        |VCC |1.8-3.3 V                                   |
|Data exchange|SCL |I<sup>2</sup>C Clock Input                  |
|Data exchange|SDA |I<sup>2</sup>C Clock Data (Open-Drain)      |
|Data exchange|INT |Active-Low Interrupt (Open-Drain)           |
|LED power    |IRD |Power Supply (Anode Connection) for IR LED. |
|LED power    |RD  |Power Supply (Anode Connection) for Red LED.|
|Power        |GND |Ground                                      |

## Sequencing
