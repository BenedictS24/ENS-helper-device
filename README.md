# ENS Helper Device

A chest-mounted haptic feedback belt that helps people with Empty Nose Syndrome (ENS) perceive their breathing again.
The device measures chest expansion using a load cell connected to a strap and translates this signal into vibrations via a small vibration motor embedded in the device.

---
## Repository Structure

```
ENS-helper-device/
├── .gitignore
├── KiCad/
│   ├── sensory_support_device.kicad_pcb
│   └── sensory_support_device.kicad_sch
├── README.md
├── force_vibration_controller/
│   └── force_vibration_controller.ino
├── images/
│   ├── PCB-3d.png
│   ├── PCB.png
│   ├── breadboard_circuit.jpeg
│   ├── circuit.png
│   ├── device_closed.jpeg
│   ├── device_open.jpeg
│   ├── device_worn.JPG
│   └── protoboard_in_case.jpeg
└── models/
    ├── back_plate.stl
    ├── load_cell_attachment.stl
    ├── main_plate.stl
    └── main_plate_cover.stl
```

---
## Overview

People with Empty Nose Syndrome often struggle to feel the airflow in their nose, which can make it harder to sense their own breathing and can greatly impact everyday life and well-being.  
This device offers an alternative channel: **haptic feedback on the chest**.

1. A chest strap attached to a **bar-type load cell** is placed around the chest.  
2. When the user inhales and exhales, the changing chest circumference alters the **force on the strap**.  
3. The load cell signal is amplified and read by an **Arduino Nano**.  
4. The firmware processes the signal and drives a **vibration motor** via a MOSFET transistor.  
5. The vibration pattern mirrors the breathing signal, giving the user a **tactile sense of their breathing**.

The firmware supports different modes, such as:

- **Absolute force** – vibration strength based on the total measured force  
- **Rate of change** – feedback based on how quickly the force changes during breathing


---

## Hardware

The prototype uses the following components:

- **Microcontroller:** Arduino Nano
- **Force sensor:** Bar-type load cell (approx. 20 kg)  
- **Amplifier:** HX711 load cell amplifier module  
- **Actuator:** Small cylindrical vibration motor  
- **Driver:** MOSFET
- **Mechanical parts:**  
  - Elastic chest strap  
  - 3D-printed enclosure and attachments (`models/`)  
  - M4/M5 Screws, nuts, protoboard  


---

## 3D-Printed Enclosure

The enclosure consists of the following STL files:

- `main_plate.stl` – holds load cell + electronics  
- `back_plate.stl` – attaches to the straps at the back  
- `load_cell_attachment.stl` – mount that connects the strap to the load cell
- `main_plate_cover.stl` – cover of main the enclosure 

Recommended print settings:

- Material: PLA or PETG
- Layer height: 0.16 mm
- Infill: 20–30 %
- Supports: minimal or only where necessary
  * You may need to adjust the print orientation 


---

## Firmware

The firmware (`force_vibration_controller.ino`) is responsible for:

- Reading load cell values via **HX711**  
- Detecting absolute force or force change  
- Driving the vibration motor using PWM  

Firmware parameters such as mode selection, calibration factors, and thresholds can be adjusted directly in the `.ino` file.

Upload using the Arduino IDE.


---

## Disclaimer

This project is a hobby / research prototype.  
It is **not** a medical device and must **not** be used as a substitute for professional medical treatment or diagnosis.  
Use at your own risk.

