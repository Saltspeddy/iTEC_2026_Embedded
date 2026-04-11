# 🤖 Maze Solver Robot for iTEC#19

A fully remote-controlled maze-solving robot built for the **Embedded section** of the **IT Engineering Contest (iTEC#19)** held by the AC League. All hardware components were provided by the judging commission — no external parts were used.

Built by a team of 3: **Christian-Andrei Micea**, **Daniel-George Radu**, and myself — in approximately **44 hours** (6 hours of sleep included).

---

## 📦 Hardware components

- STM32F407G-DISC1 board (with built-in accelerometer)
- KIT Robot with 2 motors
- Dual L298N motor driver module
- 4× HC-SR04P ultrasonic sensors
- YS-27 Hall effect sensor module + magnets
- HC-05 Bluetooth Master/Slave module with adapter (3.3V / 5V compatible)
- Breadboard, jumper wires, power supply module kit
- 12V power battery

## 🛠️ Tools & equipment

- DC power supply
- Oscilloscope
- Soldering iron
- Multimeter

---

## ⚙️ Software & development environment

- **CLion** with integrated **CubeMX** plugin
- **Minicom** for serial port communication over Bluetooth

---

## 🧠 How it works

The robot operates in **4 modes**, selectable via Bluetooth:

| Mode | Description |
|------|-------------|
| `IDLE` | Robot is standing by |
| `STOP` | Emergency stop |
| `MAP` | Autonomously maps the maze |
| `TRAVERSE` | Receives coordinates and navigates the mapped maze |

During the **MAP** phase, the robot explores the maze using the **right-hand rule**, enabling systematic wall-following and full maze discovery.

Once mapping is complete, the **TRAVERSE** mode uses a **Breadth-First Search (BFS)** algorithm to compute the shortest path between the robot’s current position and the target coordinates. The robot then follows this optimal path in real time.

The primary challenge was building a reliable maze-mapping system and combining it with an efficient pathfinding algorithm — all controlled wirelessly via Bluetooth.


---

## 🔧 Timer & PWM configuration

```c
fTC = 84 MHz   // Timer clock frequency for APB1
PSC = 20       // Prescaler
ARR = 49 // Auto-reload register (sets PWM period)
```

---

## 📡 Bluetooth setup (Linux)

HC-05 MAC address: `98:DA:50:04:13:56`

```bash
# Pair and connect the HC-05 module
bluetoothctl
power on
devices          # list paired devices, note the MAC
quit

# Bind serial port (release first if already in use)
sudo rfcomm release 0
sudo rfcomm bind 0 98:DA:50:04:13:56

# Open serial terminal
minicom -D /dev/rfcomm0
```

---

## 🗺️ Maze specifications

- Corridor width: **26 cm** (±0.5 cm, max ±1 cm tolerance)
- Wall thickness: **~2 cm** (±tolerance)
- Grid size: **10×10 cells** (implemented with 12×12 for tolerance margin)

<img width="2000" height="1500" alt="image" src="https://github.com/user-attachments/assets/ad9ab56e-668b-4111-839a-ecec4356c082" />
<img width="2000" height="1500" alt="image" src="https://github.com/user-attachments/assets/461c6ff9-5de1-48d4-bcb7-7625a077b7bf" />
