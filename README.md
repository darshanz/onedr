# onedr

**onedr** is a lightweight Python library for controlling drones via MAVLink, built on top of [pymavlink](https://github.com/ArduPilot/pymavlink). 🎮

> **Note:** This library is a focused wrapper designed specifically for **GUIDED mode** control of ArduPilot-based drones through a companion computer. It does **not** expose the full functionality of `pymavlink`.

---

## ✨ Features

- Simple Python interface for MAVLink drone control
- Supports GUIDED mode operations like takeoff, navigation, and landing
- Works with ArduPilot firmware through a companion computer

---

## 🛠 Installation

Before installing `onedr`, make sure you have Python 3.7 or higher installed.

```bash
pip install pymavlink
# Then clone and install onedr
git clone https://github.com/yourusername/onedr.git
cd onedr
pip install .
```


## 🚀 Getting Started

Example usage:

Connecting to the vehicle (currently supports Ardupilot autopilot)

```python

    drone_controller  = DroneController(logging.getLogger())
    mydrone = drone_controller.connect(FCU_URL)
    
```

Arm & Take off

```python

    mydrone.set_guided()
    time.sleep(5)
    mydrone.arm(wait_for_position=True)
    time.sleep(5)
    mydrone.take_off()
    
```


## ⚠️ Limitations

- Only meant for GUIDED mode operations
- Requires MAVLink-compatible flight controller. (Currently only supports Ardupilot)
  
---


## 🧪 Testing

If you're testing without a real drone, consider using SITL environments with ArduPilot to simulate behavior.

---




    
