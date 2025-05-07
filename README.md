# onedr 
onedr is a Python library for drone control using MAVLINK. This library heavily depends on pymavlink for communication. 🎮

### Requirements

```bash   
pip install pymavlink
```

### Installation

Download from github and setup 

```bash   
pip install -e .
```


### Usage

Example Usage

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
    
