# Drone Swarm Disaster Recovery Simulation

This repository contains an **Agent-Based Model (ABM)** simulating a swarm of drones performing disaster recovery operations. The drones search for victims, rescue them, and return them to supply hubs while navigating obstacles and managing battery levels.

The simulation is built using **[Mesa](https://mesa.readthedocs.io/)**, a Python library for ABMs.

---

## Features

- Drones autonomously explore a 2D grid.
- Victims are stationary and have health that decays over time.
- Drones can pick up victims and deliver them to supply hubs.
- Drones communicate with nearby drones to alert about victims.
- Battery management and drone failure when the battery is depleted.
- Visualisation includes dynamic agent colours to reflect states.
- Real-time charts track Coverage, Found victims, Rescued victims, and Active Drones.

---

## Agent Details & Visualization

| Agent Type       | Color         | Meaning / State                                 |
|-----------------|---------------|------------------------------------------------|
| Drone (active)  | Red → Green   | Battery level (low → full)                     |
| Drone (dead)    | Gray (#888888)| Battery depleted / inactive                     |
| Victim (unrescued)| Red (#ff0000) → Dark Red (#550000)| Needs help, health decays over time |
| Victim (rescued)| Cyan (#00ffff)| Already rescued                                 |
| Supply Hub      | Blue (#0044ff)| Drones can deliver victims and recharge here   |
| Obstacle        | Medium Gray (#666666)| Blocks drone movement                         |

---

## Simulation Controls

The simulation allows user interaction via sliders:

- **Number of Drones:** 1 – 30
- **Number of Victims:** 0 – 50
- **Number of Supply Hubs:** 1 – 4
- **Number of Obstacles:** 0 – 200
- **Drone Battery (steps):** 10 – 300
- **Sensor Success Probability:** 0.1 – 1.0
- **Drone Communication Range (cells):** 0 – 6
- **Random Seed:** Random / 0 / 1 / 42 / 123

---

## Setup Instructions

### 1. Clone the repository
```bash
git clone <your-repo-url>
cd <repo-folder>
```

### 2. Create a virtual environment (recommended)
```bash
python -m venv venv
```

Activate it:

**Windows:**
```bash
venv\Scripts\activate
```

**Mac/Linux:**
```bash
source venv/bin/activate
```

### 3. Install dependencies
```bash
pip install -r requirements.txt
```

### 4. Run the simulation
```bash
python server.py
```

Then open your browser at:
```
http://127.0.0.1:8521
```

---

## Requirements

The `requirements.txt` should include:
```
mesa==1.3.0
matplotlib==3.7.2
numpy==1.26.0
```

Adjust versions based on your environment if needed.

---

## Model Description

The **DroneSwarmModel** implements an ABM for disaster recovery:

- **Grid-based 2D world**: Drones, victims, hubs, and obstacles occupy cells.
- **Drone behavior**:
  - `search`: Move to unexplored cells and sense for victims.
  - `deliver`: Carry rescued victims to the nearest hub.
  - `recharge`: Go to the hub when battery is low.
  - `failed`: Drone stops moving when battery reaches 0.
- **Victim behavior**:
  - Health decreases over time if not rescued.
  - Rescued victims are marked `rescued=True` and remain visible.
- **Hub behavior**:
  - Static supply hubs allow drones to recharge and deliver victims.
- **Obstacle behavior**:
  - Static, impassable barriers.

### Data Collection:
- `Coverage`: Fraction of grid visited.
- `Found`: Total victims detected.
- `Rescued`: Total victims delivered to hub.
- `ActiveDrones`: Count of drones that are not failed.

---

## Visualization

- **CanvasGrid**: Shows all agents on the grid with colour-coded states.
- **ChartModule**: Tracks simulation statistics in real-time.

---

## Notes

- Ensure your virtual environment is activated before running the simulation.
- You can modify the number of drones, victims, and obstacles to test different scenarios.
- The model is fully configurable through sliders and random seed selection.

