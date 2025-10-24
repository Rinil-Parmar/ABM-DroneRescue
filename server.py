from mesa.visualization.ModularVisualization import ModularServer
from mesa.visualization.UserParam import Slider
from mesa.visualization.modules import CanvasGrid, ChartModule
from model import DroneSwarmModel, DroneAgent, VictimAgent, ObstacleAgent, SupplyHubAgent

# --- Helper: Color fade ---
def fade_color(start_color, end_color, fraction):
    start_rgb = [int(start_color[i:i+2], 16) for i in (0, 2, 4)]
    end_rgb = [int(end_color[i:i+2], 16) for i in (0, 2, 4)]
    rgb = [int(start_rgb[i] + (end_rgb[i] - start_rgb[i]) * fraction) for i in range(3)]
    return f"#{rgb[0]:02x}{rgb[1]:02x}{rgb[2]:02x}"

# --- Agent portrayal ---
def agent_portrayal(agent):
    if agent is None:
        return None

    portrayal = {"Shape": "circle", "text": "", "Layer": 0, "r": 3}

    if isinstance(agent, DroneAgent):
        frac = max(agent.battery / agent.max_battery, 0)
        color = fade_color("ff0000", "00ff00", frac)  # red->green
        portrayal.update({"Color": color, "Layer": 2, "r": 3})

    elif isinstance(agent, VictimAgent):
        frac = max(agent.health / 100, 0)
        color = fade_color("000000", "ff0000", frac)  # black->red
        portrayal.update({"Color": color, "Layer": 1, "r": 3})

    elif isinstance(agent, SupplyHubAgent):
        portrayal.update({"Color": "blue", "Layer": 3, "r": 4})

    elif isinstance(agent, ObstacleAgent):
        portrayal.update({"Color": "gray", "Layer": 1, "r": 3})

    else:
        portrayal.update({"Color": "white", "Layer": 0, "r": 2})

    return portrayal

# --- Grid ---
grid = CanvasGrid(agent_portrayal, 20, 20, 500, 500)

# --- Chart ---
chart = ChartModule(
    [
        {"Label": "Coverage", "Color": "green"},
        {"Label": "Found", "Color": "orange"},
        {"Label": "Rescued", "Color": "blue"},
        {"Label": "ActiveDrones", "Color": "red"},
    ],
    data_collector_name="datacollector"
)

# --- Model parameters ---
model_params = {
    "width": 20,
    "height": 20,
    "n_drones": Slider("Number of drones", 6, 1, 30, 1),
    "n_victims": Slider("Number of victims", 8, 0, 50, 1),
    "n_hubs": Slider("Number of hubs", 1, 1, 4, 1),
    "n_obstacles": Slider("Number of obstacles", 20, 0, 200, 1),
    "battery": Slider("Drone battery (steps)", 80, 10, 300, 5),
    "sensor_prob": Slider("Sensor success probability", 0.9, 0.1, 1.0, 0.05),
    "comms_range": Slider("Drone comms range (cells)", 2, 0, 6, 1),
}

# --- Server ---
server = ModularServer(
    DroneSwarmModel,
    [grid, chart],
    "Drone Swarm Disaster Recovery Simulation",
    model_params
)
server.port = 8521

if __name__ == "__main__":
    server.launch(open_browser=True)
