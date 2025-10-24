from mesa.visualization.ModularVisualization import ModularServer
from mesa.visualization.UserParam import Slider, Choice
from mesa.visualization.modules import CanvasGrid, ChartModule
from model import DroneSwarmModel, DroneAgent, VictimAgent, ObstacleAgent, SupplyHubAgent


# --- Helper: Smooth color transition ---
def fade_color(start_color, end_color, fraction):
    """Generate a hex color between start_color and end_color."""
    start_rgb = [int(start_color[i:i + 2], 16) for i in (0, 2, 4)]
    end_rgb = [int(end_color[i:i + 2], 16) for i in (0, 2, 4)]
    rgb = [
        int(start_rgb[i] + (end_rgb[i] - start_rgb[i]) * max(0, min(fraction, 1)))
        for i in range(3)
    ]
    return f"#{rgb[0]:02x}{rgb[1]:02x}{rgb[2]:02x}"


# --- Agent portrayal ---
# --- Agent portrayal ---
def agent_portrayal(agent):
    """Visual representation of each agent type."""
    if agent is None:
        return None

    portrayal = {
        "Shape": "circle",
        "Filled": "true",   # solid fill
        "Layer": 0,
        "r": 2              # small, balanced size
    }

    if isinstance(agent, DroneAgent):
        # ✅ Show dead/inactive drone differently
        if getattr(agent, "battery", 0) <= 0:
            # Dead drone: gray
            portrayal.update({"Color": "#888888", "Layer": 3, "r": 2.5})
        else:
            frac = max(agent.battery / agent.max_battery, 0)
            color = fade_color("ff0000", "00ff00", frac)  # Red → Green
            portrayal.update({"Color": color, "Layer": 3, "r": 2.5})

    elif isinstance(agent, VictimAgent):
        # ✅ Check if victim has been rescued
        if getattr(agent, "rescued", False):
            portrayal.update({"Color": "#00ffff", "Layer": 2, "r": 2.3})  # Cyan for rescued
        else:
            frac = max(agent.health / 100, 0)
            color = fade_color("550000", "ff0000", frac)  # Dark → Bright Red
            portrayal.update({"Color": color, "Layer": 2, "r": 2.3})

    elif isinstance(agent, SupplyHubAgent):
        portrayal.update({"Color": "#0044ff", "Layer": 4, "r": 3})

    elif isinstance(agent, ObstacleAgent):
        portrayal.update({"Color": "#666666", "Layer": 1, "r": 1.8})

    else:
        portrayal.update({"Color": "white", "Layer": 0, "r": 1.8})

    return portrayal



# --- Visualization modules ---
grid = CanvasGrid(agent_portrayal, 20, 20, 500, 500)  # ✅ more compact, fits screen better

chart = ChartModule(
    [
        {"Label": "Coverage", "Color": "green"},
        {"Label": "Found", "Color": "orange"},
        {"Label": "Rescued", "Color": "blue"},
        {"Label": "ActiveDrones", "Color": "red"},
    ],
    data_collector_name="datacollector",
)


# --- Interactive controls ---
model_params = {
    "width": 20,
    "height": 20,
    "n_drones": Slider("Number of Drones", 6, 1, 30, 1),
    "n_victims": Slider("Number of Victims", 8, 0, 50, 1),
    "n_hubs": Slider("Number of Supply Hubs", 1, 1, 4, 1),
    "n_obstacles": Slider("Number of Obstacles", 20, 0, 200, 1),
    "battery": Slider("Drone Battery (steps)", 80, 10, 300, 5),
    "sensor_prob": Slider("Sensor Success Probability", 0.9, 0.1, 1.0, 0.05),
    "comms_range": Slider("Drone Communication Range (cells)", 2, 0, 6, 1),
    "seed": Choice(value="Random", choices=["Random", 0, 1, 42, 123]),
}


# --- Server setup ---
server = ModularServer(
    DroneSwarmModel,
    [grid, chart],
    "Drone Swarm Disaster Recovery Simulation",
    model_params,
)
server.port = 8521

if __name__ == "__main__":
    server.launch()
