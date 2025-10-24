from mesa import Agent, Model
from mesa.time import RandomActivation
from mesa.space import MultiGrid
from mesa.datacollection import DataCollector
import random


class VictimAgent(Agent):
    """Immobile victim with health that decays over time."""
    def __init__(self, unique_id, model, health=100):
        super().__init__(unique_id, model)
        self.health = health
        self.rescued = False
        self.found = False

    def step(self):
        if not self.rescued:
            self.health -= self.model.victim_decay_rate
            if self.health <= 0:
                self.health = 0


class SupplyHubAgent(Agent):
    """Hub where drones drop victims and recharge."""
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)


class ObstacleAgent(Agent):
    """Static obstacle blocking movement."""
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)


class DroneAgent(Agent):
    """
    Drone that searches, communicates locally, picks up victims, and returns to hub.
    """
    def __init__(self, unique_id, model, sensor_prob=0.9, comms_range=2, battery=100):
        super().__init__(unique_id, model)
        self.sensor_prob = sensor_prob
        self.comms_range = comms_range
        self.battery = battery
        self.max_battery = battery
        self.carrying = None
        self.state = "search"  # search | deliver | recharge | failed

    def neighbors_in_range(self, radius):
        x, y = self.pos
        neighbors = []
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.model.width and 0 <= ny < self.model.height:
                    for agent in self.model.grid.get_cell_list_contents([(nx, ny)]):
                        if agent is not self:
                            neighbors.append(agent)
        return neighbors

    def sense_for_victim(self):
        for agent in self.model.grid.get_cell_list_contents([self.pos]):
            if isinstance(agent, VictimAgent) and not agent.found and agent.health > 0:
                if random.random() < self.sensor_prob:
                    return agent
        return None

    def move_towards(self, target_pos):
        if target_pos is None:
            return
        x, y = self.pos
        tx, ty = target_pos
        dx = tx - x
        dy = ty - y
        step_x = x + (1 if dx > 0 else -1 if dx < 0 else 0)
        step_y = y + (1 if dy > 0 else -1 if dy < 0 else 0)
        candidate = (step_x, step_y)
        if 0 <= candidate[0] < self.model.width and 0 <= candidate[1] < self.model.height:
            if not any(isinstance(a, ObstacleAgent) for a in self.model.grid.get_cell_list_contents([candidate])):
                self.model.grid.move_agent(self, candidate)
                self.model.visited.add(candidate)

    def alert(self, victim_id):
        if self.carrying is None and self.state not in ("deliver", "recharge", "failed"):
            victim = self.model.victims[victim_id]
            if victim.health > 0:
                self.move_towards(victim.pos)

    def step(self):
        # Drone fails if battery dead
        if self.battery <= 0:
            self.state = "failed"
            return

        # Deliver victim if carrying
        if self.carrying is not None:
            self.state = "deliver"
            self.move_towards(self.model.find_nearest_hub(self.pos))
            for agent in self.model.grid.get_cell_list_contents([self.pos]):
                if isinstance(agent, SupplyHubAgent):
                    victim = self.model.victims[self.carrying]
                    if victim.health > 0:
                        victim.rescued = True
                        victim.found = True
                        self.model.rescued_count += 1
                    self.carrying = None
                    self.state = "search"
                    self.battery = self.max_battery
                    break
            self.battery -= self.model.battery_consumption_per_step
            return

        # Recharge if low battery
        if self.battery < self.model.low_battery_threshold:
            self.state = "recharge"
            self.move_towards(self.model.find_nearest_hub(self.pos))
            for agent in self.model.grid.get_cell_list_contents([self.pos]):
                if isinstance(agent, SupplyHubAgent):
                    self.battery = self.max_battery
                    self.state = "search"
                    break
            self.battery -= self.model.battery_consumption_per_step
            return

        # Search for victim
        self.state = "search"
        victim = self.sense_for_victim()
        if victim is not None:
            victim.found = True
            self.model.found_count += 1
            self.carrying = victim.unique_id
            self.model.events.append(("found", self.unique_id, victim.unique_id, self.model.schedule.time))
            for n in self.neighbors_in_range(self.comms_range):
                if isinstance(n, DroneAgent) and n.state != "failed":
                    n.alert(victim.unique_id)
            self.battery -= self.model.battery_consumption_per_step
            return

        # Move to unexplored adjacent cell or random valid move
        target = self.model.find_unexplored_adjacent(self.pos)
        if target is None:
            possible = [pos for pos in self.model.grid.get_neighborhood(self.pos, moore=True, include_center=False)]
            possible = [p for p in possible if not any(isinstance(a, ObstacleAgent) for a in self.model.grid.get_cell_list_contents([p]))]
            new_pos = random.choice(possible) if possible else self.pos
        else:
            new_pos = target

        self.model.grid.move_agent(self, new_pos)
        self.model.visited.add(new_pos)
        self.battery -= self.model.battery_consumption_per_step


class DroneSwarmModel(Model):
    """ABM for drone swarm disaster recovery."""
    def __init__(self, width=20, height=20, n_drones=6, n_victims=8, n_hubs=1, n_obstacles=20,
                 battery=100, sensor_prob=0.9, comms_range=2):
        super().__init__()
        self.width = width
        self.height = height
        self.schedule = RandomActivation(self)
        self.grid = MultiGrid(width, height, torus=False)
        self.n_drones = n_drones
        self.n_victims = n_victims
        self.n_hubs = n_hubs
        self.n_obstacles = n_obstacles
        self.battery = battery
        self.victim_decay_rate = 0.5
        self.battery_consumption_per_step = 1
        self.low_battery_threshold = int(0.25 * battery)
        self.visited = set()
        self.found_count = 0
        self.rescued_count = 0
        self.events = []
        self.victims = {}

        uid = 0

        # Place hubs
        hubs_positions = [(0, 0)]
        if self.n_hubs > 1:
            hubs_positions.append((width - 1, height - 1))
        for pos in hubs_positions[:self.n_hubs]:
            hub = SupplyHubAgent(uid, self)
            self.grid.place_agent(hub, pos)
            self.schedule.add(hub)
            uid += 1

        # Place obstacles
        for _ in range(self.n_obstacles):
            while True:
                x, y = self.random.randrange(width), self.random.randrange(height)
                if self.grid.is_cell_empty((x, y)):
                    obs = ObstacleAgent(uid, self)
                    self.grid.place_agent(obs, (x, y))
                    self.schedule.add(obs)
                    uid += 1
                    break

        # Place victims
        for _ in range(self.n_victims):
            while True:
                x, y = self.random.randrange(width), self.random.randrange(height)
                if self.grid.is_cell_empty((x, y)):
                    v = VictimAgent(uid, self, health=100)
                    self.victims[uid] = v
                    self.grid.place_agent(v, (x, y))
                    self.schedule.add(v)
                    uid += 1
                    break

        # Place drones at first hub
        for _ in range(self.n_drones):
            pos = hubs_positions[0]
            d = DroneAgent(uid, self, sensor_prob=sensor_prob, comms_range=comms_range, battery=battery)
            self.grid.place_agent(d, pos)
            self.schedule.add(d)
            self.visited.add(pos)
            uid += 1

        # Data collector
        self.datacollector = DataCollector(
            model_reporters={
                "Coverage": lambda m: len(m.visited) / (m.width * m.height),
                "Found": lambda m: m.found_count,
                "Rescued": lambda m: m.rescued_count,
                "ActiveDrones": lambda m: sum(1 for a in m.schedule.agents if isinstance(a, DroneAgent) and a.state != "failed")
            }
        )

    def find_nearest_hub(self, pos):
        min_dist = None
        best = None
        for cell in self.grid.coord_iter():
            cell_content, x, y = cell
            for agent in cell_content:
                if isinstance(agent, SupplyHubAgent):
                    dist = abs(x - pos[0]) + abs(y - pos[1])
                    if min_dist is None or dist < min_dist:
                        min_dist = dist
                        best = (x, y)
        return best

    def find_unexplored_adjacent(self, pos):
        x, y = pos
        candidates = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.width and 0 <= ny < self.height:
                    if (nx, ny) not in self.visited and self.grid.is_cell_empty((nx, ny)):
                        candidates.append((nx, ny))
        return random.choice(candidates) if candidates else None

    def step(self):
        self.datacollector.collect(self)
        self.schedule.step()
