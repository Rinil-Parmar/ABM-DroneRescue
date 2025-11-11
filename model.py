# model.py
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
        # only degrade health if not rescued and still alive
        if not self.rescued and self.health > 0:
            self.health -= self.model.victim_decay_rate
            if self.health < 0:
                self.health = 0


class SupplyHubAgent(Agent):
    """Hub where drones drop victims and recharge. Has a no-op step so it can safely be scheduled."""
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)

    def step(self):
        # intentionally empty; hubs are static
        pass


class ObstacleAgent(Agent):
    """Static obstacle blocking movement."""
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)

    def step(self):
        # obstacles are static
        pass


class DroneAgent(Agent):
    """
    Drone that searches, communicates locally, picks up victims, and returns to hub.
    States: 'search' | 'deliver' | 'recharge' | 'failed'
    """
    def __init__(self, unique_id, model, sensor_prob=0.9, comms_range=2, battery=100):
        super().__init__(unique_id, model)
        self.sensor_prob = sensor_prob
        self.comms_range = comms_range
        self.battery = battery
        self.max_battery = battery
        self.carrying = None          # victim unique_id when carrying
        self.state = "search"

    def neighbors_in_range(self, radius):
        # Use grid neighborhood rather than manual loops to be less error-prone.
        neighbors = []
        for pos in self.model.grid.get_neighborhood(self.pos, moore=True, include_center=False, radius=radius):
            for agent in self.model.grid.get_cell_list_contents([pos]):
                if agent is not self:
                    neighbors.append(agent)
        return neighbors

    def sense_for_victim(self):
        # check same cell for victims (high priority)
        for agent in self.model.grid.get_cell_list_contents([self.pos]):
            if isinstance(agent, VictimAgent) and (not agent.found) and agent.health > 0:
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
        # move one cell closer (8-neighborhood allowed)
        step_x = x + (1 if dx > 0 else -1 if dx < 0 else 0)
        step_y = y + (1 if dy > 0 else -1 if dy < 0 else 0)
        candidate = (step_x, step_y)
        # ensure candidate is inside grid and not blocked by obstacle
        if 0 <= candidate[0] < self.model.width and 0 <= candidate[1] < self.model.height:
            cell_contents = self.model.grid.get_cell_list_contents([candidate])
            if not any(isinstance(a, ObstacleAgent) for a in cell_contents):
                self.model.grid.move_agent(self, candidate)
                self.model.visited.add(candidate)

    def alert(self, victim_id):
        # respond to peer alert by moving toward victim
        if self.carrying is None and self.state not in ("deliver", "recharge", "failed"):
            victim = self.model.victims.get(victim_id)
            if victim and victim.health > 0:
                self.move_towards(victim.pos)

    def step(self):
        # Drone fails if battery dead
        if self.battery <= 0:
            self.state = "failed"
            return

        # If carrying a victim, head to nearest hub and drop them off
        if self.carrying is not None:
            self.state = "deliver"
            hub_pos = self.model.find_nearest_hub(self.pos)
            self.move_towards(hub_pos)
            # if on hub cell, deliver
            for agent in self.model.grid.get_cell_list_contents([self.pos]):
                if isinstance(agent, SupplyHubAgent):
                    victim = self.model.victims.get(self.carrying)
                    if victim and victim.health > 0:
                        victim.rescued = True
                        victim.found = True
                        self.model.rescued_count += 1
                        # event record
                        self.model.events.append(("rescued", self.unique_id, victim.unique_id, self.model.schedule.time))
                    self.carrying = None
                    self.state = "search"
                    self.battery = self.max_battery
                    break
            # consume battery for moving / action
            self.battery -= self.model.battery_consumption_per_step
            return

        # If low on battery, go to hub to recharge
        if self.battery < self.model.low_battery_threshold:
            self.state = "recharge"
            hub_pos = self.model.find_nearest_hub(self.pos)
            self.move_towards(hub_pos)
            for agent in self.model.grid.get_cell_list_contents([self.pos]):
                if isinstance(agent, SupplyHubAgent):
                    self.battery = self.max_battery
                    self.state = "search"
                    break
            self.battery -= self.model.battery_consumption_per_step
            return

        # Normal search behavior
        self.state = "search"

        # 1) Try to sense a victim in current cell
        victim = self.sense_for_victim()
        if victim is not None:
            # pick up victim immediately and alert neighbors
            victim.found = True
            self.model.found_count += 1
            self.carrying = victim.unique_id
            self.model.events.append(("found", self.unique_id, victim.unique_id, self.model.schedule.time))
            # broadcast to neighbors within comms_range
            for n in self.neighbors_in_range(self.comms_range):
                if isinstance(n, DroneAgent) and n.state != "failed":
                    n.alert(victim.unique_id)
            self.battery -= self.model.battery_consumption_per_step
            return

        # 2) Move to an unexplored adjacent cell if possible, else random neighbor that is not obstacle
        target = self.model.find_unexplored_adjacent(self.pos)
        if target is None:
            # construct possible moves excluding obstacles and out-of-bounds
            possible = [pos for pos in self.model.grid.get_neighborhood(self.pos, moore=True, include_center=False)
                        if not any(isinstance(a, ObstacleAgent) for a in self.model.grid.get_cell_list_contents([pos]))]
            # prefer staying if nothing valid
            new_pos = self.random.choice(possible) if possible else self.pos
        else:
            new_pos = target

        self.model.grid.move_agent(self, new_pos)
        self.model.visited.add(new_pos)
        # consume battery for moving
        self.battery -= self.model.battery_consumption_per_step


class DroneSwarmModel(Model):
    """ABM for drone swarm disaster recovery."""
    def __init__(self, width=20, height=20, n_drones=6, n_victims=8, n_hubs=1, n_obstacles=20,
                 battery=100, sensor_prob=0.9, comms_range=2, seed=None):
        super().__init__(seed=seed)
        self.width = width
        self.height = height
        self.schedule = RandomActivation(self)
        self.grid = MultiGrid(width, height, torus=False)

        # configurable params
        self.n_drones = n_drones
        self.n_victims = n_victims
        self.n_hubs = n_hubs
        self.n_obstacles = n_obstacles
        self.battery = battery
        self.victim_decay_rate = 0.5
        self.battery_consumption_per_step = 1
        self.low_battery_threshold = int(0.25 * battery)

        # runtime state
        self.visited = set()
        self.found_count = 0
        self.rescued_count = 0
        self.events = []
        self.victims = {}   # dict: uid -> VictimAgent

        uid = 0

        # Place hubs (by default place first hub at (0,0); add opposite corner if multiple)
        hubs_positions = [(2, 1)]
        if self.n_hubs > 1:
            hubs_positions.append((width - 1, height - 1))
        for pos in hubs_positions[:self.n_hubs]:
            hub = SupplyHubAgent(uid, self)
            self.grid.place_agent(hub, pos)
            # hubs are static but safe to schedule (they have a no-op step)
            self.schedule.add(hub)
            uid += 1

        # Place obstacles
        for _ in range(self.n_obstacles):
            placed = False
            for _ in range(200):  # avoid infinite loop; try some attempts
                x, y = self.random.randrange(width), self.random.randrange(height)
                # avoid placing obstacles on hubs
                if all(not isinstance(a, SupplyHubAgent) for a in self.grid.get_cell_list_contents([(x, y)])):
                    obs = ObstacleAgent(uid, self)
                    self.grid.place_agent(obs, (x, y))
                    self.schedule.add(obs)
                    uid += 1
                    placed = True
                    break
            if not placed:
                break

        # Place victims (avoid obstacles and hubs)
        for _ in range(self.n_victims):
            placed = False
            for _ in range(200):
                x, y = self.random.randrange(width), self.random.randrange(height)
                cell_agents = self.grid.get_cell_list_contents([(x, y)])
                if not any(isinstance(a, ObstacleAgent) for a in cell_agents) and not any(isinstance(a, SupplyHubAgent) for a in cell_agents):
                    v = VictimAgent(uid, self, health=100)
                    self.victims[uid] = v
                    self.grid.place_agent(v, (x, y))
                    self.schedule.add(v)
                    uid += 1
                    placed = True
                    break
            if not placed:
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
            },
            agent_reporters={
                # optionally collect agent-level metrics (commented out by default)
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
        # consider 8 neighbors but allow moving into victim cells (we only block obstacles)
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.width and 0 <= ny < self.height:
                    if (nx, ny) not in self.visited:
                        # do not select cells blocked by obstacles
                        if not any(isinstance(a, ObstacleAgent) for a in self.grid.get_cell_list_contents([(nx, ny)])):
                            candidates.append((nx, ny))
        return self.random.choice(candidates) if candidates else None

    def step(self):
        # collect stats, then step all scheduled agents
        self.datacollector.collect(self)
        self.schedule.step()
