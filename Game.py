import heapq
from Node import Node
from State import State

class Game:
    def __init__(self, filename):
        self.rows = 6
        self.cols = 10
        self.node_grid = [[None for _ in range(
            self.rows)] for _ in range(self.cols)]
        self.all_treasures = []
        self.start_position = None
        self.load_map(filename)

    def load_map(self, filename):
        with open(filename, 'r') as f:
            lines = [line.strip() if line.strip()
                     else '.' for line in f.readlines()]
        index = 0
        for col in range(self.cols):
            for row in range(self.rows):
                content = lines[index]
                pos = (col, row)
                node = Node(pos, content, [], 0)
                if node.is_treasure():
                    self.all_treasures.append(pos)
                self.node_grid[col][row] = node
                index += 1

    def solve(self):
        """
        A* search algorithm to find optimal path collecting all treasures
        Returns: (path, nodes_expanded)
        """
        if not self.all_treasures:
            return [], 0 #no treasures to collect
        
        #find start position (let's assume its marked default as (0,0) or 'EN')
        start_pos = self.find_start_position()
        if not start_pos:
            start_pos = (0,0) #default start position

        # initializing start state
        initial_state = State(
            agent_location=start_pos,
            energy_remaining=100,
            treasures_collected=[],
            remaining_treasures=self.all_treasures.copy(),
            effects_active=[]
        )

        #A* algorithm core
        open_list = [] #priority queue: stores (f_cost, node_data)
        closed_set = set() #visited states to avoid cycles

        #initial node: (state, g_cost, f_cost path)
        start_node = (initial_state, 0, self.calculate_heuristic(initial_state), [])
        heapq.heappush(open_list, (start_node[2], start_node)) #push by f_cost

        nodes_expanded = 0

        while open_list:
            # get node with lowest f_cost (this is most promising)
            f_cost, (current_state, g_cost, _, path) = heapq.heappop(open_list)

            # create unique key for this state
            state_key = self.state_to_key(current_state)

            # skip if already processed
            if state_key in closed_set:
                continue

            closed_set.add(state_key)
            nodes_expanded += 1

            #check if goal is reached (all treasure collected)
            if current_state.is_goal_state():
                return path, nodes_expanded
            
            #generate all possible next moves
            for next_state, action in self.get_successors(current_state):
                if not next_state.is_valid(): #skip invalid states (-ve energy)
                    continue

                next_key = self.state_to_key(next_state)
                if next_key in closed_set: #skip states that have already been processed
                    continue

                #calculate costs for this move
                step_cost = self.calculate_step_cost(current_state, next_state)
                new_g_cost = g_cost + step_cost # total cost from start
                h_cost = self.calculate_heuristic(next_state) #estimated cost to goal
                new_f_cost = new_g_cost + h_cost #total estimated cost

                new_path = path + [action]
                new_node = (next_state, new_g_cost, new_f_cost, new_path)

                heapq.heappush(open_list, (new_f_cost, new_node))

        return [], nodes_expanded #no solution found

        
    def calculate_step_cost(self, current_state, next_state):
        """
        Calculate cost of moving from current_state to next_state
        This coonsiders all active effects that modify movement/energy costs
        """
        base_cost = 1

        #apply movement and energy multipliers based on active effects
        movement_multiplier = self.get_movement_multiplier(current_state.effects_active)
        energy_multiplier = self.get_energy_multiplier(current_state.effects_active)

        #total cost combines movement time and energy consumption
        total_cost = (base_cost * movement_multiplier) + (base_cost * energy_multiplier)

        return total_cost
    
    def calculate_heuristic(self, state):
        """Estimate cost to collect remaining treasures using Manhattan distance"""
        if not state.remaining_treasures:
            return 0
        
        current_pos = state.agent_location
        total_distance = 0

        for treasure_pos in state.remaining_treasures:
            dist = abs(current_pos[0] - treasure_pos[0]) + abs(current_pos[1] - treasure_pos[1])
            total_distance += dist

        movement_multiplier = self.get_movement_multiplier(state.effects_active)
        return total_distance * movement_multiplier
    
    def get_successors(self, current_state):
        successors = []
        current_pos = current_state.agent_location

        neighbors = self.get_valid_neighbors(current_pos)

        for next_pos in neighbors:
            next_node = self.get_node(next_pos)
            if next_node is None or next_node.is_obstacle():
                continue

            new_state = self.apply_move(current_state, next_pos)
            action = f"Move from {current_pos} to {next_pos}"
            successors.append((new_state, action))

        return successors
    
    def apply_move(self, current_state, next_pos):
        next_node = self.get_node(next_pos)

        new_treasures_collected = current_state.treasures_collected.copy()
        new_remaining_treasures = current_state.remaining_treasures.copy()
        new_effects = current_state.effects_active.copy()
        new_energy = current_state.energy_remaining

        #collect treasure
        if next_pos in new_remaining_treasures:
            new_remaining_treasures.remove(next_pos)
            new_treasures_collected.append(next_pos)

        #apply trap or reward
        if next_node.is_trap():
            new_effects, new_remaining_treasures = self.apply_trap_effect(
                next_node.get_trap_number(), new_effects, new_remaining_treasures)
        elif next_node.is_reward():
            new_effects = self.apply_reward_effect(next_node.get_reward_number(), new_effects)

        #trap 4: displacement trap
        final_pos = next_pos
        if next_node.is_trap() and next_node.get_trap_number() == 4:
            final_pos = self.apply_displacement_trap(current_state.agent_location, next_pos)

        temp_state = State(final_pos, new_energy, new_treasures_collected, new_remaining_treasures, new_effects)
        step_cost = self.calculate_step_cost(current_state, temp_state)
        new_energy -= step_cost

        return State(
            agent_location=final_pos,
            energy_remaining=new_energy,
            treasures_collected=new_treasures_collected,
            remaining_treasures=new_remaining_treasures,
            effects_active=new_effects
        )
    
    def apply_trap_effect(self, trap_number, current_effects, remaining_treasures):
        updated_effects = current_effects.copy()

        if trap_number == 1:
            updated_effects.append("trap1_speed_penalty")
        elif trap_number == 2:
            updated_effects.append("trap2_gravity_increase")
        elif trap_number == 3:
            if remaining_treasures:
                remaining_treasures.pop()  # Remove a random treasure
    # Trap 4 handled via displacement, not here

        return updated_effects, remaining_treasures
    
    def apply_reward_effect(self, reward_number, current_effects):
        updated_effects = current_effects.copy()

        if reward_number == 1:
            updated_effects.append("reward1_gravity_decrease")
        elif reward_number == 2:
            updated_effects.append("reward2_speed_bonus")

        return updated_effects
    
    def apply_displacement_trap(self, prev_pos, curr_pos):
        """Push the player backward (reverse direction)"""
        dx = curr_pos[0] - prev_pos[0]
        dy = curr_pos[1] - prev_pos[1]

        reverse_x = curr_pos[0] - dx
        reverse_y = curr_pos[1] - dy

        if 0 <= reverse_x < self.cols and 0 <= reverse_y < self.rows:
            return (reverse_x, reverse_y)
        return curr_pos  # fallback to current if out of bounds


    def get_movement_multiplier(self, active_effects):
        """Calculate how effects modify movement speed"""
        multiplier = 1.0

        if 'trap1_speed_penalty' in active_effects:
            multiplier *= 2.0 #trap 1: double steps needed
        if 'reward2_speed_bonus' in active_effects:
            multiplier *= 0.5 #reward 2: half steps needed

        return multiplier
    
    def get_energy_multiplier(self, active_effects):
        """Calculate how effects modify energy consumption"""
        multiplier = 1.0

        if 'trap2_gravity_increase' in active_effects:
            multiplier *= 2.0 #trap 2: double energy per step
        if 'reward1_gravity_decrease' in active_effects:
            multiplier *= 0.5 #reward 1: half energy per step

        return multiplier
      
    def get_hex_neighbors(self, q: int, r: int) -> list[tuple[int, int]]:
        """
        Finds the six nearest neighbors for a given hexagonal coordinate (q, r)
        in an "even-q" vertical layout.
    
        The "even-q" layout shifts even columns down, which affects the 'r'
        coordinate calculation for diagonal neighbors.
    
        Args:
            q: The 'q' (column) coordinate of the hexagon.
            r: The 'r' (row) coordinate of the hexagon.
    
        Returns:
            A list of tuples, where each tuple represents the (q, r) coordinates
            of a neighboring hexagon.
        """
        neighbors = []
    
        # Define the 6 directions for hexagonal neighbors
        # These are standard axial coordinate directions, adjusted for the "even-q" offset.
    
        # Check if the 'q' coordinate is even or odd to apply correct offsets
        if q % 2 == 0:
            # Offsets for even 'q' columns
            # (delta_q, delta_r)
            directions = [
                (1,  0),   # Top-right
                (1,  1),   # Bottom-right
                (0,  1),   # Bottom
                (-1, 1),   # Bottom-left
                (-1, 0),   # Top-left
                (0, -1)    # Top
            ]
        else:
            # Offsets for odd 'q' columns
            # (delta_q, delta_r)
            directions = [
                (1, -1),   # Top-right
                (1,  0),   # Bottom-right
                (0,  1),   # Bottom
                (-1, 0),   # Bottom-left
                (-1, -1),  # Top-left
                (0, -1)    # Top
            ]
    
        # Calculate the coordinates of each neighbor
        for dq, dr in directions:
            neighbor_q = q + dq
            neighbor_r = r + dr
    
            if (neighbor_q >-1) and (neighbor_r >-1):
                neighbors.append((neighbor_q, neighbor_r))
    
        return neighbors
    
    def get_valid_neighbors(self, pos):
        q, r = pos
        neighbors = self.get_hex_neighbors(q, r)

        valid_neighbors = []
        for x, y in neighbors:
            if 0 <= x < self.cols and 0 <= y < self.rows:
                valid_neighbors.append((x, y))
        return valid_neighbors

        
        
