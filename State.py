class State:
    def __init__(self, agent_location, energy_remaining, treasures_collected,
                 remaining_treasures, effects_active):
        self.agent_location = agent_location  # (row, col)
        self.energy_remaining = energy_remaining  # int
        self.treasures_collected = set(treasures_collected)  # set of (row, col)
        self.remaining_treasures = set(remaining_treasures)  # set of (row, col)
        self.effects_active = list(effects_active)  # list of strings, e.g. ['speed', 'gravity']


    def __eq__(self, other):
        return (self.agent_location == other.agent_location and
                self.energy_remaining == other.energy_remaining and
                self.treasures_collected == other.treasures_collected and
                self.remaining_treasures == other.remaining_treasures and
                self.effects_active == other.effects_active)

    # Checks if goal has been reached
    def is_goal_state(self):
        return len(self.remaining_treasures) == 0

    # Outputs the result
    def __repr__(self):
        return (f"State(pos={self.agent_location}, energy={self.energy_remaining}, "
                f"collected={len(self.treasures_collected)}, remaining={len(self.remaining_treasures)}, "
                f"effects={self.effects_active})")


    def is_valid(self):
        """Check if state is valid (energy >= 0)"""
        return self.energy_remaining >= 0