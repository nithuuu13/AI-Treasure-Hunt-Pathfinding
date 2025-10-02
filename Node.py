class Node:
    def __init__(self, pos, content, neighbours, h_c):
        self.position = pos #(col, row)
        self.content = content  # E.g., '.', 'X', 'T1', 'R1', '$'
        self.neighbors = neighbours
        self.heuristic_cost = h_c
        
    def is_obstacle(self):
        return self.content == 'OB'
    
    def is_treasure(self):
        return self.content == 'TR'
    
    def is_trap(self):
        return self.content.startswith('TP')
    
    def is_reward(self):
        return self.content.startswith('RW')
    
    def get_trap_number(self):
        if self.is_trap():
            return int(self.content[2])
        return None
    
    def get_reward_number(self):
        if self.is_reward():
            return int(self.content[2])
        return None