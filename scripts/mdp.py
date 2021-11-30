import math
from nav_msgs.msg import OccupancyGrid
class Node:
    values = {}

    def policy_iteration(self):
        while True:
            # Policy Evaluation
            delta = math.inf
            theta = 0.001
            while delta >= theta:
                for s in self.values:
                    a = self.policy_get(s)
                    states = self.possible_tx(s, action)

                    prev_value = self.values.get(s, 0)
                    total = 0
                    for p in states:
                        total += self.reward(s, a, p) + self.gamma * self.values.get(p, 0)

                    self.values[s] = total
                    delta = max(delta, abs(prev_value - total))

            # Policy Improvement
            for s in self.values:
                old_action = self.policy_get(s)

                max_val = math.inf
                best_action = None
                for a in self.actions:
                    states = self.possible_tx(s, a)
                    for p in states:
                        value = self.reward(s, a, p) + self.gamma * self.values[p]
                        if value > max_val:
                            max_val = value
                            best_action = a

                self.policy_set(s, best_action)
                if old_action != best_action:
                    stable = False

            if stable:
                return

    def policy_get(self, s) -> int: # n for clean room ; -1 for recharge
        # Return action for state s
        pass

    def policy_set(self, s, a) -> int :# n for clean room ; -1 for recharge
        # Set action for state s as a
        pass

    def possible_tx(self, s, a) -> s:
        # Return possible states given we start in s and take action a
        pass

    def reward(self, s, a, p) -> int:
        reward = 0
	    #penalise actions which would leave the robot stranded
        if (next.canReturntoCharge() == False):
            reward -= math.inf
	    #incentivise more clean rooms
	    if next.noOfCleanRooms > s.noOfCleanRooms:
		    reward += 50
	    #add battery level to reward, seek to conserve bettery
	    reward += next[-1]
	    #penalise outcomes which result in the robot moving further away from its current state
	    reward -= distance_between_states(self, state[-2], next[-2])
	    return reward

    def noOfCleanrooms(s):
            cleanrooms = 0
            for i in range(0, len(s)-2):
                cleanrooms += s[i]
            return cleanrooms



    def state_to_area(self, s) -> float: # areas or distance inside room s
        pass

    def distance_between_states(self, s1, s2) -> float: # distance
        pass

    def distance_to_battery(self, s, s_p) -> int: # battery level 0-100
        pass


    def get_init_map(self) -> OccupancyGrid:
        pass

    def get_segmented_map(self, map:OccupancyGrid ) -> list: # list of [{centre, area, id}]
        pass
