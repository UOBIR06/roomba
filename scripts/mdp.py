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

    def policy_get(self, s):
        # Return action for state s

    def policy_set(self, s, a):
        # Set action for state s as a

    def possible_tx(self, s, a):
        # Return possible states given we start in s and take action a

    def reward(self, s, a, p):
        # Return reward
