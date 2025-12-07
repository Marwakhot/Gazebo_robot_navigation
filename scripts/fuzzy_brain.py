import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

class FuzzyNavigator:
    def __init__(self):
        print("Initializing Fuzzy Navigation System...")
        
        self.avoiding_obstacle = False
        self.avoid_timer = 0
        self.turn_direction = 0
        
        # Input variables
        self.front = ctrl.Antecedent(np.arange(0, 5.1, 0.1), 'front')
        self.heading = ctrl.Antecedent(np.arange(-180, 181, 1), 'heading')

        # Output variables
        self.speed = ctrl.Consequent(np.arange(0, 0.5, 0.01), 'speed', defuzzify_method='centroid')
        self.turn = ctrl.Consequent(np.arange(-1.0, 1.01, 0.01), 'turn', defuzzify_method='centroid')

        # Distance membership functions
        self.front['close'] = fuzz.trimf(self.front.universe, [0, 0, 0.8])
        self.front['medium'] = fuzz.trimf(self.front.universe, [0.5, 1.2, 2.0])
        self.front['far'] = fuzz.trimf(self.front.universe, [1.5, 5.0, 5.0])

        # Heading membership functions
        self.heading['hard_left'] = fuzz.trapmf(self.heading.universe, [-180, -180, -60, -30])
        self.heading['left'] = fuzz.trimf(self.heading.universe, [-60, -20, -5])
        self.heading['straight'] = fuzz.trimf(self.heading.universe, [-10, 0, 10])
        self.heading['right'] = fuzz.trimf(self.heading.universe, [5, 20, 60])
        self.heading['hard_right'] = fuzz.trapmf(self.heading.universe, [30, 60, 180, 180])

        # Speed outputs
        self.speed['slow'] = fuzz.trimf(self.speed.universe, [0.2, 0.25, 0.3])
        self.speed['medium'] = fuzz.trimf(self.speed.universe, [0.25, 0.35, 0.4])
        self.speed['fast'] = fuzz.trimf(self.speed.universe, [0.35, 0.45, 0.5])

        # Turn outputs
        self.turn['hard_left'] = fuzz.trimf(self.turn.universe, [-1.0, -0.8, -0.5])
        self.turn['left'] = fuzz.trimf(self.turn.universe, [-0.6, -0.4, -0.1])
        self.turn['straight'] = fuzz.trimf(self.turn.universe, [-0.15, 0, 0.15])
        self.turn['right'] = fuzz.trimf(self.turn.universe, [0.1, 0.4, 0.6])
        self.turn['hard_right'] = fuzz.trimf(self.turn.universe, [0.5, 0.8, 1.0])

        # Rules
        rule1 = ctrl.Rule(self.front['close'] & self.heading['hard_left'], 
                         [self.speed['slow'], self.turn['hard_left']])
        rule2 = ctrl.Rule(self.front['close'] & self.heading['left'], 
                         [self.speed['slow'], self.turn['hard_left']])
        rule3 = ctrl.Rule(self.front['close'] & self.heading['straight'], 
                         [self.speed['slow'], self.turn['hard_left']])
        rule4 = ctrl.Rule(self.front['close'] & self.heading['right'], 
                         [self.speed['slow'], self.turn['hard_right']])
        rule5 = ctrl.Rule(self.front['close'] & self.heading['hard_right'], 
                         [self.speed['slow'], self.turn['hard_right']])
        
        rule6 = ctrl.Rule(self.front['medium'] & self.heading['hard_left'], 
                         [self.speed['medium'], self.turn['hard_left']])
        rule7 = ctrl.Rule(self.front['medium'] & self.heading['left'], 
                         [self.speed['medium'], self.turn['left']])
        rule8 = ctrl.Rule(self.front['medium'] & self.heading['straight'], 
                         [self.speed['medium'], self.turn['straight']])
        rule9 = ctrl.Rule(self.front['medium'] & self.heading['right'], 
                         [self.speed['medium'], self.turn['right']])
        rule10 = ctrl.Rule(self.front['medium'] & self.heading['hard_right'], 
                          [self.speed['medium'], self.turn['hard_right']])
        
        rule11 = ctrl.Rule(self.front['far'] & self.heading['hard_left'], 
                          [self.speed['fast'], self.turn['hard_left']])
        rule12 = ctrl.Rule(self.front['far'] & self.heading['left'], 
                          [self.speed['fast'], self.turn['left']])
        rule13 = ctrl.Rule(self.front['far'] & self.heading['straight'], 
                          [self.speed['fast'], self.turn['straight']])
        rule14 = ctrl.Rule(self.front['far'] & self.heading['right'], 
                          [self.speed['fast'], self.turn['right']])
        rule15 = ctrl.Rule(self.front['far'] & self.heading['hard_right'], 
                          [self.speed['fast'], self.turn['hard_right']])

        self.system = ctrl.ControlSystem([
            rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9, rule10,
            rule11, rule12, rule13, rule14, rule15
        ])
        self.sim = ctrl.ControlSystemSimulation(self.system)
        
        print("Fuzzy system initialized!")

    def get_action(self, f, l, r, h):
        front_clipped = np.clip(f, 0.01, 5.0)
        heading_clipped = np.clip(h, -180, 180)
        
        try:
            self.sim.input['front'] = front_clipped
            self.sim.input['heading'] = heading_clipped
            self.sim.compute()
            
            speed = float(self.sim.output['speed'])
            turn = float(self.sim.output['turn'])
            
            # OBSTACLE AVOIDANCE STATE MACHINE
            if f < 0.6:
                if not self.avoiding_obstacle:
                    self.avoiding_obstacle = True
                    self.avoid_timer = 0
                    self.turn_direction = 1.0 if l > r else -1.0
                    print(f"OBSTACLE! Starting avoidance - turning {'RIGHT' if self.turn_direction < 0 else 'LEFT'}")
            
            if self.avoiding_obstacle:
                self.avoid_timer += 1
                
                # Phase 1: Turn for 15 steps (1.5 seconds)
                if self.avoid_timer < 15:
                    speed = 0.0
                    turn = self.turn_direction * 1.0
                    print(f"Avoidance Phase 1: Turning in place ({self.avoid_timer}/15)")
                
                # Phase 2: Move forward for 30 steps (3 seconds)
                elif self.avoid_timer < 45:
                    speed = 0.4
                    turn = 0.0
                    print(f"Avoidance Phase 2: Moving forward ({self.avoid_timer}/45)")
                
                # Phase 3: Turn back for 10 steps (1 second)
                elif self.avoid_timer < 55:
                    speed = 0.3
                    turn = -self.turn_direction * 0.6
                    print(f"Avoidance Phase 3: Turning back ({self.avoid_timer}/55)")
                
                # Phase 4: Move forward again for 20 steps (2 seconds)
                elif self.avoid_timer < 75:
                    speed = 0.4
                    turn = 0.0
                    print(f"Avoidance Phase 4: Final forward ({self.avoid_timer}/75)")
                
                # Done avoiding
                else:
                    self.avoiding_obstacle = False
                    self.avoid_timer = 0
                    print("Avoidance complete - resuming navigation")
                
                return speed, turn
            
            # NORMAL NAVIGATION
            if speed < 0.25:
                speed = 0.25
            
            # Boost turn for large heading errors
            if abs(h) > 60:
                turn = turn * 1.3
                turn = np.clip(turn, -1.0, 1.0)
            
            return speed, turn
            
        except Exception as e:
            print(f"Fuzzy error: {e}")
            if f < 0.5:
                return 0.3, (0.9 if l > r else -0.9)
            elif abs(h) > 60:
                return 0.35, (0.8 if h > 0 else -0.8)
            else:
                return 0.4, 0.0