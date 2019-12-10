import numpy as np
from tanbug import TangentBug

# initialize (x, y) of destination as None for further usage
self.target_x = None
self.target_y = None
self.q_hit = None
self.explore = True
self.whole_obstacle_visited = False
self.leave = False
self.m_line = None