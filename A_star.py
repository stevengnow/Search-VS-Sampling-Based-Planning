import numpy as np
from collections import defaultdict
from pqdict import pqdict
from tqdm import tqdm

class A_star_Algorithm_Planner:
    def __init__(self, boundary, blocks,res,eps):
        self.boundary = boundary
        self.blocks = blocks
        self.res = res
        self.eps = eps

    def cost_move(self,a,b):
        '''
        Function: Cost to move from point a to point b
        Inputs: points a = (x1,y1,z1) and b = (x2,y2,z2)
        Output: Cost
        '''
        cost = np.linalg.norm(np.array(b)-np.array(a))
        return cost    

    def dist(self,a,b):
        '''
        Function: Distance from point a to point 
        Inputs: points a = (x1,y1,z1) and b = (x2,y2,z2)
        Output: Distance
        '''
        dist = np.linalg.norm(np.array(b)-np.array(a))
        return dist 

    def heuristic(self, current_point, goal, res):
        '''
        Function: Heuristic Function
        Inputs: current point = (x,y,z), the goal = (xg,yg,zg), and the resolution (float)
        Output: Heuristic
        '''
        # dist = np.linalg.norm(np.array(current_point)-np.array(goal))
        # return dist 
        x,y,z = sorted(np.abs(current_point-goal))
        return ((res*3)**0.5*x + (res*2)**0.5*(y-x) +(res)**0.5 *(z-y))

    def collision_detection(self, blocks, boundary, parent, child):
        '''
        Function: Check for collisions
        Inputs: All blocks/obstacles in np.array([obstacle1,obstacle2,...]) 
                    where obstacle = [xmin,ymin,zmin,xmax,ymax,zmax,r,g,b]
                All boundaries in np.array([boundary])
                    where boundary = [xmin,ymin,zmin,xmax,ymax,zmax,r,g,b]
                Parent node (xp,yp,zp)
                Child node (xc,yc,zc)
        Output: True if line segment between parent and child intersect any of the objects
                False if not
        '''
        # Check boundary
        if(child[0] <= self.boundary[0,0] or child[0] >= self.boundary[0,3] or \
                        child[1] <= self.boundary[0,1] or child[1] >= self.boundary[0,4] or \
                        child[2] <= self.boundary[0,2] or child[2] >= self.boundary[0,5] ):
            return True
        # Check blocks
        blocks[:,:3] = blocks[:,:3] - 1e-6     # widen blocks so that we do not ride on wall
        blocks[:,3:6] = blocks[:,3:6] + 1e-6
        blocks = blocks[:,:6]
        blocks = blocks.reshape(-1,2,3)
        delta = child - parent
        t = (blocks - parent) / delta
        tmin = np.nanmin(t,axis = 1).max(axis = 1)
        tmax = np.nanmax(t,axis = 1).min(axis = 1)
        return np.any((tmax > tmin) & (((tmax<=1) & (tmax >=0)) | ((tmin<=1) & (tmin >=0))))

    def find_optimal_path(self, parents, start, goal):
        '''
        Function: Find optimal path
        Inputs: dictionary of all parent child pairs, start node, goal node
        Output: Optimal Path
        '''
        path = [tuple(goal)]
        idx = list(parents.keys())[-1]
        while idx != tuple(start):
            idx = parents[idx]
            path.append(idx)
        path = np.flip(np.array(path), axis = 0)
        return path
        
    def plan(self, start, goal):
        '''
        Function: Motion planning
        Inputs: start point, goal point
        Output: Optimal Path
        '''
        numofdirs = 26
        [dX,dY,dZ] = np.meshgrid([-1,0,1],[-1,0,1],[-1,0,1])
        dR = np.vstack((dX.flatten(),dY.flatten(),dZ.flatten()))
        dR = np.delete(dR,13,axis=1)
        dR = dR  * self.res
        
        # Create OPEN and CLOSED list
        OPEN = pqdict({tuple(start) : (0,0)}, key = lambda x: x[0])
        CLOSED = set()

        # Parent Vector
        parents = {}

        # Ignore divide by 0 warning
        np.seterr(divide='ignore', invalid='ignore')
  
        while OPEN:
            parent,(_,arrival_cost_parent) = OPEN.popitem()
            if parent in CLOSED:
                continue
            CLOSED.add(parent)
            # Stop condition
            if self.dist(parent, goal) < 0.1:
                return self.find_optimal_path(parents,start, goal), CLOSED
            if self.dist(parent,goal) < self.res and not self.collision_detection(self.blocks, self.boundary, parent, goal):
                h_j = 0  
                g_j = arrival_cost_parent + self.cost_move(goal,parent)
                OPEN[tuple(goal)] = (g_j + h_j, g_j)
                parents[tuple(goal)] = tuple(parent)
            for k in range(numofdirs):
                child = np.around(np.array(parent) + dR[:,k], decimals = 2)   # get child
                if tuple(child) not in CLOSED:
                    # Checking to see if child is valid or not
                    intersect = self.collision_detection(self.blocks, self.boundary, parent, child)
                    if intersect == True:
                        continue
                    # Here we have found a valid child, run weighted A*
                    g_j = arrival_cost_parent + self.cost_move(tuple(child),parent)
                    if tuple(child) not in OPEN or g_j < OPEN[tuple(child)][1]:
                        OPEN[tuple(child)] = (g_j + self.eps* self.heuristic(tuple(child),goal,self.res), g_j)
                        parents[tuple(child)] = parent

    