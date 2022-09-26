import sys
from constants import *
from environment import *
from state import State
import queue
import math

class Solver:

    def __init__(self, environment, loop_counter):
        self.environment = environment
        self.loop_counter = loop_counter

    def solve_ucs(self):
        print(self.environment.widget_types)
        startState = StateWithCost(self.environment.get_init_state(),0)
        pQueue = queue.PriorityQueue()
        pQueue.put(startState)
        explored = set()
        parents = {
        }
        parents[startState.state] = (0, None, None)
        while pQueue.qsize() > 0:
            self.loop_counter.inc()
            currentState = pQueue.get().state
            if self.environment.is_solved(currentState):
                    path = [parents[currentState][2]]
                    prev_node = parents[currentState][1]
                    while prev_node != startState.state:
                        parent = parents[prev_node]
                        path.append(parent[2])  
                        prev_node = parent[1]
                    print(pQueue.qsize())
                    print(len(explored))
                    return reversed(path)
            successors = []
            for a in ROBOT_ACTIONS:
                possibleAction = self.environment.perform_action(currentState, a)
                if possibleAction[0]:
                    successors.append((possibleAction[1],possibleAction[2], a))
            for s in successors:
                if s[1] not in explored or (s[0] + parents[currentState][0] < parents[s[1]][0]):
                    parents[s[1]] = (s[0]+parents[currentState][0], currentState, s[2])
                    pQueue.put(StateWithCost(s[1],s[0]+parents[currentState][0]))
                    explored.add(s[1])

    def solve_a_star(self):
        startState = StateWithCost(self.environment.get_init_state(),0)
        pQueue = queue.PriorityQueue()
        pQueue.put(startState)
        explored = set()
        parents = {
        }
        parents[startState.state] = (0, None, None)
        while pQueue.qsize() > 0:
            self.loop_counter.inc()
            currentState = pQueue.get().state
            if self.environment.is_solved(currentState):
                    path = [parents[currentState][2]]
                    prev_node = parents[currentState][1]
                    while prev_node != startState.state:
                        child = parents[prev_node]
                        path.append(child[2])  
                        prev_node = child[1]
                    print(pQueue.qsize())
                    print(len(explored))
                    return reversed(path)
            successors = []
            for a in ROBOT_ACTIONS:
                possibleAction = self.environment.perform_action(currentState, a)
                if possibleAction[0] :
                    successors.append((possibleAction[1],possibleAction[2], a))
            for s in successors:
                if s[1] not in explored or (s[0] + parents[currentState][0] < parents[s[1]][0]):
                    parents[s[1]] = (s[0]+parents[currentState][0], currentState, s[2])
                    pQueue.put(StateWithCost(s[1],s[0]+parents[currentState][0] + self.calculateHeuristicTargetsCovered(s[1]))) 
                    explored.add(s[1])
    
    #Returns euclidean distance from closest widget to the widgets closest target
    def calculateHeuristicEuclidean(self,state):
        closeTarget = 99999
        for i in range(len(state.widget_centres)):
            for j in range(len(self.environment.target_list)):
                closeTarget = min(closeTarget, math.sqrt(abs(self.environment.target_list[j][0] - state.widget_centres[i][0])**2
                + abs(self.environment.target_list[j][1] - state.widget_centres[i][1]))**2)
        return closeTarget*1.5

    
    #Returns manhattan distance from all widgets to their nearest target
    def calculateHeuristicManhattan(self, state):
        distance = 0
        for i in range(len(state.widget_centres)):
            distance += (abs(self.closestTarget
            (state, state.widget_centres[i])[0] - state.widget_centres[i][0]) 
            + abs(self.closestTarget(state, state.widget_centres[i])[1] - state.widget_centres[i][1]))*1.5
        return distance

    #Returns how many of the targets are covered by the widgets
    def calculateHeuristicTargetsCovered(self, state):
        #From environment.py:
        widget_cells = [widget_get_occupied_cells
        (self.environment.widget_types[i], state.widget_centres[i],
        state.widget_orients[i]) for i in range(self.environment.n_widgets)][0]

        widgetsCoveringTarget = 0
        for i in range(len(widget_cells)):
            if widget_cells[i] in self.environment.target_list:
                widgetsCoveringTarget += 1
        return (len(self.environment.target_list) - widgetsCoveringTarget)/3
   
    #Returns distance from all widget-centers to a target which fits the widget
    def calculateHeuristicFindTarget(self, state):
        distance = 0
        targetSet = set(self.environment.target_list)
        for j in range(len(self.environment.widget_types)):
            if self.environment.widget_types[j] == '5':
                for i in range(len(self.environment.target_list)):
                    target = self.environment.target_list[i]
                    if (get_adjacent_cell_coords(target,'U.') in targetSet \
                    and get_adjacent_cell_coords(target,'UR') in targetSet \
                    and get_adjacent_cell_coords(target,'DL') in targetSet \
                    and get_adjacent_cell_coords(target,'D') in targetSet) \
                    or (get_adjacent_cell_coords(target,'DL') in targetSet \
                    and get_adjacent_cell_coords(target,'UL') in targetSet \
                    and get_adjacent_cell_coords(target,'UR') in targetSet \
                    and get_adjacent_cell_coords(target,'DR') in targetSet) \
                    or (get_adjacent_cell_coords(target,'U.') in targetSet \
                    and get_adjacent_cell_coords(target,'UL') in targetSet \
                    and get_adjacent_cell_coords(target,'D.') in targetSet \
                    and get_adjacent_cell_coords(target,'DR') in targetSet):
                            distance += abs(target[0] - state.widget_centres[j][0]) 
                            + abs(target[1] - state.widget_centres[j][1])

            if self.environment.widget_types[j] == '4':
                for i in range(len(self.environment.target_list)):
                    target = self.environment.target_list[i]
                    if (get_adjacent_cell_coords(target,'U.') in targetSet \
                    and get_adjacent_cell_coords(target,'DL') in targetSet \
                    and get_adjacent_cell_coords(target,'DR') in targetSet) \
                    or (get_adjacent_cell_coords(target,'D') in targetSet \
                    and get_adjacent_cell_coords(target,'UL') in targetSet \
                    and get_adjacent_cell_coords(target,'UR') in targetSet):
                        distance += abs(target[0] - state.widget_centres[j][0]) 
                        + abs(target[1] - state.widget_centres[j][1])
            if self.environment.widget_types[j] == '3':
                for i in range(len(self.environment.target_list)):
                    target = self.environment.target_list[i]
                    if (get_adjacent_cell_coords(target,'U.') in targetSet \
                    and get_adjacent_cell_coords(target,'D.') in targetSet) \
                    or (get_adjacent_cell_coords(target,'DL') in targetSet \
                    and get_adjacent_cell_coords(target,'UR') in targetSet) \
                    or (get_adjacent_cell_coords(target,'UL') in targetSet \
                    and get_adjacent_cell_coords(target,'DR') in targetSet): 
                        distance += abs(target[0] - state.widget_centres[j][0]) 
                        + abs(target[1] - state.widget_centres[j][1])
        return distance*1.5

    #Return closest target to widget center
    def closestTarget(self, state, widget_center):
        distance = None
        for i in range(len(self.environment.target_list)):
            if distance == None:
                distance = abs(widget_center[0] - state.environment.target_list[i][0]) 
                + abs(widget_center[1] - state.environment.target_list[i][1])
                x, y = state.environment.target_list[i]
            elif abs(widget_center[0] - state.environment.target_list[i][0]) \
            + abs(widget_center[1] - state.environment.target_list[i][1]) < distance:
                x, y = state.environment.target_list[i]
                distance = abs(widget_center[0] - state.environment.target_list[i][0]) \
                + abs(widget_center[1] - state.environment.target_list[i][1])
        return x, y

    #Return closest widgetCenter
    def closestWidget(self, state):
        distance = None
        for i in range(len(state.widget_centres)):
            if distance == None:
                distance = abs(state.widget_centres[i][0] - state.robot_posit[0]) + abs(state.widget_centres[i][1] - state.robot_posit[1])
                x, y = state.widget_centres[i]
            elif abs(state.widget_centres[i][0] - state.robot_posit[0]) + abs(state.widget_centres[i][1] - state.robot_posit[1]) < distance:
                x, y = state.widget_centres[i]
                distance = abs(state.widget_centres[i][0] - state.robot_posit[0]) + abs(state.widget_centres[i][1] - state.robot_posit[1])
        return x, y

class StateWithCost:
    def __init__(self, state, cost):
        self.state = state
        self.cost = cost
        
    def __lt__(self, other):
        return self.cost < other.cost

    def __eq__(self, other):
        if not isinstance(other, StateWithCost):
            return False
        return (self.state == other.state and
                self.cost == self.cost)
    def __hash__(self):
        return hash((self.state, self.cost))
    
