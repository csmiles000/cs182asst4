import util
from collections import deque
from Parser import Parser
from PropositionLayer import PropositionLayer
from Action import Action
from copy import deepcopy
from ActionLayer import ActionLayer
from Pair import Pair
from Proposition import Proposition
from PropositionLayer import PropositionLayer

# data structure used to store states, parents, actions, and cost
class Node:
  '''
  data structure to be used for all of the search algorithms.
  '''
  def __init__(self, state, parent=None, pathCost=1, prevAction=None):
    self.state = state
    self.parent = parent
    if parent:
      self.totalCost = parent.totalCost + pathCost
    else:
      self.totalCost = 0
    self.prevAction = prevAction

  def getState(self):
    return self.state

  def getParent(self):
    return self.parent

  def getPrevAction(self):
    return self.prevAction

  def getTotalCost(self):
    return self.totalCost


class DwrProblem:
  def __init__(self, domain, problem):
    p = Parser(domain, problem)
    domainKB = p.parseActionsAndPropositions();
    self.actions = domainKB[0]
    self.propositions = domainKB[1]
    prob = p.pasreProblem()
    self.initialState = prob[0]
    self.goal = prob[1]

  def getStartState(self):
    return self.initialState

  def isGoalState(self, state):
    for goal in self.goal:
      if goal not in state:
        return False
    return True

  def getSuccessors(self, state):
    successors = []
    for action in self.actions:
      if all(pre in state for pre in action.getPre()):
        successor = state + action.getAdd() #adds
        successor = [p for p in successor if p not in action.getDelete()] #deletes
        successors.append((successor, action))
    return successors

def gpHeuristic(state, problem=None):
  """
  A heuristic function estimates the cost from the current state to the nearest
  goal in the provided SearchProblem.  This heuristic is trivial.
  """
  rgp = RelaxedGraphPlan(state, problem.goal, problem.propositions, problem.actions)
  return rgp.level

def aStarSearch(problem, heuristic=gpHeuristic):
  "Search the node that has the lowest combined cost and heuristic first."
  "*** YOUR CODE HERE ***"
  # f(x) = g(x) + h(x). f(x) is stored in the fringe.
  def costPlusHeuristic(node):
    g = node.getTotalCost()
    state = node.getState()
    h = heuristic(state, problem)
    return g + h

  fringe = util.PriorityQueueWithFunction(costPlusHeuristic) # stores NODES that we encounter
  firstNode = Node(state=problem.getStartState())
  fringe.push(firstNode)

  explored = [] # stores STATES that have been explored

  while not fringe.isEmpty(): 
    currentNode = fringe.pop()
    currentState = currentNode.getState()
    isNew = True
    # skip node if we have already explored it
    for e in explored:
      if all([p in e for p in currentState]):
        isNew = False
    if isNew:
      # if at a goal state, backtrace parent nodes to get complete path taken to goal
      if problem.isGoalState(currentState):
        path = deque()
        while currentNode.getParent():
          path.appendleft(currentNode.getPrevAction())
          currentNode = currentNode.getParent()
        return path
      # if not at goal state, mark current state as explored and add successors to fringe
      else:
        explored.append(currentState)
        for successors in problem.getSuccessors(currentState):
          (successor, action) = (successors[0], successors[1])
          node = Node(state=successor, parent=currentNode, prevAction=action, pathCost=1)
          fringe.push(node)

class RelaxedGraphPlan(object):
    '''
    A class for initializing and running the relaxed graphplan algorithm
    '''

    def __init__(self, initialState, goal, propositions, actions):
        '''
        Constructor
        '''
        self.graph = []
        self.actions = deepcopy(actions)
        self.propositions = deepcopy(propositions)
        self.initialState = initialState
        self.goal = goal
        self.createNoOps() #creates noOps that are used to propogate existing propositions from one layer to the next
        self.level = self.graphplan() #calls graphplan

    '''the graphplan algorithm. If it's easier for you to write your own code, go for it. But you may use this. The code calls the extract function which you should complete below '''    
    def graphplan(self):
        
        #initialization
        initState = self.initialState
        goalState = self.goal
        actions = self.actions
        level = 0
        
        #create first layer of the graph, note it only has a proposition layer which consists of the initial state.
        propLayerInit = PropositionLayer()
        for prop in initState:
            propLayerInit.addProposition(prop)
        pgInit = RelaxedPlanGraph(0, actions)
        pgInit.setPropositionLayer(propLayerInit)
        self.graph.append(pgInit)
        
        '''while the layer does not contain all of the propositions in the goal state, or some of these propositions are mutex in the layer we, and we have not reached the fixed point, continue expanding the graph'''
        while((self.goalStateNotInPropLayer(goalState, self.graph[level].getPropositionLayer().getPropositions()))):
            level = level + 1
            pgNext = RelaxedPlanGraph(level, self.actions) #create new PlanGraph object
            pgNext.expand(self.graph[level-1], self.propositions, self.actions) #calls the expand function, which you are implementing in the PlanGraph class
            self.graph.append(deepcopy(pgNext)) #appending the new level to the plan graph

        return level
              
    '''helper function that checks whether all propositions of the goal state are in the current graph level'''
    def goalStateNotInPropLayer(self, goalState, propositions):
        for goal in goalState:
            if goal not in propositions:
                return True
        return False

    '''creates the noOps that are used to propogate propositions from one layer to the next'''
    def createNoOps(self):
        for prop in self.propositions:
            name = prop.name
            precon = []
            add = []
            precon.append(prop)
            add.append(prop)
            delete = []
            act = Action(name,precon,add,delete)
            self.actions.append(act)
            prop.addProducer(act)

class RelaxedPlanGraph(object):
    '''
    A class for representing a level in the plan graph. For each level i, the PlanGraph consists of the actionLayer and propositionLayer at this level
    '''

    def __init__(self, level, independentActions):
        '''
        Constructor
        '''
        self.level = level
        self.independentActions = independentActions # a list of the independent actions (this would be the same at each level)
        self.actionLayer = ActionLayer()
        self.propositionLayer = PropositionLayer();
    
    def getPropositionLayer(self):
        return self.propositionLayer
    
    def setPropositionLayer(self, propLayer):
        self.propositionLayer = propLayer    
    
    def expand(self, previousLevel, allProps, allActions): # you can change the params the function takes if you like
        previousPropositionLayer = previousLevel.getPropositionLayer()
        newActionLayer = ActionLayer()
        
        for action in allActions:
            if previousPropositionLayer.allPrecondsInLayer(action):
                    newActionLayer.addAction(action)
        self.actionLayer = newActionLayer
        
        newPropositionLayer = PropositionLayer()
        for prop in allProps:
            if newActionLayer.effectExists(prop):
                newPropositionLayer.addProposition(prop)
        # set new proposition layer
        self.setPropositionLayer(newPropositionLayer)


if __name__ == '__main__':
    domain = 'dwrDomain.txt'
    problem = 'dwrProblem.txt'
    problem = DwrProblem(domain, problem)
    plan = aStarSearch(problem, gpHeuristic)
    if plan:
      for a in plan:
        print a
    else:
      "IMPOSSIBRU"

