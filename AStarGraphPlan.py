import util
from collections import deque
from GraphPlan import GraphPlan
from Parser import Parser
from PlanGraph import PlanGraph
from Pair import Pair
from Action import Action

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
    if len(state) != len(self.goal):
      return False
    return all([s in self.goal for s in state]) and all(s in state for s in self.goal)

  def getSuccessors(self, state):
    successors = []
    for action in self.actions:
      if all(pre in state for pre in action.getPre()):
        successor = state + action.getAdd() #adds
        successor = [p for p in successor if p not in action.getDelete()] #deletes
        successors.append((successor, action))
    return successors

  def getCostOfActions(self, actions):
    return len(actions)

def gpHeuristic(state, problem=None):
  """
  A heuristic function estimates the cost from the current state to the nearest
  goal in the provided SearchProblem.  This heuristic is trivial.
  """
  return 0

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
    # skip node if we have already explored it
    for e in explored:
      if all(p in e for p in currentState) and all(p in currentState for p in e):
        continue
    else:
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
          (state, action) = (successors[0], successors[1])
          node = Node(state=state, parent=currentNode, prevAction=action, pathCost=1)
          fringe.push(node)
    
if __name__ == '__main__':
    domain = 'dwrDomain.txt'
    problem = 'dwrProblem.txt'
#    gp = GraphPlan(domain, problem)
    problem = DwrProblem(domain, problem)
    print aStarSearch(problem, gpHeuristic)

