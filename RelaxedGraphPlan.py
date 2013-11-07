from PropositionLayer import PropositionLayer
from RelaxedPlanGraph import RelaxedPlanGraph
from Action import Action
from copy import deepcopy

class RelaxedGraphPlan(object):
    '''
    A class for initializing and running the graphplan algorithm
    '''

    def __init__(self, initialState, goal, propositions, actions):
        '''
        Constructor
        '''
        self.graph = []
        self.actions = deepcopy(actions)
        self.propositions = deepcopy(propositions)
        self.initialState = deepcopy(initialState)
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
            level = level +1
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
