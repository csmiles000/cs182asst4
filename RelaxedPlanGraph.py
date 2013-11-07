'''
Created on Oct 20, 2013

@author: Ofra
'''
from Action import Action
from ActionLayer import ActionLayer
from Pair import Pair
from Proposition import Proposition
from PropositionLayer import PropositionLayer

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
    
    def getActionLayer(self):
        return self.actionLayer
    
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