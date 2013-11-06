'''
Created on Oct 20, 2013

@author: Ofra
'''
from Action import Action
from ActionLayer import ActionLayer
from Pair import Pair
from Proposition import Proposition
from PropositionLayer import PropositionLayer

class PlanGraph(object):
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
                dont_add = False
                for pre1 in action.getPre():
                    for pre2 in action.getPre():
                        if Pair(pre1, pre2) in previousPropositionLayer.getMutexProps():
                            dont_add = True
                if not dont_add:
                    newActionLayer.addAction(action)
        # add mutex actions
        for action1 in newActionLayer.getActions():
            for action2 in newActionLayer.getActions():
                actionPair = Pair(action1, action2)
                if action1 != action2 and self.mutexActions(action1, action2, previousPropositionLayer.getMutexProps()) and actionPair not in newActionLayer.getMutexActions():
                    newActionLayer.addMutexActions(action1, action2)
       
        self.actionLayer = newActionLayer
        
        newPropositionLayer = PropositionLayer()
        for prop in allProps:
            dont_add = True
            for action in newActionLayer.getActions():
                if prop in action.getAdd():
                    dont_add = False
            if not dont_add:
                newPropositionLayer.addProposition(prop)
    
        # add mutex propositions
        for prop1 in newPropositionLayer.getPropositions():
            for prop2 in newPropositionLayer.getPropositions():
                propPair = Pair(prop1, prop2)
                if prop1 != prop2 and self.mutexPropositions(prop1, prop2, newActionLayer.getMutexActions()) and propPair not in newPropositionLayer.getMutexProps():
                    newPropositionLayer.addMutexProp(prop1, prop2)
        
        # set new proposition layer
        self.setPropositionLayer(newPropositionLayer)
                
    def mutexActions(self, a1, a2, mutexProps):
        '''YOUR CODE HERE: complete code for deciding whether actions a1 and a2 are mutex, given the previous proposition layer. Your exapnd function should call this function'''
        if Pair(a1, a2) not in self.independentActions:
            return True
        for pre1 in a1.getPre():
            for pre2 in a2.getPre():
                if Pair(pre1, pre2) in mutexProps:
                    return True
        return False
    
    def mutexPropositions(self, prop1, prop2, mutexActions):
        '''YOUR CODE HERE: complete code for deciding whether propositions p1 and p2 are mutex, given the previous proposition layer. Your exapnd function should call this function'''
        for producer1 in prop1.getProducers():
            for producer2 in prop2.getProducers():
                if Pair(producer1, producer2) not in mutexActions:
                    return False
        return True