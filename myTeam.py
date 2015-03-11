# myTeam.py
# ---------
# Licensing Information: Please do not distribute or publish solutions to this
# project. You are free to use and extend these projects for educational
# purposes. The Pacman AI projects were developed at UC Berkeley, primarily by
# John DeNero (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# For more info, see http://inst.eecs.berkeley.edu/~cs188/sp09/pacman.html

from captureAgents import CaptureAgent
import random, time, util
from util import nearestPoint
from game import Directions
import game

#################
# Team creation #
#################

def createTeam(firstIndex, secondIndex, isRed,
               first = 'DummyAgent', second = 'DummyAgent'):
  """
  This function should return a list of two agents that will form the
  team, initialized using firstIndex and secondIndex as their agent
  index numbers.  isRed is True if the red team is being created, and
  will be False if the blue team is being created.

  As a potentially helpful development aid, this function can take
  additional string-valued keyword arguments ("first" and "second" are
  such arguments in the case of this function), which will come from
  the --redOpts and --blueOpts command-line arguments to capture.py.
  For the nightly contest, however, your team will be created without
  any extra arguments, so you should make sure that the default
  behavior is what you want for the nightly contest.
  """

  # The following line is an example only; feel free to change it.
  return [eval(first)(firstIndex), eval(second)(secondIndex)]

##########
# Agents #
##########

class DummyAgent(CaptureAgent):
  """
  A Dummy agent to serve as an example of the necessary agent structure.
  You should look at baselineTeam.py for more details about how to
  create an agent as this is the bare minimum.
  """
  
  """
  Assault engine is a stupid agent that simply finds the swiftest
  path into enemy territory and executes it
  
  Later modifcation possibility
  Find best area to assault?
  """
  #add more states with comma after Assault
  class States:
    Assault = range(1)
  
  
  #store state
  state = States.Assault
  assaultTgt = None
  
  #add more states with comma after Assault
  class States:
    Assault = range(1)
    

  def getSuccessor(self, gameState, action):
    successor = gameState.generateSuccessor(self.index, action)
    pos = successor.getAgentState(self.index).getPosition()
    if pos != nearestPoint(pos):
      return successor.generateSuccessor(self.index, action)
    else:
      return successor
  
  #return the best available action in assault mode
  #using a simple greedy search to be augmented later
  def beginAssault(self, gameState):
    #just for testing, this can be removed
    if self.assaultTgt != None:
      return
    
    features  = util.Counter()
    myState   = gameState.getAgentState(self.index)
    myPos     = myState.getPosition()
    
    #Find Border, no direct access, have to calculate it
    thisInitialPos = gameState.getInitialAgentPosition(self.index)
    enemyInitialPos = gameState.getInitialAgentPosition(3 - self.index)
    border = int(max(thisInitialPos[0],enemyInitialPos[0]))/2
    
    #assume linear path possible and set as target,
    #it isn't but this can be tweaked later
    
    newTarget = [myPos[0], myPos[1]]
    
    #determine correct side of border to find
    if(myPos[0] > border):
      newTarget[0] = border+1
    else:
      newTarget[0] = border-1
    
    self.assaultTgt = (newTarget[0], newTarget[1])
    
    print self.assaultTgt
    
    
  def AssaultAction(self, gameState):
    self.beginAssault(gameState)
    actions = gameState.getLegalActions(self.index)
    bestAction = None
    bestVal    = float("-inf")
    
    for action in actions:
      thisValue = self.AssaultFeatures(gameState, action)
      if(thisValue > bestVal or bestAction == None):
        bestAction = action
        bestVal = thisValue
    
    return bestAction
  
  #returns a value representing the utility
  #of features observed in the game state
  #in this early example, it's just the
  #reciprocal of the closest food distance
  def AssaultFeatures(self, gameState, action):
    features  = util.Counter()
    successor = self.getSuccessor(gameState, action)
    myState   = successor.getAgentState(self.index)
    myPos     = myState.getPosition()

    #we want to minimize distance from pacman to
    #border
    try :
      return 1.0/self.getMazeDistance(myPos,self.assaultTgt)
    except ZeroDivisionError:
      return 1.0

  def registerInitialState(self, gameState):
    """
    This method handles the initial setup of the
    agent to populate useful fields (such as what team
    we're on). 
    
    A distanceCalculator instance caches the maze distances
    between each pair of positions, so your agents can use:
    self.distancer.getDistance(p1, p2)

    IMPORTANT: This method may run for at most 15 seconds.
    """

    ''' 
    Make sure you do not delete the following line. If you would like to
    use Manhattan distances instead of maze distances in order to save
    on initialization time, please take a look at
    CaptureAgent.registerInitialState in captureAgents.py. 
    '''
    CaptureAgent.registerInitialState(self, gameState)
    self.beginAssault(gameState)
    ''' 
    Your initialization code goes here, if you need any.
    '''


  def chooseAction(self, gameState):
    """
    Picks among actions randomly.
    """
    actions = gameState.getLegalActions(self.index)

    ''' 
    You should change this in your own agent.
    '''
    
    
    if(self.state == self.States.Assault):
      return self.AssaultAction(gameState)
    
    return random.choice(actions)

