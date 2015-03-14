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
import itertools
import game

#############
# Tree Vars #
#############

minSpanTreeDict = {(): 0}
  
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
    Assault, Nommer = range(2)
  
  
  #store state
  state = States.Nommer
  assaultTgt    = None
  isTop         = None
  
  #Store which Ghosts are still Respawning
  

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
    
    #determine who's top and who's bottom
    
    #first get team info
    teamIndices = None
    teamMateIndex = None
    
    if(gameState.isOnRedTeam(self.index)):
      teamIndices = gameState.getRedTeamIndices()
    else:
      teamIndices = gameState.getBlueTeamIndices()
    
    teamIndices.remove(self.index)
    teamMateIndex = teamIndices[0]
    teamMateState = gameState.getAgentState(teamMateIndex)
    teamMatePos   = teamMateState.getPosition()
    
    #if equal y cases, why not randomize to keep enemy learning minimal
    if myPos[1] > teamMatePos[1]:
      self.isTop = True
    else:
      self.isTop = False
    
    #Find Border, no direct access, have to calculate it
    thisInitialPos = gameState.getInitialAgentPosition(self.index)
    enemyInitialPos = gameState.getInitialAgentPosition(3 - self.index)
    border = int(max(thisInitialPos[0],enemyInitialPos[0]))/2
    
    #assume linear path possible and set as target,
    #it isn't but this can be tweaked later
    
    newTarget = [enemyInitialPos[0], enemyInitialPos[1]]
    
    #determine correct side of border to find
    if(self.isTop):
      newTarget[1] = thisInitialPos[1]
      
    
    self.assaultTgt = (newTarget[0], newTarget[1])
    
    
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

    #weights
    #give it just the smallest cookie crumb for y pos incentive
    yWeight = 0.000001
    distWeight = 1.0
    
    yProxFeat = 0;
    tgtDistFeat = 0;
    #make agent favor it's Lane... I hate dota, and therefore
    #this term angers me ><
    laneDist = abs(self.assaultTgt[1]-myPos[1])
    if (laneDist != 0):
      yProxFeat = 1.0/laneDist
    else:
      yProxFeat = 1.0
    
    tgtDistFeat = self.getMazeDistance(myPos,self.assaultTgt)
    
    if (tgtDistFeat != 0):
      tgtDistFeat = 1.0/self.getMazeDistance(myPos,self.assaultTgt)
    else:
      tgtDistFeat = 1.0
    
    return (yWeight * yProxFeat) + (distWeight * tgtDistFeat)
  
  def NommerAction(self, gameState):
    actions = gameState.getLegalActions(self.index)
    weights = self.getNommerWeights()
    bestAction = None
    bestVal    = float("-inf")
    
    for action in actions:
      features = self.NommerFeatures(gameState, action)
      thisVal = features * weights
      if (thisVal > bestVal):
         bestVal = thisVal
         bestAction = action
    
    return bestAction
  
  def NommerFeatures(self, gameState, action):
    features  = util.Counter()
    features['bias'] = 1
    successor = self.getSuccessor(gameState, action)
    features['successorScore'] = self.getScore(successor)
    
    # Compute distance to the nearest food
    foodList = self.getFood(successor).asList()
    if len(foodList) > 0: # This should always be True,  but better safe than sorry
      myPos = successor.getAgentState(self.index).getPosition()
      minDistance = min([self.getMazeDistance(myPos, food) for food in foodList])
      features['distanceToFood'] = minDistance
      
    features['foodNet'] = self.minSpanTreeWeight(foodList)
    """
    #Compute Total Ghost Distance        THIS DOESN'T WORK YET
    totalDistToGhost = 0
    closestDistToGhost = float("inf")
    for enemy in self.getOpponents(gameState):
      print gameState.getAgentState(enemy)
      ghostPos = gameState.getAgentState(enemy).getPosition()
      distToGhost = self.getMazeDistance(myPos, ghostPos)
      totalDistToGhost += distToGhost
      closestDistToGhost = min(closestDistToGhost, distToGhost)
      
    features['distanceToGhost'] = totalDistToGhost
    features['closestDistToGhost'] = closestDistToGhost
    """
    
    return features
  
  def getNommerWeights(self):
    return {
    'successorScore': 10,
    'foodNet': -2,
    'distanceToFood': -1,
    'distanceToGhost': 3,
    'closestDistToGhost': 5,
    'bias': 0}
  
  def minSpanTreeWeight(self, foodList):
     """
     Calculates and returns the weight of the minSpan tree denoted by
     the food in the list foodList. Each food must be a tuple storing
     the position of the food.
     """
     
     foodTuple = tuple(foodList)
     closestDot = [-1, -1]
     result = 0
     
     #See if the minSpanManTree has already been calculated...
     #...if not, find / record the weight of the Minimum Spanning Manhattan Tree
     if foodTuple not in minSpanTreeDict:
       #Calculate MDistances between Food
       foodDists = {}
       for food1 in foodList:
         food1_Dist = {}
         for food2 in foodList:
           food1_Dist[food2] = self.distancer.getDistance(food1, food2)
         foodDists[food1] = food1_Dist
      
       #Build Tree
       minSpanManTree = set()
       minSpanManTree.add(foodList[0])
       foodList.remove(foodList[0])
       treeWeight = 0
       while (len(foodList) > 0):
         #Find min dist from FoodInTree to FoodNotInTree
         closestFoodToTree = [-1, [-1, -1]]
         for FoodInTree in minSpanManTree:
           for FoodNotInTree in foodList:
             tempDist = foodDists[FoodInTree][FoodNotInTree]
             if ((closestFoodToTree[0] == -1) or (tempDist < closestFoodToTree[0])):
               closestFoodToTree = [tempDist, FoodNotInTree]
         
         #Add FoodNotInTree to Tree
         minSpanManTree.add(tuple(closestFoodToTree[1]))
         foodList.remove(closestFoodToTree[1])
         
         #Add Dist to weight
         treeWeight += closestFoodToTree[0]
       minSpanTreeDict[foodTuple] = treeWeight
     else:
       treeWeight = minSpanTreeDict[foodTuple]
     
     #Return the weight of the tree
     return treeWeight

  
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
    
    foodList = self.getFood(gameState).asList()
    if tuple(foodList) not in minSpanTreeDict:
      self.minSpanTreeWeight(foodList)
    
    self.beginAssault(gameState)
    ''' 
    Your initialization code goes here, if you need any.
    '''
  
  def determineState(self, gameState):
    #condition to switch from assault to guard/nom mode
    #here. called at front of chooseAction
    return
  
  def chooseAction(self, gameState):
    """
    Picks among actions randomly.
    """
    actions = gameState.getLegalActions(self.index)
    observation = self.getCurrentObservation()
    ''' 
    You should change this in your own agent.
    '''
    
    
    if(self.state == self.States.Assault):
      return self.AssaultAction(observation)
    elif (self.state == self.States.Nommer):
      return self.NommerAction(observation)
    
    return random.choice(actions)
  