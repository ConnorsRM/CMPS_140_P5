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
    Assault, Nommer, Defend = range(3)
  
  
  #store state
  state = States.Assault
  assaultTgt    = None
  isTop         = None
  nommerTarget  = None
  
  #Defense properties
  defIsOnRed      = None
  defMyTeamInd    = None
  defEnemyTeamInd = None
  #Defense Observations:
  defenseBeliefs  = None
  legalPositions  = None
  DEFABDEPTH      = 2
  defCurrentDepth = 0
  #Defense Patrol FIeld
  currentDirection = None
  initialAssault = True
  #Store which Ghosts are still Respawning
  

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
    
    #set redteamflag
    if(gameState.isOnRedTeam(self.index)):
      self.defIsOnRed = True
    else :
      self.defIsOnRed = False
    
    if (self.defIsOnRed):
      self.defMyTeamInd    = gameState.getRedTeamIndices()
      self.defEnemyTeamInd = gameState.getBlueTeamIndices()
    else :
      self.defMyTeamInd    = gameState.getBlueTeamIndices()
      self.defEnemyTeamInd = gameState.getRedTeamIndices()
    
    self.legalPositions = self.getLegalPositions(gameState)
    
    self.beginDefense(gameState)
    ''' 
    Your initialization code goes here, if you need any.
    '''
  
  def getSuccessor(self, gameState, action):
    successor = gameState.generateSuccessor(self.index, action)
    pos = successor.getAgentState(self.index).getPosition()
    if pos != nearestPoint(pos):
      return successor.generateSuccessor(self.index, action)
    else:
      return successor

  """
   ASSAULT AGENT:
   uses a greedy search agent with a mazedistance based simple
   heuristic to move accross border as soon as possible.
  """
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

  """
     DEFENSE AGENT
  """
  def getLegalPositions(self, gameState):
    thisInitialPos = gameState.getInitialAgentPosition(self.index)
    enemyInitialPos = gameState.getInitialAgentPosition(3 - self.index)
    maxX  = max(thisInitialPos[0], enemyInitialPos[0])
    maxY  = max(thisInitialPos[1], enemyInitialPos[1])
    
    #we can derive a list os positions from this,
    #just remove those where hasWall is true
    legalPos = []
    
    for xPos in range(1, maxX):
      for yPos in range(1, maxY):
        if not gameState.hasWall(xPos, yPos):
          legalPos += [(xPos, yPos)]
    
    return legalPos
    
  def believedPos(self, gameState):
    enemyTeamPositions = []
    
    for index in self.defEnemyTeamInd:
      enemyTeamPositions += [self.defenseBeliefs[index].argMax()]
    
    enemyAObs = gameState.getAgentPosition(self.defEnemyTeamInd[0])
    enemyBObs = gameState.getAgentPosition(self.defEnemyTeamInd[1])
    
    if enemyAObs:
      enemyTeamPositions[0] = enemyAObs
    
    if enemyBObs:
      enemyTeamPositions[1] = enemyBObs
    
    
    return enemyTeamPositions
  
  def beginDefense(self, gameState):
    #initialize beliefs uniformly
    self.defenseBeliefs = util.Counter()
    self.defenseBeliefs[self.defEnemyTeamInd[0]] = util.Counter()
    self.defenseBeliefs[self.defEnemyTeamInd[1]] = util.Counter()
    
    
    for pos in self.legalPositions:
      self.defenseBeliefs[self.defEnemyTeamInd[0]][pos] = 0.0001
      self.defenseBeliefs[self.defEnemyTeamInd[1]][pos] = 0.0001
    
    self.defenseBeliefs[self.defEnemyTeamInd[0]][gameState.getInitialAgentPosition(self.defEnemyTeamInd[0])] = 1.0
    self.defenseBeliefs[self.defEnemyTeamInd[1]][gameState.getInitialAgentPosition(self.defEnemyTeamInd[1])] = 1.0
    
    self.defenseBeliefs[self.defEnemyTeamInd[0]].normalize()
    self.defenseBeliefs[self.defEnemyTeamInd[1]].normalize()
    
    self.defCurrentDepth = 0
    
    
    return
  
  
  def DefenseAction(self, gameState):
    self.DefenseObserve(gameState)
    enemyPositions = self.believedPos(gameState)
    
    thisInitialPos = gameState.getInitialAgentPosition(self.index)
    enemyInitialPos = gameState.getInitialAgentPosition(3 - self.index)
    border = int(max(thisInitialPos[0],enemyInitialPos[0]))/2
    myState   = gameState.getAgentState(self.index)
    myPos     = myState.getPosition()
    evalFunction = self.DefensePatrolHeuristic
    
    #if stay at least 2 grids outside of spawn
    if(abs(myPos[0] - border) > 5):
      self.beginAssault(gameState)
      return self.AssaultAction(gameState)
    else :
      self.initialAssault = False
    
    for position in enemyPositions:
      #if closer to my spawn then theirs
      if(abs(position[0] - thisInitialPos[0]) < abs(position[0] - enemyInitialPos[0])):
        evalFunction = self.DefenseCatchHeuristic
    
    #now use selected heuristic to govern action
    actions = gameState.getLegalActions(self.index)
    if (Directions.STOP in actions):
      actions.remove(Directions.STOP)
    
    bestVal    = None
    bestAction = None
    for action in actions:
      if(bestAction == None):
        bestAction = action
        bestVal = evalFunction(gameState, action)
      else :
        thisVal = evalFunction(gameState, action)
        if (thisVal > bestVal):
          bestVal = thisVal
          bestAction = action
    
    
    return bestAction
  
  
  def DefensePatrolHeuristic(self, gameState, action):
    #get enemy Positions
    #if not in our territory, patrol
    #else capture
    newState = gameState.generateSuccessor(self.index, action)
    
    #ensure we're not clumping
    clumpMotive = 0.0
    
    aPos = newState.getAgentPosition(self.defMyTeamInd[0])
    bPos = newState.getAgentPosition(self.defMyTeamInd[1])
    
    clumpDist = self.getMazeDistance(aPos, bPos)
    
    if(clumpDist < 5):
      clumpMotive -= 0.5
    
    
    #stay away from spawn
    myState   = gameState.getAgentState(self.index)
    myPos     = myState.getPosition()
    initial   = gameState.getInitialAgentPosition(self.index)
    
    spawnMotive = util.manhattanDistance(myPos, initial)
    
    #one step look ahead

    return clumpMotive + spawnMotive

  def DefenseCatchHeuristic(self, gameState, action):
    thisInitialPos = gameState.getInitialAgentPosition(self.index)
    enemyInitialPos = gameState.getInitialAgentPosition(3 - self.index)
    enemyPositions = self.believedPos(gameState)
    myState   = gameState.getAgentState(self.index)
    myPos     = myState.getPosition()
    
    closestEnemy = enemyPositions[0]
    closestDist  = self.getMazeDistance(myPos, enemyPositions[0])
    
    for position in enemyPositions:
      if (abs(position[0] - thisInitialPos[0]) < abs(position[0] - thisInitialPos[0])):
        thisDist =   util.manhattanDistance(myPos, position)
        if thisDist < closestDist:    
          closestEnemy = position
          closestDist = thisDist
    #find closestEnemy
    
    print "DIST"
    print closestDist
    if(closestDist == 0): return float("inf")
    return -1.0*(closestDist)
  
  def DefenseEval(self, gameState):
    #we want to ensure superiority
    
    #   measures:
    #  1.) less food -> less superior
    #  2.) pacman is close to initial spawn than I am -> less superior
    
    foodWeight = 0.9
    posWeight  = 0.01
    
    foodMotive = 0
    
    #spin on turings algorithm
    #my food * value of food divided by their food * val of food
    #all foods the same, so lets just drop that
    myFood    = (len(self.getFoodYouAreDefending(gameState).asList()))
    enemyFood = (len(self.getFood(gameState).asList()))
    
    try:
      foodMotive = myFood/enemyFood
    except ZeroDivisionError:
      foodMotive = myFood
    
    
    enemyPositions = self.believedPos(gameState)
    myState   = gameState.getAgentState(self.index)
    myPos     = myState.getPosition()
    posMotive = 0
    myHome    = gameState.getInitialAgentPosition(self.index)
    
    for position in enemyPositions:
      if (position == None): continue
      if(abs(myHome[0] - position[0]) < abs(myHome[0] - myPos[0])):
        posMotive -= 1
    
    
    return (foodMotive * foodWeight) + (posMotive * posWeight)
  
  """
  def DefenseMaxVal(self, gameState, agent, alpha, beta):
    self.defCurrentDepth += 1
    if self.DefenseDepthCheck(gameState) :
      return [self.DefenseEval(gameState), Directions.STOP]
    
    v = [float("-inf"), Directions.STOP]
    
    actions = actions = gameState.getLegalActions(agent)

    if (Directions.STOP in actions):
      actions.remove(Directions.STOP)
    
    newAgent = agent + 1
    if(agent == gameState.getNumAgents() -1):
      newAgent = 0
    
    for thisAction in actions:
      newState = gameState.generateSuccessor(agent, thisAction)
      thisUtility = self.abVal(newState, newAgent, alpha, beta)[0]
      if thisUtility > v[0]:
        v = [thisUtility, thisAction]
      if v[0] >= beta:
        return v
      alpha = max(alpha, v[0])
    return v
  
  def DefenseEnemyLegalActions(self, gameState, agent):
    legalActions = [Directions.STOP]
    #get believed position
    enemyPos = self.defenseBeliefs[agent].argMax()
    print "-=-=-=-"
    print enemyPos
    print "********"
    #this is gross, but there is not other way of accessing
    #I can find.
    
    #lateral limits
    thisInitialPos = gameState.getInitialAgentPosition(self.index)
    enemyInitialPos = gameState.getInitialAgentPosition(3 - self.index)
    maxX = max(thisInitialPos[0], enemyInitialPos[0])
    maxY = max(thisInitialPos[1], enemyInitialPos[1])
    subOneX = enemyPos[0] - 1
    subOneY = enemyPos[1] - 1
    addOneX = enemyPos[0] + 1
    addOneY = enemyPos[1] + 1
    
    
    if (subOneX >= 1 and not gameState.hasWall(subOneX, enemyPos[1])):
      legalActions += [Directions.WEST]
    if (addOneX <= maxX and not gameState.hasWall(addOneX, enemyPos[1])):
      legalActions += [Directions.EAST]
    if (subOneY >= 1 and not gameState.hasWall(enemyPos[0], subOneY)):
      legalActions += [Directions.SOUTH]
    if (addOneY <= maxY and not gameState.hasWall(enemyPos[0], addOneY)):
      legalActions += [Directions.NORTH]
    
    
    return legalActions
  
  def abVal(self, gameState, agent, alpha, beta):
    if (agent in self.defEnemyTeamInd):
      return self.DefenseMinVal(gameState, agent, alpha, beta)
    else:
      return self.DefenseMaxVal(gameState, agent, alpha, beta)
  
  
  def DefenseMinVal(self, gameState, agent, alpha, beta):
    if self.DefenseDepthCheck(gameState) :
      return [self.DefenseEval(gameState), Directions.STOP]
    
    v = [float("inf"), Directions.STOP]
    
    actions = self.DefenseEnemyLegalActions(gameState, agent)
    
    for index in self.defEnemyTeamInd:
      print self.DefenseEnemyLegalActions(gameState, index)
      print "=-=-=-=-="
    print "================="
    if (Directions.STOP in actions):
      actions.remove(Directions.STOP)
    
    newAgent = agent + 1
    if(agent == gameState.getNumAgents() - 1):
      newAgent = 0
    
    for thisAction in actions:
      print agent
      print thisAction
      try :
        newState = gameState.generateSuccessor(agent, thisAction)
      except :
        newState = gameState
      
      thisUtility = self.abVal(newState, newAgent, alpha, beta)[0]
      
      if thisUtility < v[0]:
        v = [thisUtility, thisAction]
      
      if v[0] <= alpha:
        return v
      
      beta = min(beta, v[0])
    
    return v
  
  def DefenseDepthCheck(self, gameState):
    if ((self.defCurrentDepth / 2) >= self.DEFABDEPTH
      or (gameState.isOver())):                            return True
    
    return False
  
  """
  def DefenseObserve(self, gameState):
    
    noisyDistance = self.getCurrentObservation().getAgentDistances()
    
    
    #for each noisyDistance, use getDistanceProb
    #to obtain P(noisyDist | True)
    
    #start with first enemy
    enemyAAgentIndex = self.defEnemyTeamInd[0]
    enemyBAgentIndex = self.defEnemyTeamInd[1]
    
    newBelief = util.Counter()
    newBelief[self.defEnemyTeamInd[0]] = util.Counter()
    newBelief[self.defEnemyTeamInd[1]] = util.Counter()
    myState   = gameState.getAgentState(self.index)
    myPos     = myState.getPosition()
    
    for position in self.legalPositions:
      knownDistance = util.manhattanDistance(position, myPos)
      newProb = self.defenseBeliefs[enemyAAgentIndex][position] * gameState.getDistanceProb(noisyDistance[enemyAAgentIndex], knownDistance)
      newBelief[enemyAAgentIndex][position] = newProb
      newProb = self.defenseBeliefs[enemyBAgentIndex][position] * gameState.getDistanceProb(noisyDistance[enemyBAgentIndex], knownDistance)
      newBelief[enemyBAgentIndex][position] = newProb
    
    newBelief[enemyAAgentIndex].normalize()
    newBelief[enemyBAgentIndex].normalize()
    self.defenseBeliefs = newBelief

  
  def NommerAction(self, gameState):
    actions = gameState.getLegalActions(self.index)
    weights = self.getNommerWeights()
    bestAction = None
    bestVal    = float("-inf")
    
    for action in actions:
      features = self.NommerFeatures(gameState, action)
      thisVal = features * weights
      if (thisVal > bestVal) or (bestAction == None):
         bestVal = thisVal
         bestAction = action
    
    return bestAction
  
  def NommerFeatures(self, gameState, action):
    features  = util.Counter()
    features['bias'] = 1
    successor = self.getSuccessor(gameState, action)
    foodList = self.getFood(gameState).asList()
    myPos = successor.getAgentState(self.index).getPosition()
    features['successorScore'] = self.getScore(successor)
    
    if ((self.nommerTarget == None) or (self.nommerTarget not in foodList)):
      bestVal = float("inf")
      for food in foodList:
         thisVal = self.minSpanTreeWeight(foodList) + 3 * self.getMazeDistance(myPos, food)
         if (thisVal < bestVal):
            bestVal = thisVal
            self.nommerTarget = food
    
    if len(foodList) > 0: # This should always be True,  but better safe than sorry
      features['distanceToFood'] = self.getMazeDistance(myPos, self.nommerTarget)
      
    features['foodNet'] = self.minSpanTreeWeight(foodList)
    
    #Compute Total Ghost Distance
    capsuleList = gameState.getCapsules()
    distToSuperFood = 0
    getCapsule = False
    closestDistToGhost = float("inf")
    thisInitialPos = gameState.getInitialAgentPosition(self.index)
    enemyInitialPos = gameState.getInitialAgentPosition(3 - self.index)
    border = int(max(thisInitialPos[0],enemyInitialPos[0]))/2
    
    for enemy in self.getOpponents(gameState):
      ghostPos = gameState.getAgentState(enemy).getPosition()
      if (ghostPos != None):
         distToGhost = self.getMazeDistance(myPos, ghostPos)
         if (gameState.getAgentState(enemy).scaredTimer > 0):
            closestDistToGhost = min(closestDistToGhost, distToGhost)
         for capsule in capsuleList:
            if (((capsule[0] > thisInitialPos[0]) and (capsule[0] > border)) or
               ((capsule[0] < thisInitialPos[0]) and (capsule[0] < border))):
               distToSuperFood = self.getMazeDistance(capsule, myPos)
               if ((distToGhost <= 10) and (distToGhost > distToSuperFood)):
                  getCapsule = True
         
    if getCapsule:
      features['distToCapsule'] = distToSuperFood
      features['closestDistToGhost'] = closestDistToGhost
    
    for ally in self.getTeam(gameState):
      friendPos = gameState.getAgentState(ally).getPosition()
      distToAlly = self.getMazeDistance(myPos, friendPos)
      features['distToAlly'] = distToAlly
    
    return features
  
  def getNommerWeights(self):
    return {
    'successorScore': 10,
    #'foodNet': -5,
    'distanceToFood': -1,
    #'distToCapsule': -10,
    #'closestDistToGhost': -5,
    #'distToAlly': .5,
    #'bias': 1
    }
  
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
           food1_Dist[food2] = self.getMazeDistance(food1, food2)
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

  
  def determineState(self, gameState):
    #condition to switch from assault to guard/nom mode
    #here. called at front of chooseAction
    thisInitialPos = gameState.getInitialAgentPosition(self.index)
    enemyInitialPos = gameState.getInitialAgentPosition(3 - self.index)
    border = int(max(thisInitialPos[0],enemyInitialPos[0]))/2
    myPos = gameState.getAgentState(self.index).getPosition()
    
    self.state = self.States.Defend
    
    if (self.state == self.States.Assault):
      if (((myPos[0] > thisInitialPos[0]) and (myPos[0] > border)) or
            ((myPos[0] < thisInitialPos[0]) and (myPos[0] < border))):
         self.state = self.States.Nommer
    elif (self.state == self.States.Nommer):
      if (((myPos[0] > thisInitialPos[0]) and (myPos[0] < border)) or
            ((myPos[0] < thisInitialPos[0]) and (myPos[0] > border))):
         self.state = self.States.Assault
         
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
    self.determineState(gameState)
    
    if(self.state == self.States.Assault):
      return self.AssaultAction(observation)
    elif (self.state == self.States.Nommer):
      return self.NommerAction(observation)
    elif (self.state == self.States.Defend):
      return self.DefenseAction(observation)
    
    return random.choice(actions)
    