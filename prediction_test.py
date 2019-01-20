#from prediction import nextTickBallPos

import json
from prediction import BallPredictor, distanceBetweenCenters, ALLOWED_CALCULATIONS_ERROR
from model import Rules, Ball
from vectors import Vector2D, Vector3D

from prediction import solveSquareEquation

def ballFromLog(ballLogForm):
  ballD = {}
  ballD['x'] = ballLogForm['position']['x']
  ballD['y'] = ballLogForm['position']['y']
  ballD['z'] = ballLogForm['position']['z']
  ballD['velocity_x'] = ballLogForm['velocity']['x']
  ballD['velocity_y'] = ballLogForm['velocity']['y']
  ballD['velocity_z'] = ballLogForm['velocity']['z']
  ballD['radius'] = ballLogForm['radius']
  ball = Ball(ballD)
  return ball

#inQuestion = [1364, 1365, 1366, 1457, 2574, 2888]

gameLog = [json.loads(s) for s in open('half_pipe.log', 'rt')]

rulesDct = gameLog[0]
gameLog = gameLog[1:]
rules = Rules(rulesDct)

ballPred = BallPredictor(rules)

#basicMoment = gameLog[106]
basicMoment = gameLog[81]
basicBall = ballFromLog(basicMoment['ball'])

for e in gameLog[245:390]:
##for e in [gameLog[196],
##          gameLog[260],
##          gameLog[270],
##          gameLog[273],
##          gameLog[276]]:
  ball = ballFromLog(e['ball'])
  ticks = e['current_tick'] - basicMoment['current_tick']
  predictedBall = ballPred.predictedBall(basicBall, ticks)
  delta = distanceBetweenCenters(ball, predictedBall)
  print(e['current_tick'], delta)
  #print(ball.x, ball.y, ball.z)
  #print(predictedBall.x, predictedBall.y, predictedBall.z)

  
  '''
  ball = ballFromLog(e['ball'])
  pt = Vector3D(ball.x, ball.y, ball.z)
  dst, norm = ballPred.distanceToArena(pt)
  print('  == ', e['current_tick'], ' ==')
  print(dst, '   ', norm.x, norm.y, norm.z)
  print(ball.x, ball.y, ball.z)
  '''


'''
for e in gameLog[650:660]:
  print(e['current_tick'], e['ball']['velocity']['x'],
        e['ball']['velocity']['z'])
print()
for e in gameLog[703:715]:
  print(e['current_tick'], e['ball']['position']['y'],
        e['ball']['velocity']['y'])
print()
for e in gameLog[760:770]:
  print(e['current_tick'], e['ball']['velocity']['x'],
        e['ball']['velocity']['z'])
'''

'''
e = gameLog[0]

bot = e['robots'][0]
bot = ballFromLog(bot)
#ball = ballFromLog(e['ball'])
print(bot.x, bot.y, bot.z)
dirVec = Vector3D(1.0, 1.0, 50.0)
botVec = Vector3D(bot.x, bot.y, bot.z)
testVec = Vector3D(1.0, -9.0, 2.0)
projV = testVec.projectOn(dirVec)
print(projV.x, projV.y, projV.z)
'''
"""

basicMoment = gameLog[876]
basicBall = ballFromLog(basicMoment['ball'])

for e in gameLog[877:1152]:
  #print(e['current_tick'])

  timeInterv = (e['current_tick'] - basicMoment['current_tick']) * ballPred.tik
  ball = ballFromLog(e['ball'])
  #print('a', basicBall.y)
  predictedBall = ballPred.predictedBall(basicBall, timeInterv)
  delta = distanceBetweenCenters(ball, predictedBall)
  if (delta > 0.0001):
    print("LARGE DIVERGENCE AT ", e['current_tick'], delta)
    #print(predictedBall.x, predictedBall.z)
    #print(ball.x, ball.z)
    #basicMoment = e
    #basicBall = ballFromLog(basicMoment['ball'])

  
  '''
  timeInterv = (e['current_tick'] - basicMoment['current_tick']) * ballPred.tik
  ball = ballFromLog(e['ball'])
  predictedBall = ballPred.predictedBallWoColls(basicBall, timeInterv)
  delta = distanceBetweenCenters(ball, predictedBall)
  if (delta > ALLOWED_CALCULATIONS_ERROR):
    print("LARGE DIVERGENCE AT ", e['current_tick'], delta)
  basicMoment = e
  basicBall = ballFromLog(basicMoment['ball'])
  '''










predictedBall = None
prevBall = None
start = 0
end = 0
periods = []
#for e in gameLog[:1000]:
for ix in range(len(gameLog)):
#for ix in range(1000):
  e = gameLog[ix] #adapter
  
  ball = ballFromLog(e['ball'])
  #print(e['ball'])

  if (predictedBall != None and
      prevBall != None ):
    delta = distanceBetweenCenters(ball, predictedBall)
    #print('DELTA {:.4}'.format(delta))
    if (delta > ALLOWED_CALCULATIONS_ERROR or
        ballPred.ballHadCollision(prevBall, ball) ):
      end = ix - 1
      tiks = end-start
      #print(start, end, tiks)
      if (tiks >=2):
        periods.append([start, end])
      start = ix

      '''
      if (delta > ALLOWED_CALCULATIONS_ERROR and
          ballPred.ballHadCollision(prevBall, ball) ):
        pass
        #print('COLLISION AND LARGE DELTA {}'.format(e['current_tick']))
      else:
        print('SOME SHIT {}'.format(e['current_tick']))
        print('DELTA {:.4}'.format(delta))
        print(ballPred.ballHadCollision(prevBall, ball))
      #continue
    #print(e['hits'], '{:.4}, {:.4}'.format(rules.BALL_RADIUS - ball.y, ball.velocity_y))
    '''

  predictedBall = ballPred.nextTickBall(ball)
  prevBall = ball

#lens = []
ixs = []
big = -1
for ix in range(len(periods)):
  start = periods[ix][0]
  end = periods[ix][1]
  l = end - start
  if ( l == big):
    ixs.append(ix)
  if ( l > big):
    big = l
    ixs = [ix]

print(big)
print(ixs)

currentPeriodIx = 0
predictedBall = None
for e in gameLog:
  try:
    start = periods[currentPeriodIx][0]
    end = periods[currentPeriodIx][1]
  except IndexError:
    print('IndexError', currentPeriodIx, len(periods), e['current_tick'])
  
  if (e['current_tick'] == start ):
    timeInterv = (end - start) * (1 / rules.TICKS_PER_SECOND)
    ball = ballFromLog(e['ball'])
    predictedBall = ballPred.predictedBallWoColls(ball, timeInterv)

  if (e['current_tick'] == end ):
    ball = ballFromLog(e['ball'])
    delta = distanceBetweenCenters(ball, predictedBall)
    if (delta > ALLOWED_CALCULATIONS_ERROR):
      print(delta)
      print(start, end, end-start)
    if ( currentPeriodIx in ixs ):
      print('longest', delta)

    currentPeriodIx += 1




"""

    
