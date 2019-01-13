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

gameLog = [json.loads(s) for s in open('empty.log', 'rt')]

rulesDct = gameLog[0]
gameLog = gameLog[1:]
rules = Rules(rulesDct)

ballPred = BallPredictor(rules)

for e in gameLog[950:]:
  s = '{} {:.4} {:.4}'.format(e['current_tick'], e['ball']['position']['y'],
                e['ball']['velocity']['y'])
  print(s)
