from vectors import Vector2D

def botToPointAndStop(targetPoint, bot, action, rules):
  botVec = Vector2D(bot.x, bot.z)
  vecToTarget = targetPoint - botVec
  dst = vecToTarget.len()
  dstX = vecToTarget.x
  signX = 1.0
  if ( dstX < 0 ):
    dstX = abs(dstX)
    signX = -1.0
  dstZ = vecToTarget.z
  signZ = 1.0
  if ( dstZ < 0 ):
    dstZ = abs(dstZ)
    signZ = -1.0

  '''
  dt = 1/rules.TICKS_PER_SECOND
  t = ( (2 * dst - dt) / 100 ) ** 0.5 
  lenSpeed = ( dst + 50 * (t + dt) ** 2 ) / (t + dt)
  if ( lenSpeed < 0.0 ):
    lenSpeed = 0.0
  '''
  '''
  # костыли какие-то получились (но они сука работают, а другие варианты нет (( )
  if ( dst < 0.01 ):
    lenSpeed = 0.0
  elif ( dst < 0.05 ):
    lenSpeed = dst
  else:
    lenSpeed = (2 * dst * 100 ) ** 0.5
  '''
  
  
  '''
  oneTick = 1/rules.TICKS_PER_SECOND
  if ( lenSpeed > oneTick ):
    lenSpeed -= oneTick
  '''
  #reqSpeed = vecToTarget.normalize() * lenSpeed # TODO
  #action.target_velocity_x = reqSpeed.x
  #action.target_velocity_z = reqSpeed.z
  if ( dstX < 0.01 ):
    action.target_velocity_x = 0.0
  elif ( dstX < 0.05 ):
    action.target_velocity_x = dstX * signX
  else:
    action.target_velocity_x = (2 * dstX * 100 ) ** 0.5 * signX
    
  if ( dstZ < 0.01 ):
    action.target_velocity_z = 0.0
  elif ( dstZ < 0.05 ):
    action.target_velocity_z = dstZ * signZ
  else:
    action.target_velocity_z = (2 * dstZ * 100 ) ** 0.5 * signZ

  '''
  outS_2 = 'dstX: {dstnX:.4}, trgVx: {trgVx:.4}, trgVz: {trgVz:.4}'.format(
                            trgVx = action.target_velocity_x,
                            dstnX = dstX,
                            trgVz = action.target_velocity_z )
  print(outS_2)
  '''
  # не сработает
  if ( bot.x == targetPoint.x and
       bot.z == targetPoint.z and
       bot.velocity_x == 0.0 and
       bot.velocity_z == 0.0 ):
    return True
  else:
    return False

def botToPointAndKick(targetPoint, bot, action, rules):
  # TODO добавить параметров
  botVec = Vector2D(bot.x, bot.z)
  vecToTarget = targetPoint - botVec

  velocityVec = vecToTarget.normalize() * rules.ROBOT_MAX_GROUND_SPEED
  action.target_velocity_x = velocityVec.x
  action.target_velocity_z = velocityVec.z

# ==^== botToPointAndKick() ==^==

def solveSquareEquation(a, b, c):
  discr = b ** 2 - 4 * a * c
  if (discr < 0):
    return None
  elif ( discr == 0 ):
    root = - (b / (2 * a))
    return (root, root)
  else:
    root1 = (-b + (discr ** 0.5)) / (2 * a)
    root2 = (-b - (discr ** 0.5)) / (2 * a)
    return (root1, root2)

TICKS_PER_SECOND = 60
ROBOT_MAX_GROUND_SPEED = 30
ROBOT_ACCELERATION = 100

# ball = strike point, ye
ballX = 0
ballZ = 0

targetVx = -1.5
targetVz = ( ROBOT_MAX_GROUND_SPEED ** 2 - targetVx ** 2 ) ** 0.5

rbtX = -5.2
rbtZ = -15

# count of Z divisions
dS = ROBOT_MAX_GROUND_SPEED * ( 1 / TICKS_PER_SECOND )
countZ = ( ballX - rbtX ) / dS
if ( countZ < 2 ):
    dist = ( Vector2D(ballX, ballZ) - Vector2D(rbtX, rbtZ) ).len()
    print("Мы слишком близко!!! расстояние ", dist)
    exit()
if (countZ > 10):
    countZ = 10

dZ = ( ballX - rbtX ) / countZ

# first step
# определим пределы положения бота на прямой (m-1)
# чтобы достигнуть целевой скорости, бот мог максимально ускоряться
# или максимально замедляться
# (или что-то между)
targetV = ( targetVz ** 2 + targetVx ** 2 ) ** 0.5
#vPrevMax 
# при этом 
