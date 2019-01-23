from vectors import Vector2D, Vector3D
import copy

import pdb

ALLOWED_POSITION_DIVERGENCE = 0.01
ALLOWED_VELOCITY_DIVERGENCE = 0.001

class BotController:
  def __init__(self, rules):
    self.rules = rules
    # вспомогательные константы, позволяющие сократить вычисления
    self.tik = 1 / rules.TICKS_PER_SECOND
    # изменение вертикальной скорости за 1 тик при отсутствии столкновений
    #self.baseVelYDelta = self.rules.GRAVITY * self.tik

    # макс. высота, на которую может запрыгнуть робот
    # (пока без нитры и акробатики)
    flT = rules.ROBOT_MAX_JUMP_SPEED / rules.GRAVITY
    self.maxReachableZ = rules.ROBOT_MIN_RADIUS + \
                         rules.ROBOT_MAX_JUMP_SPEED * flT - \
                         rules.GRAVITY * flT * flT / 2

    # макс. время, за которое робот достигает верхней точки траектории
    # ( при, как нетрудно догадаться,  прыжке с максимальной начальной скоростью)
    self.maxFlightHalfTime = flT
    
  def preventWallCollision(self, bot, action):
    maxX = self.rules.arena.width / 2 - self.rules.arena.bottom_radius \
           - self.rules.BALL_RADIUS
    maxGoalX = self.rules.arena.goal_width / 2 - self.rules.BALL_RADIUS
    if (abs(bot.x) < maxGoalX):
      maxZ = self.rules.arena.depth / 2 + self.rules.arena.goal_depth - \
             self.rules.arena.bottom_radius - \
             self.rules.BALL_RADIUS
    else:
      maxZ = self.rules.arena.depth / 2 - self.rules.arena.bottom_radius - \
             self.rules.BALL_RADIUS
    
    sameSignX = bot.x * bot. velocity_x # совпадение знаков координаты и скорости
    if ( sameSignX ):
      dstX = maxX - abs(bot.x)
      if (dstX <= 0 ):
        maxVelX = 0
      else:
        maxVelX = (2 * dstX * self.rules.ROBOT_ACCELERATION) ** 0.5
      if ( abs(bot.velocity_x) > maxVelX ):
        signVelX = -1.0 if bot.velocity_x > 0 else -1.0
        action.target_velocity_x = signVelX * maxVelX

    sameSignZ = bot.z * bot. velocity_z # совпадение знаков координаты и скорости
    if ( sameSignZ ):
      dstZ = maxZ - abs(bot.z)
      if (dstZ <= 0 ):
        maxVelZ = 0
      else:
        maxVelZ = (2 * dstZ * self.rules.ROBOT_ACCELERATION) ** 0.5
      if ( abs(bot.velocity_z) > maxVelZ ):
        signVelZ = -1.0 if bot.velocity_z > 0 else -1.0
        action.target_velocity_z = signVelZ * maxVelZ

  def botToPoint(self, targetPoint, bot, action):
    # TODO добавить параметров
    botVec = Vector2D(bot.x, bot.z)
    vecToTarget = targetPoint - botVec

    velocityVec = vecToTarget.normalize() * self.rules.ROBOT_MAX_GROUND_SPEED
    action.target_velocity_x = velocityVec.x
    action.target_velocity_z = velocityVec.z

  def botToPointAndStop(self, targetPoint, bot, action):
    botVec = Vector2D(bot.x, bot.z)
    vecToTarget = targetPoint - botVec
    allowedComponentDivergence = ALLOWED_POSITION_DIVERGENCE / ( 2 ** 0.5 )
    allowedVelocityComponentDivergence = ALLOWED_VELOCITY_DIVERGENCE / ( 2 ** 0.5 )
    
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

    velocityX = bot.velocity_x
    velocityZ = bot.velocity_z
    signVelocX = 1.0
    if (velocityX < 0):
      velocityX = abs(velocityX)
      signVelocX = -1.0
    signVelocZ = 1.0
    if (velocityZ < 0):
      velocityZ = abs(velocityZ)
      signVelocZ = -1.0

    brakeTx = velocityX / self.rules.ROBOT_ACCELERATION
    brakeTz = velocityZ / self.rules.ROBOT_ACCELERATION
    #brakeDstZ = 100 * brakeTz**2 / 2 + 30 * tik + 100*tik**2/2
    # 30 (max), или текущая скорость ?
    brakeDstX = self.rules.ROBOT_ACCELERATION * brakeTx**2 / 2 + \
                velocityX * self.tik + \
                self.rules.ROBOT_ACCELERATION * self.tik**2 / 2
    brakeDstZ = self.rules.ROBOT_ACCELERATION * brakeTz**2 / 2 + \
                velocityZ * self.tik + \
                self.rules.ROBOT_ACCELERATION * self.tik**2 / 2
    
    if (dstX < allowedComponentDivergence):
      action.target_velocity_x = 0.0
    elif ( dstX < brakeDstX and
         signVelocX * signX > 0 ): # т.е. знаки совпадают
      accel = ( bot.velocity_x ** 2 ) / (2 * dstX)
      action.target_velocity_x = \
       (bot.velocity_x - accel * self.tik) * signVelocX
    else:
      action.target_velocity_x = \
        (2 * dstX * self.rules.ROBOT_ACCELERATION ) ** 0.5 * signX

    if (dstZ < allowedComponentDivergence):
      action.target_velocity_z = 0.0
    elif ( dstZ < brakeDstZ and
         signVelocZ * signZ > 0 ): # т.е. знаки совпадают
      accel = ( bot.velocity_z ** 2 ) / (2 *dstZ)
      action.target_velocity_z = \
       (bot.velocity_z - accel * self.tik) * signVelocZ
    else:
      action.target_velocity_z = \
        (2 * dstZ * self.rules.ROBOT_ACCELERATION ) ** 0.5 * signZ

    if ( ( Vector2D(bot.x, bot.z) -
           Vector2D(targetPoint.x, targetPoint.z) ).len() < \
           2 * ALLOWED_POSITION_DIVERGENCE and
         # пришлось ввести дополнительный множитель 2, т.к. без него робот
         # иногда не до конца приходит в круг радиуса ALLOWED_POSITION_DIVERGENCE
         Vector2D(bot.velocity_x, bot.velocity_z).len() < \
           ALLOWED_VELOCITY_DIVERGENCE ):
      return True
    else:
      return False

  def approxTimeToPoint(self, targetPoint, game, bot, timeAsTicks = True):
    botVec = Vector2D(bot.x, bot.z)
    vecToTarget = targetPoint - botVec

    # составляющие скорости мяча: проекция скорости на вектор, направленный
    # на целевую точку, и перпендикулярная ей составляющая
    # ( работаем только с направленной на точку )
    botVelocity = Vector2D(bot.velocity_x, bot.velocity_z)
    velTgtScalar = botVelocity.scalarProjectOn(vecToTarget)

    tToTarget = 0.0
    # время разворота
    if ( velTgtScalar < 0 ):
      tToTarget += 2 * abs(velTgtScalar) / self.rules.ROBOT_ACCELERATION
      velTgtScalar = - velTgtScalar
    dst = vecToTarget.len()
    
    # время разгона
    tAccel = (self.rules.ROBOT_MAX_GROUND_SPEED - velTgtScalar) / \
             self.rules.ROBOT_ACCELERATION
    sAccel = velTgtScalar * tAccel + \
             self.rules.ROBOT_ACCELERATION * tAccel  ** 2 / 2
    #pdb.set_trace()
    if ( sAccel < dst ):
    	tToTarget += tAccel + \
                     (dst - sAccel) / self.rules.ROBOT_MAX_GROUND_SPEED
    else:
      # придется потиково, ничего не поделать
      tik = 1 / self.rules.TICKS_PER_SECOND
      auxVelocity = self.rules.ROBOT_MAX_GROUND_SPEED
      while (sAccel < dst):
        auxVelocity -= tik * self.rules.ROBOT_ACCELERATION
        ds = auxVelocity * tik + \
             self.rules.ROBOT_ACCELERATION * tik ** 2 / 2
        sAccel -= ds
        tAccel -= tik
        if (tAccel < 0):
          print('approxTimeToPoint() unexpected behavior')
          return tik if ( timeAsTicks == False ) else 1
      tToTarget = tAccel
    if (timeAsTicks == True):
      return round(tToTarget / self.tik)
    else:
      return tToTarget
      
  def getAimPoint(self, game):
    # точка в воротах противника, траектория полета мяча к которой 
    # из его текущего положения максимально удалена от роботов противника
    
    # TODO пока используется сильно упрощенный вариант

    enemyBots = []

    for bot in game.robots:
      if ( bot.is_teammate == False and
           bot.z > game.ball.z ):
        enemyBots.append(bot)

    ballRVec = Vector3D(game.ball.x, game.ball.y, game.ball.z)

    # базовые значения - работают, если на пути роботов противника нет
    px = 0 # obviously
    py = self.rules.arena.goal_height / 2
    pz = self.rules.arena.depth / 2 # и не меняется
    return Vector3D(px, py, pz)

    topY = self.rules.arena.goal_height - self.rules.BALL_RADIUS
    bottomY = self.rules.BALL_RADIUS
    
    rightX = self.rules.arena.goal_width / 2 - self.rules.BALL_RADIUS
    leftX = - rightX

    vects = [ [Vector3D(px, py, pz) - ballRVec, 0.0],
              [Vector3D(rightX, topY, pz) - ballRVec, 0.0],
              [Vector3D(leftX, topY, pz) - ballRVec, 0.0],
              [Vector3D(rightX, bottomY, pz) - ballRVec, 0.0],
              [Vector3D(leftX, bottomY, pz) - ballRVec, 0.0] ]

    def elemVal(elem):
      return elem[1]

    for bot in enemyBots:
      botVec = Vector3D(bot.x, bot.y, bot.z) - ballRVec
      for v in vects:
        projection = botVec.projectOn(v[0])
        v[1] = (projection-botVec).len()
      vects.sort(key = elemVal)
      try:
        vects.pop()
        vects.pop()
      except IndexError:
        pass

    return vects[0][0]

  def getStrikePoint(self, game): # (game: Game)
    dstDelta = self.rules.ROBOT_MIN_RADIUS + \
               self.rules.BALL_RADIUS - \
               0.15 # в лучших традициях подбирается экспертным путем ((
    aimPoint = self.getAimPoint(game)
    
    if ( game.ball.z < aimPoint.z ):
      pt2 = Vector3D(game.ball.x, game.ball.y, game.ball.z)
      pt1 = aimPoint
    else:
      pt2 = aimPoint
      pt1 = Vector3D(game.ball.x, game.ball.y, game.ball.z)

    kx = pt2.x - pt1.x
    ky = pt2.y - pt1.y
    kz = pt2.z - pt1.z

    ts = ( (dstDelta ** 2) /
           ( kx**2 + ky**2 + kz**2 ) ) ** 0.5

    xs = kx * ts + game.ball.x
    ys = ky * ts + game.ball.y
    zs = kz * ts + game.ball.z

    return Vector3D(xs, ys, zs)

  def calculateJump(self, height, timeAsTicks = True):
    # для переданной высоты прыжка возвращает время, требуемое на достижение
    # этой высоты, и начальную скорость прыжка
    # если слишком высоко - возвращает неправдоподобно большие значения
    if ( height > self.maxReachableZ ):
      t = 4.0
      if (timeAsTicks == True):
        t = round(t / self.tik)
      return (t, 200)
    jHeight = height - self.rules.ROBOT_MIN_RADIUS
    if (jHeight <= self.rules.ROBOT_MIN_RADIUS):
      return (0, 0.0)
    t = (2 * jHeight / self.rules.GRAVITY) ** 0.5
    v0 = t * self.rules.GRAVITY
    if (timeAsTicks == True):
      t = round(t / self.tik)
    return (t, v0)

  def doDecideJump(self, ballTrajectory, game, bot, action):
    if ( ballTrajectory[0].tick != game.current_tick ):
      print('doDecideJump(): trajectory error')
      return

    if ( bot.z < game.ball.z or
         bot.touch == False ):
      return

    # полагая, что робот прыгнет прям сейчас
    botMovePerTick = Vector2D(bot.velocity_x * self.tik,
                              bot.velocity_z * self.tik)
    botStartRVec = Vector2D(bot.x, bot.z)
    radiusSum = self.rules.ROBOT_MIN_RADIUS + self.rules.BALL_RADIUS

    collisionTick = None

    for i in range(1,32):
      botPos2d = botStartRVec + botMovePerTick * i
      ballPos2d = Vector2D(ballTrajectory[i].x,
                         ballTrajectory[i].z)
      dst2d =( ballPos2d - botPos2d).len()
      if (dst2d < radiusSum):
        collisionTick = i
        break

    if ( collisionTick == None ):
      return

    # требуемая высота)
    minDeltaH = (radiusSum ** 2 - dst2d ** 2) ** 0.5
    if ( minDeltaH < 0 ):
      print('doDecideJump(): minDeltaH error')
      return
    standardBotCY = game.ball.y - radiusSum + 0.5
    if ( standardBotCY < self.rules.ROBOT_MIN_RADIUS ):
      # прыжок не требуется
      return
    # needed bot center y
    needBotCY = min(game.ball.y - minDeltaH,
                    standardBotCY )
    if ( needBotCY < self.maxReachableZ ):
      # не допрыгнем
      return
    jumpTicks, v0 = self.calculateJump(needBotCY)
    if (jumpTicks == collisionTick):
      action.jump_speed = v0
    

      

      
      
      
  
  def assignJump(self, bot, ballPred, game, action):
    mfhTime = self.maxFlightHalfTime
    count = int(mfhTime / 0.01) # точность моделирования
    gameStub = copy.deepcopy(game)
    states = []
    for i in range(count):
      t = i * 0.01
      ballState = ballPred.predictedBall(game.ball, t)
      gameStub.ball = ballState
      strikePoint = self.getStrikePoint(gameStub)
      strikePt2D = Vector2D(strikePoint.x, strikePoint.z)
      botPt2D = Vector2D(bot.x + bot.velocity_x * t,
                         bot.z + bot.velocity_z * t)
      dst = (strikePt2D - botPt2D).len()
      states.append([t, strikePoint, dst])

    def elemVal(elem):
      return elem[2]

    minState = min(states, key = elemVal)
    
    if (minState[2] < 0.55): # можно попробовать
      requiredY = minState[1].y
      print(minState, requiredY, end = ' ')
      if (requiredY > self.maxReachableZ):
        return
      requiredVelY = ( requiredY - self.rules.ROBOT_MIN_RADIUS \
                       + self.rules.GRAVITY * minState[0] ** 2 / 2 ) / t
      #print(requiredVelY)
      if ( requiredVelY > self.rules.ROBOT_MAX_JUMP_SPEED ):
        return
      action.jump_speed = requiredVelY
      




      
