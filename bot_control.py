from vectors import Vector2D, Vector3D
import copy



class BotController:
  def __init__(self, rules):
    self.rules = rules
    # вспомогательные константы, позволяющие сократить вычисления
    #self.tik = 1 / rules.TICKS_PER_SECOND
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
    maxZ = self.rules.arena.depth / 2 - self.rules.arena.bottom_radius \
           - self.rules.BALL_RADIUS
    maxGoalX = self.rules.arena.goal_width / 2 - self.rules.BALL_RADIUS

    dstX = min([abs(maxX - bot.x), abs( - maxX - bot.x)])
    maxVelX = (2 * dstX * self.rules.ROBOT_ACCELERATION) ** 0.5

    if ( bot.x > 0 and
         bot.velocity_x > maxVelX ):
      action.target_velocity_x = min([action.target_velocity_x, maxVelX])

    if ( bot.x < 0 and
         bot.velocity_x < - maxVelX ):
      action.target_velocity_x = max([action.target_velocity_x, - maxVelX])

    dstZ = min([abs(maxZ - bot.z), abs( - maxZ - bot.z)])
    maxVelZ = (2 * dstZ * self.rules.ROBOT_ACCELERATION) ** 0.5

    if ( bot.z > 0 and
         abs(bot.x) > maxGoalX and
         bot.velocity_z > maxVelZ ):
      action.target_velocity_z = min([action.target_velocity_z, maxVelZ])

    if ( bot.z < 0 and
         abs(bot.x) > maxGoalX and
         bot.velocity_z < - maxVelZ ):
      action.target_velocity_z = max([action.target_velocity_z, - maxVelZ])

  def botToPoint(self, targetPoint, bot, action):
    # TODO добавить параметров
    botVec = Vector2D(bot.x, bot.z)
    vecToTarget = targetPoint - botVec

    velocityVec = vecToTarget.normalize() * self.rules.ROBOT_MAX_GROUND_SPEED
    action.target_velocity_x = velocityVec.x
    action.target_velocity_z = velocityVec.z
      
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
      




      
