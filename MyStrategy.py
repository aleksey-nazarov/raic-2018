# -*- coding: utf-8 -*-

from vectors import Vector2D, Vector3D
from aux_scratch import botToPointAndStop, botToPointAndKick, solveSquareEquation
import copy

class MiscInfo:
  pass

miscInfo = MiscInfo()
miscInfo.initDone = False
miscInfo.ballPrevTick = None
miscInfo.gameRules = None
#miscInfo.robots = {}

#testBotID = None
#last_y_speed = 0.0
#jumped = False

def getAimPoint(rules):
  # точка в центре ворот противника, куда целится робод
  px = 0 # obviously
  py = rules.arena.goal_height / 2
  pz = rules.arena.depth / 2
  return Vector3D(px, py, pz)

def getDefencePoint(rules):
  # точка стояния защитника в воротах
  px = 0 # obviously
  py = robot.radius
  pz = 0.0 - rules.arena.depth / 2
  return Vector3D(px, py, pz)

def getStrikePoint(game): # (game: Game)
  '''
  dstDelta = miscInfo.gameRules.ROBOT_MIN_RADIUS + \
             miscInfo.gameRules.BALL_RADIUS
  '''
  dstDelta = miscInfo.gameRules.ROBOT_MIN_RADIUS + \
             miscInfo.gameRules.BALL_RADIUS - \
             0.20 # в лучших традициях подбирается экспертным путем ((
  
  if ( game.ball.z < miscInfo.aimPoint.z ):
    pt2 = Vector3D(game.ball.x, game.ball.y, game.ball.z)
    pt1 = miscInfo.aimPoint
  else:
    pt2 = miscInfo.aimPoint
    pt1 = Vector3D(game.ball.x, game.ball.y, game.ball.z)

  kx = pt2.x - pt1.x
  ky = pt2.y - pt1.y
  kz = pt2.z - pt1.z

  ts = ( (dstDelta ** 2) /
         ( kx**2 + ky**2 + kz**2 ) ) ** 0.5

  xs = kx * ts + game.ball.x
  ys = ky * ts + game.ball.y
  zs = kz * ts + game.ball.z
  
  #print(pt2.z, pt1.z)

  #print(dstDelta, miscInfo.aimPoint.x, miscInfo.aimPoint.y, miscInfo.aimPoint.z, \
       #kx,ky,kz,ts, sep = '\n' )

  #if ( game.current_tick == 5 ):
    #return None

  return Vector3D(xs, ys, zs)

def getPursuePoint(bot, game):
  # если робот дальше от ворот, чем мяч, то прыгать прямо на мяч - плохая идея
  if ( bot.z > game.ball.z ):
    # вообще это должен проверять клиент, но ладно уж
    return getStrikePoint(game)

  botRVec = Vector2D(bot.x, bot.z)
  ballRVec = Vector2D(game.ball.x, game.ball.z)

  vecToBall = ballRVec - botRVec
  intermVec = vecToBall.normalize()
  if ( bot.x <= game.ball.x ):
    ortoVec = Vector2D(intermVec.z, Vector2D(0.0, 0.0) - intermVec.x)
  else:
    ortoVec = Vector2D(Vector2D(0.0, 0.0) - intermVec.z, intermVec.x)
  pursuePtVec = vecToBall + ortoVec

  return pursuePtVec

def init(me, rules, game):
  global miscInfo
  if (miscInfo.initDone == False):
    miscInfo.ballPrevTick = game.ball
    miscInfo.gameRules = rules
    miscInfo.initDone = True
    miscInfo.aimPoint = getAimPoint(rules)
    miscInfo.currentTick = -1
    # TODO REMOVE
    miscInfo.playBotId = me.id
    miscInfo.reqX = 10.0 # me.z * 1.5
    miscInfo.reqZ = 20.0 # 0.0 - me.x * 2
    miscInfo.prevBotState = me
    # ====

    
    # макс. высота, на которую может запрыгнуть робот
    # (пока без нитры и акробатики)
    flT = rules.ROBOT_MAX_JUMP_SPEED / rules.GRAVITY
    miscInfo.reachableZ = rules.ROBOT_MIN_RADIUS + \
                          rules.ROBOT_MAX_JUMP_SPEED * flT - \
                          rules.GRAVITY * flT * flT / 2

    # макс. время, за которое робот достигает верхней точки траектории
    # ( при, как нетрудно догадаться,  прыжке с максимальной начальной скоростью)
    miscInfo.maxFlightHalfTime = flT

def setRoles(game):
  if ( game.current_tick != miscInfo.currentTick ): # новый тик
    #miscInfo.ballPrevTick = game.ball
    miscInfo.currentTick = game.current_tick
    miscInfo.roles = {}
    myBots = []
    for b in game.robots: # init!
      if ( b.is_teammate == True ):
        #miscInfo.roles[b.id] = None
        myBots.append(b)

    for bot in myBots:
      # TODO очень упрощенно пока
      bot.dstToBall = ( Vector3D(bot.x, bot.y, bot.z) - \
                        Vector3D(game.ball.x, game.ball.y, game.ball.z) ).len() - \
                        bot.radius - game.ball.radius
    minDist = myBots[0].dstToBall
    closest = 0
    for i in range(len(myBots)):
      if ( myBots[i].dstToBall < minDist ):
        closest = i
        minDist = myBots[i].dstToBall
    for i in range(len(myBots)):
      if ( closest == i ):
        miscInfo.roles[myBots[i].id] = 'attacker' # нападающий пока что АДЫН
      else:
        miscInfo.roles[myBots[i].id] = 'defender'

class MyStrategy:
  def custom_rendering(self):
    return ""
      
  def act(self, me, rules, game, action):
    init(me, rules, game) # runs once
    setRoles(game) #runs once per tick
    #print(miscInfo.roles)

    #rVec = Vector2D(miscInfo.reqX, miscInfo.reqZ)
    rVec = Vector2D( game.ball.x, game.ball.z )
    
    if ( me.id == miscInfo.playBotId ):
      a = ( Vector2D(me.velocity_x, me.velocity_z) -
            Vector2D(miscInfo.prevBotState.velocity_x,
                     miscInfo.prevBotState.velocity_z) ).len() / \
          ( 1/rules.TICKS_PER_SECOND )
      miscInfo.prevBotState = me
      dst = (rVec - Vector2D(me.x, me.z)).len()
      '''
      outS = 'cT: {cT}, dst: {dstn:.4}, acc:{acc:.4}'.format(
                            cT = game.current_tick,
                            dstn = dst,
                            acc = a )
      print(outS)
      
      outS_2 = 'trgVx: {trgVx:.4}, trgVz: {trgVz:.4}'.format(
                            trgVx = action.target_velocity_x,
                            trgVz = action.target_velocity_z )
      print(outS_2)
      '''

    if ( miscInfo.roles[me.id] == 'attacker' ):
      if ( me.z > game.ball.z ):
        pursuePt = getPursuePoint(me, game)
        pursuePt2D = Vector2D(pursuePt.x, pursuePt.z)
        botToPointAndKick(pursuePt2D, me, action, rules)
      else:
        strikePoint = getStrikePoint(game)
        strikePt2D = Vector2D(strikePoint.x, strikePoint.z)
        botToPointAndKick(strikePt2D, me, action, rules)

        # botToPointAndKick не обрабатывает прыжки!

        miscInfo.maxFlightHalfTime
        tic = 1 / rules.TICKS_PER_SECOND
        fhTime = int(miscInfo.maxFlightHalfTime / tic) + 1
        # моделируем на fhTime тиков положение мяча и робота
        ballStates = []
        rangesJumping = []
        rangesOnGround = []
        meStatesJumping = []
        for i in range(fhTime):
          if (i == 0):
            ballState = copy.copy(game.ball)
            ballStates.append(ballState)
            rangesJumping.append(500) # TODO исправить
            rangesOnGround.append(500) # TODO исправить
            meStatesJumping.append(Vector3D(me.x, me.y, me.z))# TODO исправить
            continue
          ballPrevState = ballStates[i-1]
          ballState = copy.copy(ballPrevState)
          ballState.x = ballPrevState.x + ballPrevState.velocity_x * tic * i
          ballState.z = ballPrevState.z + ballPrevState.velocity_z * tic * i
          # с Y не так все просто - есть гравитация и пол
          ballState.y = game.ball.y + game.ball.velocity_y * tic * i - \
                        rules.GRAVITY * (tic * i) ** 2 / 2
          ballState.velocity_y = ballPrevState.velocity_y - rules.GRAVITY * tic
          ballAuxState = copy.copy(ballPrevState)
          if ( ballState.y < rules.BALL_RADIUS ):
            # вот тут-то и подкрался белый пушной зверек
            # пересчитываем положение мяча с учетом отскакивания от пола
            # TODO ахтунг! процедура одношаговая, что приводит к неточности в вычислениях! переделать на итерационную
            h = ballAuxState.y - rules.BALL_RADIUS
            roots = solveSquareEquation(rules.GRAVITY / 2,
                                        -ballAuxState.velocity_y,
                                        h-ballAuxState.y)
            if ( (roots == None) or
                 (roots[0] < 0 and roots[1] < 0) or
                 (roots[0] > tic and roots[1] > tic) ):
              print('Ball impact time calculation error') #TODO REMOVE
              break
            rootsRef = [rt for rt in roots if rt >= 0]
            t = min(rootsRef)
            impX = ballAuxState.x + ballAuxState.velocity_x * t
            impY = 0 # obviously
            impZ = ballAuxState.z + ballAuxState.velocity_z * t
            impVelocityY = ballAuxState.velocity_y - rules.GRAVITY * t
            
            t2 = tic - t
            ballState.velocity_x = ballAuxState.velocity_x * rules.BALL_ARENA_E
            ballState.velocity_y = (- impVelocityY * rules.BALL_ARENA_E) - \
                                   rules.GRAVITY * t2
            ballState.velocity_z = ballAuxState.velocity_z * rules.BALL_ARENA_E
            ballState.x = ballAuxState.x + ballState.velocity_x * t2
            ballState.y = rules.BALL_RADIUS - impVelocityY * rules.BALL_ARENA_E * t2 - \
                          rules.GRAVITY * t ** 2 / 2
            if (ballState.y < rules.BALL_RADIUS):
              ballState.y = rules.BALL_RADIUS + 0.01 # TODO этот костыль надо убрать при переделке на итеративный подход
            ballState.z = ballAuxState.z + ballState.velocity_z * t2
          ballStates.append(ballState)

          jumpingMeRVec = Vector3D(me.x + me.velocity_x * tic * i,
                                   me.radius + rules.ROBOT_MAX_JUMP_SPEED * tic * i - \
                                   rules.GRAVITY * (tic * i) ** 2 / 2,
                                   me.z + me.velocity_z * tic * i)
          onGroundMeRVec = Vector3D(me.x + me.velocity_x * tic * i,
                                    me.radius,
                                    me.z + me.velocity_z * tic * i)
          meStatesJumping.append(jumpingMeRVec)

          class GameStub:
            pass
          gameStub = GameStub()
          gameStub.ball = ballState

          tgtPt = getStrikePoint(gameStub)

          rangeJumping = (jumpingMeRVec - tgtPt).len()
          rangeOnGround = (onGroundMeRVec - tgtPt).len()

          rangesJumping.append(rangeJumping)
          rangesOnGround.append(rangeOnGround)

        minRJ = min(rangesJumping)
        minRoG = min(rangesOnGround)

        if (minRJ < minRoG):
          ix = rangesJumping.index(minRJ)
          ballSt = ballStates[ix]
          ballRVec = Vector3D(ballSt.x, ballSt.y, ballSt.z)
          dst = (ballRVec - meStatesJumping[ix]).len()
          if (dst < (rules.ROBOT_MIN_RADIUS + rules.BALL_RADIUS)):
            action.jump_speed = rules.ROBOT_MAX_JUMP_SPEED

    else: # if ( miscInfo.roles[me.id] == 'defender' ):
      # берем из quick_start
      EPS = 1e-5
      # Будем стоять посередине наших ворот
      target_pos = Vector2D(
          0.0, -(rules.arena.depth / 2.0) + rules.arena.bottom_radius)
      # Причем, если мяч движется в сторону наших ворот
      if game.ball.velocity_z < -EPS:
          # Найдем время и место, в котором мяч пересечет линию ворот
          t = (target_pos.z - game.ball.z) / game.ball.velocity_z
          x = game.ball.x + game.ball.velocity_x * t
          # Если это место - внутри ворот
          if abs(x) < rules.arena.goal_width / 2.0:
              # То пойдем защищать его
              target_pos.x = x
      # Установка нужных полей для желаемого действия
      target_velocity = Vector2D(
          target_pos.x - me.x, target_pos.z - me.z) * rules.ROBOT_MAX_GROUND_SPEED

      action.target_velocity_x = target_velocity.x
      action.target_velocity_z = target_velocity.z
      dist_to_ball = ((me.x - game.ball.x) ** 2
                        + (me.y - game.ball.y) ** 2
                        + (me.z - game.ball.z) ** 2
                        ) ** 0.5

        # Если при прыжке произойдет столкновение с мячом, и мы находимся
        # с той же стороны от мяча, что и наши ворота, прыгнем, тем самым
        # ударив по мячу сильнее в сторону противника
      jump = (dist_to_ball < rules.BALL_RADIUS +
              rules.ROBOT_MAX_RADIUS and me.z < game.ball.z)
      action.jump_speed = rules.ROBOT_MAX_JUMP_SPEED if jump else 0.0
            




      
      

    
