# -*- coding: utf-8 -*-

from vectors import Vector2D, Vector3D
#from aux_scratch import botToPointAndStop
from prediction import BallPredictor, distanceBetweenCenters
from bot_control import BotController
import copy
import pdb

class MiscInfo:
  pass

ballPred = None
botControl = None
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
             miscInfo.gameRules.BALL_RADIUS - \
             0.20 # в лучших традициях подбирается экспертным путем ((
  '''
  dstDelta = miscInfo.touchDst2d - 0.1
  
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
  
  dst = distanceBetweenCenters(game.ball, bot)
  multiplier = dst / (dst - 3.0 + 1e-10)
  if ( bot.x <= game.ball.x ):
    ortoVec = Vector2D(intermVec.z, Vector2D(0.0, 0.0) - intermVec.x)
  else:
    ortoVec = Vector2D(Vector2D(0.0, 0.0) - intermVec.z, intermVec.x)
  pursuePtVec = vecToBall + ortoVec * multiplier

  return pursuePtVec

def init(me, rules, game):
  global ballPred
  global botControl
  global miscInfo
  if (miscInfo.initDone == False):
    miscInfo.ballPrevTick = game.ball
    miscInfo.gameRules = rules
    miscInfo.initDone = True
    miscInfo.rolesGiven = False
    miscInfo.aimPoint = getAimPoint(rules)
    miscInfo.currentTick = -1
    miscInfo.tik = 1 / rules.TICKS_PER_SECOND
    # TODO REMOVE
    miscInfo.playBotId = me.id
    miscInfo.reqX = 10.0 # me.z * 1.5
    miscInfo.reqZ = 20.0 # 0.0 - me.x * 2
    miscInfo.prevBotState = me

    ballPred = BallPredictor(rules)
    botControl = BotController(rules)
    # ====

    # расстояние по горизонтали между центрами соприкасающихся мяча и робота,
    # лежащих на полу
    miscInfo.touchDst2d = ( (miscInfo.gameRules.BALL_RADIUS + \
                             miscInfo.gameRules.ROBOT_MIN_RADIUS) ** 2 - \
                            (miscInfo.gameRules.BALL_RADIUS - \
                             miscInfo.gameRules.ROBOT_MIN_RADIUS) ** 2 ) ** 0.5
    

    
    # макс. высота, на которую может запрыгнуть робот
    # (пока без нитры и акробатики)
    flT = rules.ROBOT_MAX_JUMP_SPEED / rules.GRAVITY
    miscInfo.reachableZ = rules.ROBOT_MIN_RADIUS + \
                          rules.ROBOT_MAX_JUMP_SPEED * flT - \
                          rules.GRAVITY * flT * flT / 2

    # макс. время, за которое робот достигает верхней точки траектории
    # ( при, как нетрудно догадаться,  прыжке с максимальной начальной скоростью)
    miscInfo.maxFlightHalfTime = flT

def initTick(game):
  global ballPred
  ballPred.update(game)

def ticksFromTime(time):
  return round(time / miscInfo.tik)

def setRoles(game):
  if ( miscInfo.rolesGiven == False and
       game.ball.velocity_x == 0 and
       game.ball.velocity_z == 0 and
       game.ball.x == 0 and
       game.ball.z == 0 ): # новый розыгрыш мяча):
    #miscInfo.ballPrevTick = game.ball
    miscInfo.currentTick = game.current_tick
    miscInfo.roles = {}
    myBots = [ [b, distanceBetweenCenters(b, game.ball) ]
               for b in game.robots if ( b.is_teammate == True ) ]
    def dstCompEl(e):
      return e[1]
    attacker = min(myBots, key = dstCompEl)[0]
    for bot in myBots:
      if ( bot[0].id == attacker.id ):
        miscInfo.roles[bot[0].id] = 'attacker'
      else:
        miscInfo.roles[bot[0].id] = 'defender'
    miscInfo.rolesGiven = True

  if ( game.ball.velocity_x != 0 or
       game.ball.velocity_z != 0 or
       game.ball.x != 0 or
       game.ball.z != 0 ): # новый розыгрыш мяча
    miscInfo.rolesGiven = False

def doAttack(bot, game, action):

  botVec = Vector2D(bot.x, bot.z)
  ballVec = Vector2D(game.ball.x, game.ball.z)

  # если мяч позади нас, отрабатываем возвращение
  if (bot.z > game.ball.z):
    botControl.pursueBall(bot, game, action)
    return

  prTrajectory = ballPred.getPredictedTrajectory()

  # частный случай - робот вблизи мяча, который катится (не прыгает)
  flatTrajectory = True
  for b in prTrajectory[:30]:
    if ( b.y > miscInfo.gameRules.BALL_RADIUS + 1 ):
      flatTrajectory = False
      break
  if ( flatTrajectory == True and
       (botVec - ballVec).len() < miscInfo.touchDst2d * 2 ):
    # точка удара по мячу
    gateToBallVec = ballVec - \
                    Vector2D(0.0, miscInfo.gameRules.arena.depth / 2 - \
                             miscInfo.gameRules.BALL_RADIUS)
    ballVelocVec = Vector2D(game.ball.velocity_x, game.ball.velocity_z)
    correctionVec = ( gateToBallVec.normalize() + \
                      ballVelocVec.normalize() ).normalize() * \
                      ( miscInfo.touchDst2d / 2 )
    tgtPt = ballVec + correctionVec
    botControl.botToPoint(tgtPt, bot, action)
    return
  

  # общий случай - мяч где-то летает
  ballInterceptionPos = None

  ptsCount = len(prTrajectory)

  for i in range(1, ptsCount - 1):
    if (prTrajectory[i].y < prTrajectory[i-1].y and
        prTrajectory[i].y < prTrajectory[i+1].y):
      ballInterceptionPos = prTrajectory[i]
      break

  if (ballInterceptionPos == None):
    return

  # точка удара по мячу
  ballVelocVec = Vector2D(game.ball.velocity_x, game.ball.velocity_z)
  ballToGateVec = Vector2D(0.0, miscInfo.gameRules.arena.depth / 2 - \
                           miscInfo.gameRules.BALL_RADIUS) - \
                  ballVec
  if ( ballVelocVec.scalarProjectOn(ballToGateVec) <= 0 ):
    correctionVec = ( (Vector2D(0,0) - ballToGateVec).normalize() + \
                      ballVelocVec.normalize() ).normalize() * \
                      ( miscInfo.touchDst2d / 2 )
  else:
    # чет я не уверен
    correctionVec = ( (Vector2D(0,0) - ballToGateVec).normalize() * 2.0 + \
                      ballVelocVec.normalize() ).normalize() * \
                      ( miscInfo.touchDst2d / 2)

  intcptPt = Vector2D(ballInterceptionPos.x + correctionVec.x,
                      ballInterceptionPos.z + correctionVec.z)

  botControl.botToPointAndStop(intcptPt, bot, action)

def doDefend(me, game, action):
  prTrajectory = ballPred.getPredictedTrajectory()
  # если все спокойно, стоять в центре ворот
  waitPt = Vector2D(0.0,
                   -(miscInfo.gameRules.arena.depth / 2.0) - 1.5 )

  defAreaWidth = miscInfo.gameRules.arena.goal_width
  defAreaDepth = 13.0 # эмпирически, как и всегда ((
  defAreaZeroZ = - (miscInfo.gameRules.arena.depth / 2.0)
  defAreaZ = defAreaZeroZ + defAreaDepth

  interceptionPoint = None
  goodHeight = miscInfo.gameRules.BALL_RADIUS + \
               miscInfo.gameRules.ROBOT_MIN_RADIUS - 0.5
  # max BALL CENTER height при которой до мяча еще дотянется робот
  maxRH = miscInfo.reachableZ + \
          miscInfo.gameRules.ROBOT_MIN_RADIUS + \
          miscInfo.gameRules.BALL_RADIUS

  lineCrossX = None

  # если мяч в ближайшее время попадает на площадку перед воротами
  for b in prTrajectory:
    if ( b.z < (- miscInfo.gameRules.arena.depth / 2) - \
         miscInfo.gameRules.BALL_RADIUS ):
      lineCrossX = b.x
      break
    if ( abs(b.x) < defAreaWidth / 2 and
         b.z < defAreaZ ):
      #ballNearGoal = True
      #print(game.current_tick, ' detected')
      ticksLeft = b.tick - game.current_tick
      bPt = Vector2D(b.x, b.z)
      ticksNeed = botControl.approxTimeToPoint(bPt, game, me)
      if (ticksNeed < 0):
        pdb.set_trace()
      ticksNeedToJump, v0 = \
        botControl.calculateJump(b.y - \
                                 miscInfo.gameRules.BALL_RADIUS - \
                                 miscInfo.gameRules.ROBOT_MIN_RADIUS)
      if (ticksNeedToJump == 0 and
          interceptionPoint == None):
        interceptionPoint = b
        #break
      '''
      if (ticksNeed + ticksNeedToJump < ticksLeft):
        if (interceptionPoint == None):
          interceptionPoint = b
        else:
          if (b.y < interceptionPoint.y):
            interceptionPoint = b
      '''

  # TODO REMOVE
  '''
  readyPt = copy.copy(waitPt)
  if (goalAlert == True):
    readyPt.x = lineCrossX
    print('cross', lineCrossX)
  botControl.botToPointAndStop(readyPt, me, action)

  return
  '''

  #if (lineCrossX != None and interceptionPoint == None):
    #print(game.current_tick, ' FUCK, GOAL IMMINENT')

  # прочие случаи
  if (me.z > game.ball.z or 
      interceptionPoint == None ):
    botControl.botToPointAndStop(waitPt, me, action)
  
  if ( interceptionPoint != None ):
    tgtPt = Vector2D(interceptionPoint.x, interceptionPoint.z - 1.0)
    ticksLeft = interceptionPoint.tick - game.current_tick
    ticksNeedToRoll = botControl.approxTimeToPoint(tgtPt, game, me)
    h = min(interceptionPoint.y, miscInfo.reachableZ)
    ticksNeedToJump, v0 = \
      botControl.calculateJump(h)

    #print(game.current_tick, ticksNeedToJump, ticksNeedToRoll, ticksLeft)
    if ( ticksLeft - ticksNeedToRoll <= 0 ):
      botControl.botToPoint(tgtPt, me, action)
      #botControl.botToPointAndStop(tgtPt, me, action)
    else:
      readyPt = copy.copy(waitPt)
      if (lineCrossX != None):
        readyPt.x = lineCrossX
      botControl.botToPointAndStop(readyPt, me, action)
      #botControl.botToPointAndStop(waitPt, me, action)
    '''
    if ( ticksLeft - ticksNeedToJump <= 0 and
         me.touch == True ):
      action.jump_speed = v0
    '''
    botControl.doDecideJump(prTrajectory, game, me, action)
    
    kickBall = ( Vector3D(game.ball.x, game.ball.y, game.ball.z) - \
                 Vector3D(me.x, me.y, me.z) ).len() <= \
               ( miscInfo.gameRules.ROBOT_MAX_RADIUS + \
                 miscInfo.gameRules.BALL_RADIUS ) and \
               me.z < game.ball.z    
    if ( kickBall ):
      action.jump_speed = miscInfo.gameRules.ROBOT_MAX_JUMP_SPEED
    return
  

  


            



  

class MyStrategy:
  def custom_rendering(self):
    return ""
      
  def act(self, me, rules, game, action):
    init(me, rules, game) # runs once
    initTick(game) #runs once per tick
    setRoles(game) #runs once per tick

    #if (game.current_tick % 100 == 0):
      #print(game.current_tick)

    if ( miscInfo.roles[me.id] == 'attacker' ):
      '''
      tgtPt = Vector2D(-27 if me.x < 0 else 27,
                       25)
      botControl.botToPointAndStop(tgtPt, me, action)
      '''
      doAttack(me, game, action)
      botControl.preventWallCollision(me, action)
      
    else: # ( miscInfo.roles[me.id] == 'defender' )
      doDefend(me, game, action)


    return


    



    
    '''
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
        #botToPointAndKick(pursuePt2D, me, action, rules)
        botControl.botToPoint(pursuePt2D, me, action)
        botControl.preventWallCollision(me, action)
      else:
        strikePoint = botControl.getStrikePoint(game)
        strikePt2D = Vector2D(strikePoint.x, strikePoint.z)
        botControl.botToPoint(strikePt2D, me, action)

        botControl.preventWallCollision(me, action)

        # botToPointAndKick не обрабатывает прыжки!
        botControl.assignJump(me, ballPred, game, action)

        '''
        #miscInfo.maxFlightHalfTime
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
        '''

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
              target_pos.z = -(rules.arena.depth / 2.0) - 2

      if (distanceBetweenCenters(game.ball, me) < rules.arena.goal_width / 2.0):
        target_pos.x = game.ball.x
        target_pos.z = game.ball.z
                
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
      dst = ( Vector2D(ball.x, ball.y) - Vector2D(me.x, me.y) ).len()#
      t = dst / ( Vector2D(ball.x, ball.y) - Vector2D(me.x, me.y) )#
      jump =( (dist_to_ball < rules.BALL_RADIUS +
              rules.ROBOT_MAX_RADIUS and me.z < game.ball.z) or
              () )
      action.jump_speed = rules.ROBOT_MAX_JUMP_SPEED if jump else 0.0
            




      
      

    
