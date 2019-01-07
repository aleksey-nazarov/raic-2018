# -*- coding: utf-8 -*-

from vectors import Vector2D, Vector3D
from aux_scratch import botToPoint

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
  dstDelta = miscInfo.gameRules.ROBOT_MIN_RADIUS + \
             miscInfo.gameRules.BALL_RADIUS
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

  if ( game.current_tick == 5 ):
    return None



  return Vector3D(xs, ys, zs)
    

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
      outS = 'cT: {cT}, dst: {dstn:.4}, acc:{acc:.4}'.format(
                            cT = game.current_tick,
                            dstn = dst,
                            acc = a )
      print(outS)
      '''
      outS_2 = 'trgVx: {trgVx:.4}, trgVz: {trgVz:.4}'.format(
                            trgVx = action.target_velocity_x,
                            trgVz = action.target_velocity_z )
      print(outS_2)
      '''
      if ( botToPoint(rVec, me, action, rules) == True ):
        print('FUCKING HOORAH!!!')


'''
    if ( miscInfo.roles[me.id] == 'attacker' ):
      # TOODOO
      strikePoint = getStrikePoint(game)
      if (me.z < 0):
        action.target_velocity_z = 0.0 - 30 * me.z
      if (me.z > 0):
        action.target_velocity_z = 0.0 - 30 * me.z
        
'''


      
      

    
