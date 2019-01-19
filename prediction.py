# -*- coding: utf-8 -*-

from model.ball import Ball
from vectors import Vector2D, Vector3D
import copy
import numpy as np

ALLOWED_CALCULATIONS_ERROR = 1e-10
AXIS_VELOCITY_THRESHOLD = 0.01

def solveSquareEquation(a, b, c):
  discr = b ** 2 - 4 * a * c
  if (discr < 0):
    return []
  elif ( discr == 0 ):
    root = - (b / (2 * a))
    return [root, root]
  else:
    root1 = (-b + (discr ** 0.5)) / (2 * a)
    root2 = (-b - (discr ** 0.5)) / (2 * a)
    return [root1, root2]

def solvePower4Equation(cx4, cx3, cx2, cx, c0):
  roots = np.roots([cx4, cx3, cx2, cx, c0])
  # возвращаются только реальные корни
  filteredRoots = []
  for r in roots:
    if ( np.isreal(r) ):
      filteredRoots.append(r.real)
  return filteredRoots

def distanceBetweenCenters(entity1, entity2):
  # расстояние между роботом и мячом (или двумя роботами)
  rVec1 = Vector3D(entity1.x,
                   entity1.y,
                   entity1.z)
  rVec2 = Vector3D(entity2.x,
                   entity2.y,
                   entity2.z)
  return (rVec2 - rVec1).len()
  


class BallPredictor:
  def __init__(self, rules):
    self.rules = rules
    # вспомогательные константы, позволяющие сократить вычисления
    self.tik = 1 / rules.TICKS_PER_SECOND
    # изменение вертикальной скорости за 1 тик при отсутствии столкновений
    self.baseVelYDelta = self.rules.GRAVITY * self.tik

  def nextTickBallPos(self, ball):
    newBall = copy.copy(ball)
    newBall.x = ball.x + ball.velocity_x * self.tik
    newBall.y = ball.y + ball.velocity_y * self.tik - \
                self.rules.GRAVITY * self.tik ** 2 / 2
    newBall.z = ball.z + ball.velocity_z * self.tik
    return newBall

  def nextTickBall(self, ball):
    # предсказывает положение И скорость на следующий тик БЕЗ УЧЕТА cтолкновений
    newBall = self.nextTickBallPos(ball)
    newBall.velocity_x = ball.velocity_x
    newBall.velocity_y = ball.velocity_y - self.baseVelYDelta
    newBall.velocity_z = ball.velocity_z
    return newBall

  def predictedBallPosWoColls(self, ball, timeInterv):
    newBall = copy.copy(ball)
    #print('d1', newBall.y, newBall.velocity_y)
    newBall.x = ball.x + ball.velocity_x * timeInterv
    
    if (newBall.y - self.rules.BALL_RADIUS <= AXIS_VELOCITY_THRESHOLD and
        newBall.velocity_y <= AXIS_VELOCITY_THRESHOLD):
      newBall.y = self.rules.BALL_RADIUS + AXIS_VELOCITY_THRESHOLD
      
      #print('gotcha')
    else:
      newBall.y = ball.y + ball.velocity_y * timeInterv - \
                  self.rules.GRAVITY * timeInterv ** 2 / 2
      
    newBall.z = ball.z + ball.velocity_z * timeInterv
    return newBall

  def predictedBallWoColls(self, ball, timeInterv):
    # предсказывает положение И скорость на заданный интервал БЕЗ УЧЕТА cтолкновений
    newBall = self.predictedBallPosWoColls(ball, timeInterv)
    #print('d2', newBall.y, newBall.velocity_y)
    newBall.velocity_x = ball.velocity_x

    if (ball.y - self.rules.BALL_RADIUS <= AXIS_VELOCITY_THRESHOLD and
        ball.velocity_y <= AXIS_VELOCITY_THRESHOLD):
      newBall.velocity_y = 0
    else:
      newBall.velocity_y = ball.velocity_y - self.rules.GRAVITY * timeInterv
    
    newBall.velocity_z = ball.velocity_z
    return newBall

  def predictedBall(self, ball, timeInterv):
    #print('call')
    totalTime = 0.0
    newBall = copy.copy(ball)
    while ( totalTime < timeInterv ):
      #print('iter')
      ballPrev = copy.copy(newBall)
      timePrev = totalTime
      print('b', newBall.y)
      (timeToColl, newBall) = self.nextCollision(newBall)
      print('timez {:.4} {:.4}  {:.4}'.format(timeInterv, totalTime, timeToColl))
      totalTime += timeToColl
      #print('c', newBall.y)
      #print('c2', newBall.velocity_y)
      if ( totalTime > timeInterv ):
        newBall = self.predictedBallWoColls(ballPrev, timeInterv - timePrev)
    return newBall

  def nextCollision(self, ball):
    # роботов не учитываем, только стены
    # TODO пока не учитываются ворота и все изгибы
    rules = self.rules
    #print('ball.y', ball.y)

    # пользуясь симметрией арены по X и Z, будем считать в рамках одной
    # четверти арены, для чего приведем координаты и скорости

    # TODO не требует ли это вдумчивого рефакторинга?
    #ball = copy.copy(inBall) # TODO REMOVE?
    #print('befor', ball.x,ball.velocity_x)
    

    signX = 1.0
    if (ball.x < 0):
      signX = -1.0
    signZ = 1.0
    if (ball.z < 0):
      signZ = -1.0
      
    ball.x = abs(ball.x)
    ball.z = abs(ball.z)
    ball.velocity_x = ball.velocity_x * signX
    ball.velocity_z = ball.velocity_z * signZ

    #print('aftar',ball.x,ball.velocity_x, signX)
    
    

    '''
    итого имеем возможные времена пересечения интересующих нас линий:
    txWall
    ty (обрабатывает и пол, и потолок в плоской их части)

    по осям - отражения НЕ происходит!!!
    txAxis
    tzAxis
    '''
    # по X
    xMax = rules.arena.width - rules.BALL_RADIUS
    try:
      txWall = xMax / ball.velocity_x
      #if (tx < 0):
        #print('nextCollision() tx calculation error')
    except ZeroDivisionError:
      txWall = -1
      
    xMin = 0.0
    try:
      txAxis= xMin / ball.velocity_x
    except ZeroDivisionError:
      txAxis = -1

    # по Z
    zMax = rules.arena.depth - rules.BALL_RADIUS
    zMin = - rules.arena.depth + rules.BALL_RADIUS
    try:
      tz1 = zMin / ball.velocity_z
      tz2 = zMax / ball.velocity_z
      tz = max([tz1, tz2])
      if (tz < 0):
        print('nextCollision() tz calculation error')
    except ZeroDivisionError:
      tz = -1

    # по Y
    if ( ball.y - self.rules.BALL_RADIUS <= AXIS_VELOCITY_THRESHOLD and
         abs(ball.velocity_y) <= AXIS_VELOCITY_THRESHOLD ):
      # мяч лежит на полу (или катится по полу)
      ty = -1
      #print('vel_y',ball.velocity_y)
    else:
      # есть смысл что-то считать
      yMax = rules.arena.height - rules.BALL_RADIUS
      yMin = rules.BALL_RADIUS
      tys1 = solveSquareEquation(rules.GRAVITY / 2,
                                - ball.velocity_y,
                                yMin - ball.y)
      tys2 = solveSquareEquation(rules.GRAVITY / 2,
                                - ball.velocity_y,
                                yMax - ball.y)
      
      tys1.extend(tys2)
      try:
        ty = min([e for e in tys1 if e > ALLOWED_CALCULATIONS_ERROR])
      except ValueError:
        print('nextCollision() ty calculation error')
        ty = -1

    # в плоскости XY при столкновении с нижним радиусом
    brcX = (rules.arena.width / 2 - rules.arena.bottom_radius)
    # bottom radius center x
    brcY = rules.arena.bottom_radius
    radius = rules.arena.bottom_radius - rules.BALL_RADIUS
    ct4 = rules.GRAVITY ** 2 / 4
    ct3 = - rules.GRAVITY * ball.velocity_y
    ct2 = rules.GRAVITY * brcY - \
          rules.GRAVITY * ball.y + \
          ball.velocity_x ** 2 + \
          ball.velocity_y ** 2
    ct = - 2 * ball.velocity_x * brcX + \
         2 * ball.velocity_x * ball.x - \
         2 * ball.velocity_y * brcY + \
         2 * ball.velocity_y * ball.y
    c = - radius ** 2 + \
        brcX ** 2 - \
        2 * brcX * ball.x + \
        ball.x ** 2 + \
        brcY ** 2 - \
        2 * brcY * ball.y + \
        ball.y ** 2
    txys = solvePower4Equation(ct4, ct3, ct2, ct, c)
    txys = [ttc for ttc in txys if ttc > ALLOWED_CALCULATIONS_ERROR]
    xyBallStates = [self.predictedBallWoColls(ball, ttc) for ttc in txys]
    print(txys)
    filteredTxys = []
    for i in range(len(txys)):
      
      if ( xyBallStates[i].x > brcX and
           xyBallStates[i].y < brcY ):
        filteredTxys.append(txys[i])
        
        b = xyBallStates[i]
        print('state', b.x, b.y, b.z)
    #txys = filteredTxys
    try:
      txy = min(filteredTxys)
    except ValueError:
      txy = -1
    
    try:
      timeToColl = min([e for e in [txWall, ty, tz, txy] if e > 0])
    except ValueError:
      # не можем посчитать ни одну коллизию - сильно похоже на то, что мяч
      # просто лежит на полу
      #print('nextCollision() collision time calculation error: assuming ball is not moving')

      #print([tx, ty, tz])
      #print(ball.x, ball.y, ball.z)
      #print(ball.velocity_x, ball.velocity_y, ball.velocity_z)
      newBall = copy.copy(ball)
      #print(newBall.y)
      #newBall.velocity_y = 0.0
      return (1.0, newBall)
    #print('t', timeToColl)

    #b = ball
    #print('b', b.x, b.velocity_x)
    newBall = self.predictedBallWoColls(ball, timeToColl)
    #b = newBall
    #print('nb', b.x, b.velocity_x)

    # пересчитать скорости в newBall
    # для чего сначала получить нормаль к поверхности
    if ( timeToColl == txWall):
      print('onxW')
      newBall.velocity_x = - 0.7 * newBall.velocity_x
      # по Z и Y - без изменений
    elif ( timeToColl == tz):
      print('onz')
      newBall.velocity_z = - 0.7 * newBall.velocity_z
      # по Y и X - без изменений
    elif ( timeToColl == txy):
      print('onxy')
      bVel = Vector3D(newBall.velocity_x,
                      newBall.velocity_y,
                      newBall.velocity_z)
      normal = Vector3D(ball.x - brcX,
                        ball.y - brcY,
                        0)
      bVelNorm = bVel.projectOn(normal)
      bVelOrto = bVel - bVelNorm
      bVelNorm = Vector3D(0.0, 0.0, 0.0) - bVelNorm * rules.BALL_ARENA_E
      bVel = bVelNorm + bVelOrto

      newBall.velocity_x = bVel.x
      newBall.velocity_y = bVel.y
      newBall.velocity_z = bVel.z
      
    else: #( timeToColl == ty):
      print('ony')
      newBall.velocity_y = - 0.7 * newBall.velocity_y
      # по Z и X - без изменений

    # не забываем обратно поменять знак
    newBall.x = newBall.x * signX
    newBall.z = newBall.z * signZ
    newBall.velocity_x = newBall.velocity_x * signX
    newBall.velocity_z = newBall.velocity_z * signZ

    #print('new vel_y',newBall.velocity_y)

    return (timeToColl, newBall)

  def ballHadCollision(self, ball1, ball2, timeInterval = None):
    # проверяет по отдельности изменение вектора вертикальной скорости
    velYDelta = abs(ball2.velocity_y - ball1.velocity_y)
    # и изменение вектора горизонтальной скорости
    velVec1 = Vector2D(ball1.velocity_x,
                       ball1.velocity_z)
    velVec2 = Vector2D(ball2.velocity_x,
                       ball2.velocity_z)
    horizontalVelDelta = (velVec2 - velVec1).len()


    if (timeInterval == None):
      predictedVelYDelta = self.baseVelYDelta
    else:
      predictedVelYDelta = self.rules.GRAVITY * timeInterval

    if ( horizontalVelDelta != 0.0 or
         abs(velYDelta - predictedVelYDelta) > ALLOWED_CALCULATIONS_ERROR ):
      #print('HAD COLLISION')
      return True
    else:
      #print('NO COLLISION')
      return False




    
    delta = abs(ball2.velocity_y - ball1.velocity_y)
    return delta
