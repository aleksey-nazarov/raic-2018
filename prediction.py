# -*- coding: utf-8 -*-

from model.ball import Ball
from vectors import Vector2D, Vector3D
import copy

ALLOWED_CALCULATIONS_ERROR = 1e-10

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
    newBall.x = ball.x + ball.velocity_x * timeInterv
    newBall.y = ball.y + ball.velocity_y * timeInterv - \
                self.rules.GRAVITY * timeInterv ** 2 / 2
    newBall.z = ball.z + ball.velocity_z * timeInterv
    return newBall

  def predictedBallWoColls(self, ball, timeInterv):
    # предсказывает положение И скорость на заданный интервал БЕЗ УЧЕТА cтолкновений
    newBall = self.predictedBallPosWoColls(ball, timeInterv)
    newBall.velocity_x = ball.velocity_x
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
      #print('b', newBall.y)
      (timeToColl, newBall) = self.nextCollision(newBall)
      print('{:.4}  {:.4}'.format(totalTime, timeToColl))
      totalTime += timeToColl
      #print('c', newBall.y)
      #print('c2', newBall.velocity_y)
      if ( totalTime > timeInterv ):
        #print('d', ballPrev.y)
        newBall = self.predictedBallWoColls(ballPrev, timeInterv - timePrev)
    return newBall

  def nextCollision(self, ball):
    # роботов не учитываем, только стены
    # TODO пока не учитываются ворота и все изгибы
    rules = self.rules

    # TODO тестовая вещь, возможно не стоит так делать
    '''
    tooSmallVel = 0.01
    if ( abs(ball.velocity_x) < tooSmallVel and
         abs(ball.velocity_y) < tooSmallVel and
         abs(ball.velocity_z) < tooSmallVel ):
      newBall = copy.copy(ball)
      newBall.velocity_x = 0.0
      newBall.velocity_y = 0.0
      newBall.velocity_z = 0.0
      return (1.0, newBall)
    '''
    

    # по X
    xMax = rules.arena.width - rules.BALL_RADIUS
    xMin = - rules.arena.width + rules.BALL_RADIUS
    try:
      tx1 = xMin / ball.velocity_x
      tx2 = xMax / ball.velocity_x
      tx = max([tx1, tx2])
      if (tx < 0):
        print('nextCollision() tx calculation error')
    except ZeroDivisionError:
      tx = -1

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
    yMax = rules.arena.height - rules.BALL_RADIUS
    yMin = rules.BALL_RADIUS
    tys1 = solveSquareEquation(rules.GRAVITY / 2,
                              - ball.velocity_y,
                              yMin - ball.y)
    tys2 = solveSquareEquation(rules.GRAVITY / 2,
                              - ball.velocity_y,
                              yMax - ball.y)
    
    tys1.extend(tys2)
    #print('tys', tys1)
    try:
      ty = min([e for e in tys1 if e > ALLOWED_CALCULATIONS_ERROR])
    except ValueError:
      print('nextCollision() ty calculation error')
      ty = -1

    # этот блок ничего не знает про скругления углов
    try:
      timeToColl = min([e for e in [tx, ty, tz] if e > 0])
    except ValueError:
      # распсиховаться и все бросить
      print('nextCollision() collision time calculation error')
      #print([tx, ty, tz])
      #print(ball.x, ball.y, ball.z)
      #print(ball.velocity_x, ball.velocity_y, ball.velocity_z)
      newBall = copy.copy(ball)
      newBall.velocity_y = 0.0
      return (1.0, newBall)
    print('t', timeToColl)
    
    newBall = self.predictedBallWoColls(ball, timeToColl)
    '''
    if ( abs(newBall.velocity_x) < tooSmallVel ):
      newBall.velocity_x = 0.0
    if ( abs(newBall.velocity_y) < tooSmallVel ):
      newBall.velocity_y = 0.0
    if ( abs(newBall.velocity_z) < tooSmallVel ):
      newBall.velocity_z = 0.0
    '''

    # пересчитать скорости в newBall
    # для чего сначала получить нормаль к поверхности
    if ( timeToColl == tx):
      newBall.velocity_x = - 0.7 * newBall.velocity_x
      # по Z и Y - без изменений
    elif ( timeToColl == tz):
      newBall.velocity_z = - 0.7 * newBall.velocity_z
      # по Y и X - без изменений
    else: #( timeToColl == ty):
      newBall.velocity_y = - 0.7 * newBall.velocity_y
      # по Z и X - без изменений

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
