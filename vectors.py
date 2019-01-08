# -*- coding: utf-8 -*-
'''
class Point3D:
  def __init__(self, x=0.0, y=0.0, z=0.0):
    self.x = x
    self.y = y
    self.z = z


  def __init__(self, entity):
    # штоп вручную не переписывать три координаты
    self.x = entity.x
    self.y = entity.y
    self.z = entity.z

    
  def distanceTo(self, other):
    dX = self.x - other.x
    dY = self.y - other.y
    dZ = self.z - other.z
    distance = ( (dX * dX) + \
                 (dY * dY) + \
                 (dZ * dZ) ) ** 0.5
    return distance
'''
class Vector2D:
  # Нам понадобится работа с 2d векторами
  def __init__(self, x=0.0, z=0.0):
      self.x = x
      self.z = z

  # Нахождение длины вектора
  def len(self):
      return ((self.x * self.x) + (self.z * self.z))**0.5

  # Операция - для векторов
  def __sub__(self, other):
      return Vector2D(self.x - other.x, self.z - other.z)

  # Операция + для векторов
  def __add__(self, other):
      return Vector2D(self.x + other.x, self.z + other.z)

  # Операция умножения вектора на число
  def __mul__(self, num: float):
      return Vector2D(self.x * num, self.z * num)

  # Нормализация вектора (приведение длины к 1)
  def normalize(self):
      return Vector2D(self.x/self.len(), self.z/self.len())

class Vector3D:
  def __init__(self, x=0.0, y=0.0, z=0.0):
      self.x = x
      self.y = y
      self.z = z

  '''
  def __init__(self, entity):
    # штоп вручную не переписывать три координаты
    self.x = entity.x
    self.y = entity.y
    self.z = entity.z
  '''

  # Нахождение длины вектора
  def len(self):
      return ( (self.x * self.x) + \
               (self.y * self.y) + \
               (self.z * self.z) ) ** 0.5

  # Операция - для векторов
  def __sub__(self, other):
      return Vector3D(self.x - other.x,\
                      self.y - other.y,\
                      self.z - other.z)

  # Операция + для векторов
  def __add__(self, other):
      return Vector3D(self.x + other.x,\
                      self.y + other.y,\
                      self.z + other.z)

  # Операция умножения вектора на число
  def __mul__(self, num: float):
      return Vector3D(self.x * num,\
                      self.y * num,\
                      self.z * num)

  # Нормализация вектора (приведение длины к 1)
  def normalize(self):
      return Vector3D(self.x/self.len(),\
                      self.y/self.len(),\
                      self.z/self.len())