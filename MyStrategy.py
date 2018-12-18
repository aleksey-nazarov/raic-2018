# import robot

testBotID = None
last_y_speed = 0.0
jumped = False
'''
telemetry = open('log.csv', 'wt')
print('current_tick;x;y;z;velocity_x;velocity_y;velocity_z;\
radius;touch;touch_normal_x;touch_normal_y;touch_normal_z\n',
      file=telemetry, flush=True)
'''

class MyStrategy:
    def act(self, me, rules, game, action):
        global testBotID
        global last_y_speed
        global jumped
        # global telemetry
        if (testBotID == None):
            testBotID = me.id

        '''
        if (me.id == testBotID):
            if (me.velocity_z < 30.0 and
                jumped == False):
                action.target_velocity_z = 250
            else:
                jumped = True
        '''
        if (game.current_tick < 25 ):
            action.target_velocity_z = 250

        '''
        if ( jumped == False ):
            if (me.id == testBotID):
                action.target_velocity_x = 250

                if ( last_y_speed >=0 and
                     me.velocity_y <0 ):
                    action.jump_speed = 30
                    jumped = True

            
            last_y_speed = me.velocity_y
            #action.jump_speed = 30 #game.current_tick * 0.075
        '''

        '''
        print(game.current_tick, me.x, me.y, me.z, me.velocity_x, me.velocity_y,
              me.velocity_z, me.radius, me.touch, me.touch_normal_x,
              me.touch_normal_y, me.touch_normal_z, sep =';',
              file = telemetry, flush = True)
        '''
