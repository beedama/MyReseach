import time

STOP              = 0
UP_FAST_SPEED     = 1
DOWN_FAST_SPEED   = 2
UP_MIDDLE_SPEED   = 3
DOWN_MIDDLE_SPEED = 4
UP_SLOW_SPEED     = 5
DOWN_SLOW_SPEED   = 6


state_names = ["STOP", "UP_FAST_SPEED", "DOWN_FAST_SPEED", "UP_MIDDLE_SPEED", "UP_MIDDLE_SPEED", "UP_SLOW_SPEED", "DOWN_SLOW_SPEED"]


class MotorControl():
    def __init__(self, ser):
        self.ser     = ser
        self.state   = STOP

        self.min_fast_vel = 0
        self.max_fast_vel = 90000
        self.fast_accel   = 65535

        self.min_middle_vel = 0
        self.max_middle_vel = 28500
        self.middle_accel   = 65535
        
        self.min_slow_vel = 0
        self.max_slow_vel = 20000
        self.slow_accel   = 65535

        self.slow_speed_threshold    = 20
        self.middle_speed_threshould = 70
        
        self.enc_count_per_px    = 120

        self.now_time     = time.time()
        self.sended_time  = self.now_time
        self.stopped_time = self.now_time

        print("init motor controller")


    def update(self, diff):
        print(state_names[self.state])

        self.now_time = time.time()

        time_duration = 2

        # clean APT controller
        # if self.now_time - self.stopped_time > time_duration: 
#        if self.now_time - self.stopped_time > 8:
 #           self.clean_apt_controller()
  #          self.stopped_time = time.time()
   #         print("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")

        
        # if 1 second have passed since sended serial, resend these serial comm.
        if diff == 0:
            next_state = STOP

            if self.state != next_state or self.now_time-self.sended_time > time_duration:
                self.state = self.stop_speed()
                self.sended_time = time.time()
                print("sended {}".format(state_names[self.state]))
        elif  abs(diff) < self.slow_speed_threshold:
            next_state = UP_SLOW_SPEED if diff > 0 else DOWN_SLOW_SPEED
            
            if self.state != next_state or self.now_time-self.sended_time > time_duration:
                self.state = self.move_slow_speed(diff)
                self.sended_time = time.time()
                print("sended {}".format(state_names[self.state]))
        elif  self.slow_speed_threshold <= abs(diff) and abs(diff) < self.middle_speed_threshould: # middle area
            next_state = UP_MIDDLE_SPEED if diff > 0 else DOWN_MIDDLE_SPEED
            
            if self.state != next_state or self.now_time-self.sended_time > time_duration:
                self.state = self.move_middle_speed(diff)
                self.sended_time = time.time()
                print("sended {}".format(state_names[self.state]))
        else:
            next_state = UP_FAST_SPEED  if diff > 0 else DOWN_FAST_SPEED
            
            if self.state != next_state or self.now_time-self.sended_time > time_duration:
                self.state = self.move_fast_speed(diff)
                self.sended_time = time.time()
                print("sended {}".format(state_names[self.state]))


    def clean_apt_controller(self):
        header = b'\x12\x00\x00\x00\xA2\x01'

    def stop_speed(self):
        header = b'\x65\x04\x01\x02\xA2\x01'
        self.ser.write(header)

        return STOP


    def move_slow_speed(self, diff):
        # self.stop_speed()

        # set velocity
        motor_addr = b'\xA2'
        raspi_addr = b'\x01'
        header     = b'\x13\x04\x0E\x00' + motor_addr + raspi_addr

        ch_indent = b'\x01\x00'

        min_vel = int(self.min_slow_vel).to_bytes(4, byteorder='little', signed=False) # 0 encoder-count/sec
        max_vel = int(self.max_slow_vel).to_bytes(4, byteorder='little', signed=False)
        accel   = int( self.slow_accel ).to_bytes(4, byteorder='little', signed=False)

        self.ser.write(b'\x65\x04\x01\x01\xA2\x01') #stop motor each time
        self.ser.write(header + ch_indent + min_vel + accel + max_vel)

        # \xFF\x00\x00\x00 <= accel <= \xFF\xFF\x00\x00 (caution!: this byte is little endian)
        ch_indent = b'\x01'
        direction = b'\x01' if diff > 0 else b'\x02' # move up if diff is plus, or move down if diff is minus

        header = b'\x57\x04' + ch_indent + direction + b'\x11\x01'

        self.ser.write(header)
        self.ser.write(b'\x65\x04\x01\x01\xA2\x01') #stop motor each time
        # return state
        return UP_SLOW_SPEED if diff > 0 else DOWN_SLOW_SPEED


    def move_middle_speed(self, diff):
        # set velocity
        motor_addr = b'\xA2'
        raspi_addr = b'\x01'
        header     = b'\x13\x04\x0E\x00' + motor_addr + raspi_addr

        ch_indent = b'\x01\x00'

        min_vel = int(self.min_middle_vel).to_bytes(4, byteorder='little', signed=False) # 0 encoder-count/sec
        max_vel = int(self.max_middle_vel).to_bytes(4, byteorder='little', signed=False)
        accel   = int( self.middle_accel ).to_bytes(4, byteorder='little', signed=False)

        self.ser.write(header + ch_indent + min_vel + accel + max_vel)

        # \xFF\x00\x00\x00 <= accel <= \xFF\xFF\x00\x00 (caution!: this byte is little endian)
        ch_indent = b'\x01'
        direction = b'\x01' if diff > 0 else b'\x02' # move up if diff is plus, or move down if diff is minus

        header = b'\x57\x04' + ch_indent + direction + b'\x11\x01'

       # self.ser.write(b'\x65\x04\x01\x01\xA2\x01') #stop motor each time
        self.ser.write(header)

        # return state
        return UP_MIDDLE_SPEED if diff > 0 else DOWN_MIDDLE_SPEED


    def move_fast_speed(self, diff):
        # set velocity
        motor_addr = b'\xA2'
        raspi_addr = b'\x01'
        header     = b'\x13\x04\x0E\x00' + motor_addr + raspi_addr

        ch_indent = b'\x01\x00'

        min_vel = int(self.min_fast_vel).to_bytes(4, byteorder='little', signed=False) # 0 encoder-count/sec
        max_vel = int(self.max_fast_vel).to_bytes(4, byteorder='little', signed=False)
        accel   = int( self.fast_accel ).to_bytes(4, byteorder='little', signed=False)

        self.ser.write(header + ch_indent + min_vel + accel + max_vel)

        ch_indent = b'\x01'
        # move up if diff is plus, or move down if diff is minus
        direction = b'\x01' if diff > 0 else b'\x02' 

        header = b'\x57\x04' + ch_indent + direction + b'\x11\x01'

#        self.ser.write(b'\x65\x04\x01\x01\xA2\x01') #stop motor each time
        self.ser.write(header)

        # return state
        return UP_FAST_SPEED if diff > 0 else DOWN_FAST_SPEED

