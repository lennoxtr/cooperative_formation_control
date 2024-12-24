import time

class PidController():
    def __init__(self, Kp, Ki, Kd):
        #PID params
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        
        #PID values
        self.previous_error = 0
        self.integral = 0
        self.last_time = time.time()


    def compute(self, error, current_time):
        # PID terms
        P = self.Kp * error

        self.integral += error
        I = self.Ki * self.integral

        dedt = (error - self.previous_error) / (current_time - self.last_time)
        D = self.Kd * dedt
        self.last_time = current_time
        self.previous_error = error

        output = P + I + D

        return output