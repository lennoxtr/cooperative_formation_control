class PidController():
    def __init__(self, Kp, Ki, Kd):
        #PID params
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        
        #PID values
        self.previous_error = None
        self.integral = None
        self.last_time = None


    def compute(self, error, current_time):
        # check for None
        self.previous_error = 0 if self.previous_error is None else self.previous_error
        self.integral = 0 if self.integral is None else self.integral
        self.last_time = 0 if self.integral is None else self.last_time

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