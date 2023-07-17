class Pidcal:
    error_sum = 0
    error_old = 0

    p = [0.90009, 0.0005509,0.95968]
    dp = [p[0]/10, p[1]/10, p[2]/10] # to twiddle kp, ki, kd

    def __init__(self):
        print ("init PidCal")
        self.x = 0
        self.pre_setpoint = 0

    def cal_error(self, setpoint): #Width/2
        return self.x - setpoint

    # twiddle is for optimize the kp,ki,kd
    def twiddle(self, setpoint): #245
        best_err = self.cal_error(setpoint)
        #threshold = 0.1
        #threshold = 1e-09
        threshold = 0.0000000000000000000000000000001

        # searching by move 1.1x to the target and if go more through the target comeback to -2x
        while sum(self.dp) > threshold:

            for i in range(len(self.p)):
                self.p[i] += self.dp[i]
                err = self.cal_error(setpoint)

                if err < best_err:  # There was some improvement
                    best_err = err
                    self.dp[i] *= 1.1
                else:  # There was no improvement
                    self.p[i] -= 2*self.dp[i]  # Go into the other direction
                    err = self.cal_error(setpoint)

                    if err < best_err:  # There was an improvement
                        best_err = err
                        self.dp[i] *= 1.05
                    else:  # There was no improvement
                        self.p[i] += self.dp[i]
                        # As there was no improvement, the step size in either
                        # direction, the step size might simply be too big.
                        self.dp[i] *= 0.95
        #print(self.p)


    def get_pid(self, car_loc, setpoint): #245 -> 255
        global errorList

        # for case if flag : None
        self.pre_setpoint = setpoint

        x_current = car_loc
        self.x = int(x_current)
        self.twiddle(setpoint)

        err = abs(x_current - setpoint)
        error = x_current - setpoint

        #print("x_current: ", x_current)

        pControl = round(self.p[0] * error, 9)
        self.error_sum += error
        iControl = round(self.p[1] * self.error_sum, 9) #* dt
        #if (dt != 0):
        dControl = round(self.p[2] * (error - self.error_old), 9) #/ dt)
        self.error_old = error

        #print("pControl: ", pControl)
        #print("dControl: ", dControl)
        #print("iControl: ", iControl)

        pid = pControl + dControl + iControl

        return pid

    # for case if flag : None
    def get_pre_setpoint(self):
        return self.pre_setpoint
