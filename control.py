import matplotlib.pyplot as plt
import time
class PIDController:
    def __init__(self, Kp, Ki, Kd, interval, limits, initial, filter = False):
        """

        :param Kp: proportional gain
        :param Ki: intergral gain
        :param Kd: differential gain
        :param interval: interval between control inputs in seconds
        :param limits:  2 element list, first element lower limit, second element upper limit
        """
        # Set proportional, integral and differential gains for PID control
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.filter = filter
        self.interval = interval
        # current output
        if self.interval == 0:
            raise RuntimeError("Interval set to zero, PID controller will attempt to divide by zero")

        # Set limits on
        self.lim_high = limits[1]
        self.lim_low = limits[0]

        #Variables to store previous values of system input and error signal for PID control
        self.initial = initial
        self.u_k1 = 0
        self.e_k1 = 0
        self.e_k2 = 0
        self.e_k3 = 0
        self.e_k4 = 0
        self.reference = None
        self.saturation_flag = False

        # Low pass filter for derivative
        self.lpf = LPF(1.08484579, self.interval)

        # Initialise diagnostic plots and variables
        self.error = []
        self.output = []
        self.saved_u = []
        self.filterederror = []
        # Uncomment for debugging plots
        # self.f, (self.ax1, self.ax2) = plt.subplots(2, 1, sharex=True)
        self.first_run_flag=True
        self.time = time.time()
    def get_output(self, measurement: float) -> float:
        '''PID CONTROL
        '''
        self.interval = time.time() - self.time
        self.time = time.time()

        if self.interval == 0:
            raise ZeroDivisionError
        e_k = self.reference - measurement  # Calculate error signal for current time-step

        if self.first_run_flag:
            self.u_k1 = e_k*self.Kp # on first run when initial value is the initial value set add it on
            self.initial = 0 # set the initial value to zero for subsequent calls
            self.e_k4 = e_k
            self.e_k3= e_k
            self.e_k2= e_k
            self.e_k1= e_k
            self.first_run_flag = False

        if self.interval == 0:
            raise RuntimeError("Interval set to zero, PID controller attempting to divide by zero")

        #compute the low pass filtered value of e_k2
        if self.filter:
            filtered_e = self.lpf.filter([self.e_k4, self.e_k3, self.e_k2, self.e_k1, e_k])
            print("filtering")
        else:
            filtered_e = [self.e_k4, self.e_k3, self.e_k2, self.e_k1, e_k]
        e_k1_f = filtered_e[3]
        e_k2_f = filtered_e[2]

        # if self.e_k2 is not None and self.e_k2 != 0:
        # Calculate input for current time-step using PID control algorithm
        u_k = self.u_k1 + ((self.Kp + (self.interval * self.Ki) + (self.Kd /self.interval)) * e_k - (
                    self.Kp + 2 * (self.Kd / self.interval)) * e_k1_f + (
                                       self.Kd / self.interval) * e_k2_f)

        # else:
        #     # If required error signals not yet defined, set output to initial
        #     u_k = self.u_k1

        # Enforce saturation limits. If the output saturates, set the output at the previous timestep equal to the
        # computed output,(without integral action to prevent integral windup)in order that at the next iteration the
        # backwards difference calculation holds.

        if u_k < self.lim_low:
            self.saturation_flag = True
            self.u_k1 = self.u_k1 + ((self.Kp + (self.Kd / self.interval)) * e_k -
                                    self.Kp * self.e_k1 + 2 * (self.Kd / self.interval) * e_k1_f +
                                     (self.Kd / self.interval) * e_k2_f)
            u_k = self.lim_low
        elif u_k > self.lim_high:
            self.saturation_flag = True
            self.u_k1 = self.u_k1 + ((self.Kp + (self.Kd / self.interval)) * e_k -
                                    self.Kp * self.e_k1 + 2 * (self.Kd / self.interval) * e_k1_f +
                                     (self.Kd / self.interval) * e_k2_f)
            u_k = self.lim_high
        else:
            self.saturation_flag = False
            self.u_k1 = u_k

        # Update stored input and error signals
        self.e_k4 = self.e_k3
        self.e_k3 = self.e_k2
        self.e_k2 = self.e_k1
        self.e_k1 = e_k
        # #diagnostic plots
        # # todo this plotting code throws a IndexError quite regularly
        # self.ax1.cla()
        # self.ax2.cla()
        # self.error.append(e_k)
        # self.output.append(u_k)
        #
        # self.saved_u.append(self.u_k1)
        # self.u_k1 = u_k #trying this as a solution to loop issues
        #
        # self.filterederror.append(filtered_e[2])
        # self.ax1.plot(self.saved_u)
        # self.ax1.plot(self.output)
        #
        # self.ax2.plot(self.error)
        # self.ax2.plot(self.filterederror)
        # self.ax2.legend(["error","filtered error"])
        # self.ax1.set_ylabel("Output")
        # self.ax2.set_ylabel("Error")
        # plt.draw()

        return u_k

    def set_reference(self, reference):
        self.reference = reference


class OnOffController:
    def __init__(self, limits, hystersisBand=0.2):
        """

        :param hystersisBand:
        :param limits: 2 element list, first element lower limit, second element higher limit
        """
        self.band = hystersisBand  # Define bounds of hysteresis band
        self.limits = limits

    def set_reference(self, ref):
        self.ref = ref

    def get_output(self, measurement):
        if measurement >= self.ref + self.band:  # If above hysteresis band, set to lower limit
            return self.limits[0]
        elif measurement <= self.ref - self.band:  # If below hysteresis band, set to upper limit
            return self.limits[1]

class LPF:
    def __init__(self, TimeConstant, interval):
        self.interval = interval
        self.Tc = TimeConstant
        self.a = self.interval/(self.Tc+self.interval)

    def filter(self, samples):
        if 0 in samples:
            return samples
        output = [None] * len(samples)
        output[0] = self.a * samples[0]
        for i in range(1, len(samples)):
            output[i] = self.a*samples[i] + (1-self.a)*output[i-1]
        return output
