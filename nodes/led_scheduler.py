from led_pwm_proxy import LedPwmProxy
import rospy

class LedScheduler(object):

    def __init__(self,param) :
        self.param = dict(param)

        # State
        self.led_on = False 
        self.last_on_t  = -(self.param['refractory_duration'] + self.param['on_duration'])
        self.activation_count = 0

        self.led_proxy = LedPwmProxy()
        self.turn_off_led()

    def __del__(self):
        self.turn_off_led()

    @property
    def value(self):
        if self.led_on: 
            return self.param['on_value']
        else:
            return self.param['off_value']
    @property
    def pin(self):
        return self.param['pin']
    
    def turn_on_led(self,t):
        if not self.led_on:
            self.led_proxy.set_value(self.param['pin'],self.param['on_value'])
            self.activation_count += 1
            self.led_on = True
            self.last_on_t = t

    def turn_off_led(self):
        self.led_proxy.set_value(self.param['pin'], self.param['off_value'])
        self.led_on = False

    def update(self, t, inside_region): 
        if self.led_on:
            if (t - self.last_on_t) > self.param['on_duration']:
                self.turn_off_led()
        else:
            if inside_region:
                if (t - self.last_on_t) > (self.param['on_duration'] + self.param['refractory_duration']):
                    self.turn_on_led(t)   





