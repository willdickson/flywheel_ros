from led_scheduler import LedScheduler

class FoodRegion(object):

    def __init__(self, param):
        self.param = dict(param)
        self.led_scheduler = LedScheduler(self.param['led'])

    @property
    def index(self):
        return self.param['index']

    @property
    def position(self):
        return self.param['position']

    @property
    def width(self):
        return self.param['width']

    def update(self,t,fly_position, led_enabled=True):
        contains_fly = self.contains(fly_position)
        if led_enabled:
            self.led_scheduler.update(t,contains_fly)
        if contains_fly:
            print('region {} contains fly, led = {}'.format(self.index, self.led_scheduler.led_on))

    def contains(self, x):
        if abs(x - self.param['position']) < 0.5*self.param['width']:
            return True
        else:
            return False
