from led_scheduler import LedScheduler

class FoodRegion(object):

    def __init__(self, params):
        self.params = dict(params)
        self.led_scheduler = LedScheduler(self.params['led'])

    @property
    def index(self):
        return self.params['index']

    def update(self,t,fly_position):
        contains_fly = self.contains(fly_position)
        self.led_scheduler.update(t,contains_fly)
        if contains_fly:
            print('region {} contains fly, led = {}'.format(self.index, self.led_scheduler.led_on))

    def contains(self, x):
        if abs(x - self.params['position']) < 0.5*self.params['width']:
            return True
        else:
            return False
