import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

import board
import busio
import adafruit_pca9685
import operator

DC_OFF = 0
# DC_MIN = 24575
DC_MIN = 25575
DC_MAX = 65535
DC_DIFF = DC_MAX - DC_MIN
MOTORS = 4
FREQUENCY = 100
# top to bottom, left  to  right
FORWARD = [13, 15, 2, 0]
BACKWARD = [12, 14, 3, 1]


def setupPCA(self):
    self.i2c = busio.I2C(board.SCL, board.SDA)
    self.pca = adafruit_pca9685.PCA9685(self.i2c)
    self.pca.frequency = FREQUENCY
    # print('pca is setup!')

class PiCarDriver(Node):

    def __init__(self):
        super().__init__('pi_car_driver')
        setupPCA(self)
        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.driver_callback, 5)

    def driver_callback(self, msg):
        global activated, timeout
        x = msg.linear.x
        y = msg.angular.z
        # print('x = ' , x)
        # print('y = ' , y)
        if(x == 0 and y == 0):
            breaking(self)
        else:
            xy = abs(x) + abs(y)
            # check for > 1
            if(xy > 1):
                x = x / abs(xy)
                y = y / abs(xy)

            # print('x = ' , x)
            # print('y = ' , y)
            
            acceleration_front = [x, x, x, x]
            acceleration_left = [y, y, -y, -y]
            acceleration = list(map(operator.sub, acceleration_front, acceleration_left))
            print(acceleration)
            # acceleration = indexwise_add(acceleration_front, acceleration_left)
            

            
            # print('acceleration: ', [v /2 for v in acceleration])
            # set_acceleration(self, [v /2 for v in acceleration])
            set_acceleration(self, acceleration)
        

    
    def __del__(self):
        # destructor
        if self.pca is not None:
            self.pca.deinit()
        if self.i2c is not None:
            self.i2c.deinit()


def indexwise_add(a, b):
    return [sum(x) for x in zip(a, b)]

def reset_engine(self):
    breaking(self)


def breaking(self):
    global activated
    for i in range(MOTORS):
        self.pca.channels[(FORWARD[i])].duty_cycle = DC_OFF
        self.pca.channels[(BACKWARD[i])].duty_cycle = DC_OFF


def set_acceleration(self, acc):
    print(acc)
    # self.pca.channels[15].duty_cycle = DC_MAX
    for i in range(MOTORS):
        if (acc[i] >= 0):
            # print(int(acc[i]*DC_DIFF))
            # print(DC_MAX + int(acc[i]*DC_DIFF))
            self.pca.channels[FORWARD[i]].duty_cycle = DC_MIN + int(acc[i]*DC_DIFF)
            # self.pca.channels[FORWARD[i]].duty_cycle = DC_MIN
            self.pca.channels[BACKWARD[i]].duty_cycle = DC_OFF
    
        else:
            self.pca.channels[FORWARD[i]].duty_cycle = DC_OFF
            self.pca.channels[BACKWARD[i]].duty_cycle = DC_MIN - int(acc[i]*DC_DIFF)


def main(args=None):
    print('pi_car is seting up!')
    rclpy.init(args=args)
    pi_car_driver = PiCarDriver()
    rclpy.spin(pi_car_driver)

    pi_car_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
