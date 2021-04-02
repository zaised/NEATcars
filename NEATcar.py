import os
from math import sin, copysign, radians, degrees, sqrt, cos
import pygame
from pygame.math import Vector2
import random
import pickle
import neat

pygame.init()
pygame.font.init()

WIN_WIDTH = 1280
WIN_HEIGHT = 720
PPU = 32
GEN = 0
SENS_LENGTH = 7

CAR_IMAGE = pygame.image.load(os.path.join("imgs", "car.png"))

FONT = pygame.font.SysFont('Calibri', 15)


class Sensor:

    def __init__(self, posintion, angle):
        self.position = Vector2(posintion.x, posintion.y)
        self.angle = angle
        self.dir = sin(radians(self.angle))
        self.length = SENS_LENGTH

    def find_end(self):
        x = self.position.x + (self.length * cos(radians(self.angle)))
        y = self.position.y + (self.length * sin(radians(self.angle)))

        return Vector2(x, y)

    def collision(self):
        if (0 <= abs(self.dir) <= 0.001) & (self.length <= 2):
            return True
        if (abs(self.dir) == 1) & (self.length <= 1.1):
            return True
        if (0.4999 <= abs(self.dir) <= 0.5001) & (self.length <= 2.24):
            return True
        return False


class Car:

    def __init__(self, x, y, angle=90, length=4, max_steering=30, max_acceleration=5.0):
        self.position = Vector2(x, y)  # координаты автомобиля
        self.velocity = Vector2(0.0, 0.0)  # скорость автомобиля в м/с
        self.angle = angle  # поворот автомобиля относительно поверхности
        self.length = length  # длинна автомобиля
        self.max_steering = max_steering  # максимальный угол поворота колёс в градусах
        self.max_acceleration = max_acceleration  # максималльное ускорение в м/с
        self.max_velocity = 20  # максимальная скорость автомобиля
        self.brake_deceleration = 10  # отрицательное ускроение при нажатии на тормоз
        self.traction_deceleration = 2  # отрицательное ускорение вызванное трением

        self.acceleration = 0.0  # текущее ускорение в м/с
        self.steering = 0.0  # текущий угол поворота колёс в градусах

        # сенсоры
        self.sensors = [0, 1, 2, 3, 4, 5, 6, 7]
        self.sensors[0] = Sensor(self.position, 0)
        self.sensors[1] = Sensor(self.position, -30)
        self.sensors[2] = Sensor(self.position, 30)
        self.sensors[3] = Sensor(self.position, 90)
        self.sensors[4] = Sensor(self.position, -90)
        self.sensors[5] = Sensor(self.position, 180)
        self.sensors[6] = Sensor(self.position, -150)
        self.sensors[7] = Sensor(self.position, 150)

    # dt - дельта времени

    def update(self, dt):
        self.velocity += (self.acceleration*dt, 0)
        self.velocity.x = max(-self.max_velocity, min(self.velocity.x, self.max_velocity))

        # Радиус поворота равен длинне автомобиля делённой на синус угла поворота
        if self.steering:
            turninng_radius = self.length / sin(radians(self.steering))
            angular_velocity = self.velocity.x / turninng_radius
        else:
            angular_velocity = 0

        self.position += self.velocity.rotate(-self.angle) * dt
        self.angle += degrees(angular_velocity) * dt

        for sensor in self.sensors:
            sensor.position = self.position

        self.sensors[0].angle = -self.angle
        self.sensors[1].angle = -self.angle - 30
        self.sensors[2].angle = -self.angle + 30
        self.sensors[3].angle = -self.angle + 90
        self.sensors[4].angle = -self.angle - 90
        self.sensors[5].angle = -self.angle + 180
        self.sensors[6].angle = -self.angle - 150
        self.sensors[7].angle = -self.angle + 150


class Target:
    def __init__(self, x):

        self.LENGTH = 6
        self.WIDTH = 4

        # direction
        if (random.randint(0, 100) < 0):
            self.a = self.WIDTH
            self.WIDTH = self.LENGTH
            self.LENGTH = self.a

        self.position = Vector2(x+self.LENGTH/2, 15+self.WIDTH/2)
        self.walls = [0, 1, 2, 3, 4, 5, 6, 7, 8]

        self.walls[0] = Vector2(0, 15)
        self.walls[1] = Vector2(x, 15)
        self.walls[2] = Vector2(x, 15+self.LENGTH)
        self.walls[3] = Vector2(x+self.WIDTH, 15+self.LENGTH)
        self.walls[4] = Vector2(x+self.WIDTH, 15)
        self.walls[5] = Vector2(40, 15)
        self.walls[6] = Vector2(40, 0)
        self.walls[7] = Vector2(0, 0)
        self.walls[8] = Vector2(0, 15)

        self.wall_left_start = Vector2(0, 10)
        self.wall_left_end = Vector2(x, 10)

        self.wall_right_start = Vector2(x+self.LENGTH, 10)
        self.wall_right_end = Vector2(40, 10)

        self.wall_bottom_start = Vector2(x, 10 + self.WIDTH)
        self.wall_bottom_end = Vector2(x + self.LENGTH, 10 + self.WIDTH)


class Agent:
    def __init__(self, target, car):
        self.target = target
        self.car = car
        self.pr = self.get_distance()

    def get_distance(self):
        dist = abs(sqrt((self.target.position.x - self.car.position.x) ** 2 + (self.target.position.y - self.car.position.y) ** 2))
        return dist

    def get_sensor_distance(self, i):
        sensor = self.car.sensors[i]
        sensor.length = SENS_LENGTH

        y1 = sensor.position.y
        y2 = sensor.find_end().y
        x1 = sensor.position.x
        x2 = sensor.find_end().x

        dir1 = Vector2(x2, y2)-Vector2(x1, y1)
        a1 = -dir1.y
        b1 = dir1.x
        d1 = -(a1*x1+b1*y1)

        min = SENS_LENGTH

        # inrage
        for i in range(1, 9):
            y3 = self.target.walls[i].y
            y4 = self.target.walls[i-1].y
            x3 = self.target.walls[i].x
            x4 = self.target.walls[i-1].x

            dir2 = Vector2(x4, y4)-Vector2(x3, y3)
            a2 = -dir2.y
            b2 = dir2.x
            d2 = -(a2*x3+b2*y3)

            seg1_start = a2*x1+b2*y1+d2
            seg1_end = a2*x2+b2*y2+d2

            seg2_start = a1*x3+b1*y3+d1
            seg2_end = a1*x4+b1*y4+d1

            if ((seg1_start * seg1_end >= 0) | (seg2_start * seg2_end >= 0)):
                length = SENS_LENGTH
            else:
                u = seg1_start / (seg1_start - seg1_end)
                intersect = sensor.position + u*dir1
                length = abs(sqrt((x1 - intersect.x) ** 2 + (y1 - intersect.y) ** 2))

            if min > length:
                min = length

        return min

    def draw(self, win):

        for i in range(0, 8):
            pygame.draw.line(win, (255, 255, 255), self.car.sensors[i].position * PPU, self.car.sensors[i].find_end() * PPU)
            pygame.draw.circle(win, (255, 0, 0), (int(self.car.sensors[i].find_end().x * PPU), int(self.car.sensors[i].find_end().y*PPU)), 2)

        rotated = pygame.transform.rotate(CAR_IMAGE, self.car.angle)
        rect = rotated.get_rect()
        win.blit(rotated, self.car.position * PPU - (rect.width / 2, rect.height / 2))


def draw_window(win, target, agents):
    win.fill((100, 100, 100))
    for agent in agents:
        agent.draw(win)

    for i in range(1, 9):
        pygame.draw.line(win, (0, 0, 0), target.walls[i-1] * PPU, target.walls[i] * PPU)

    pygame.display.flip()


def main(genomes, config):
    global GEN
    GEN += 1
    # списки сетей, геномов и агентов
    nets = []
    ge = []
    agents = []

    target = Target(25)
    # создание агентов
    for _, g in genomes:
        net = neat.nn.FeedForwardNetwork.create(g, config)
        nets.append(net)
        agent = Agent(target, Car(15, 5, 0))
        agents.append(agent)
        g.fitness = 0
        ge.append(g)

    win = pygame.display.set_mode((WIN_WIDTH, WIN_HEIGHT))
    pygame.display.set_caption("Car Parking")

    clock = pygame.time.Clock()
    ticks = 60
    timer = 0

    run = True
    while run:
        clock.tick(ticks)
        dt = clock.get_time()/1000
        timer += dt

        # Events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
                pygame.quit()

        if len(agents) <= 0:
            run = False
            break

        for i, agent in enumerate(agents):

            # Logic
            for j in range(0, 8):
                agent.car.sensors[j].length = agent.get_sensor_distance(j)
            agent.car.update(dt)

            # Input
            output = nets[i].activate((agent.car.sensors[0].length, agent.car.sensors[1].length,
                                       agent.car.sensors[2].length, agent.car.sensors[3].length,
                                       agent.car.sensors[4].length, agent.car.sensors[5].length,
                                       agent.car.sensors[6].length, agent.car.sensors[7].length,
                                       agent.car.acceleration, agent.car.steering))

            if output[0] > 0.5:  # положительное ускорение
                if agent.car.acceleration < 0:
                    agent.car.acceleration = 0
                else:
                    agent.car.acceleration += 10 * dt
            elif output[1] > 0.5:  # отрицательное ускорение
                if agent.car.acceleration > 0:
                    agent.car.acceleration = 0
                else:
                    agent.car.acceleration -= 10 * dt
            elif output[2] > 0.5:  # торможение
                if abs(agent.car.velocity.x) > dt * agent.car.brake_deceleration:
                    agent.car.acceleration = -copysign(agent.car.brake_deceleration, agent.car.velocity.x)
                else:
                    agent.car.acceleration = -agent.car.velocity.x / dt
            else:
                if abs(agent.car.velocity.x) > dt * agent.car.traction_deceleration:
                    agent.car.acceleration = -copysign(agent.car.traction_deceleration, agent.car.velocity.x)
                else:
                    if dt != 0:
                        agent.car.acceleration = -agent.car.velocity.x / dt
            # ограничение максимального ускорения
            agent.car.acceleration = max(-agent.car.max_acceleration, min(agent.car.acceleration, agent.car.max_acceleration))

            if output[3] > 0.5:
                agent.car.steering -= 60 * dt  # поворот колёс на 30 градусов вправо за секунду
            elif output[4] > 0.5:
                agent.car.steering += 60 * dt  # поворот колёс на 30 градусов влево за секунду
            else:
                agent.car.steering = 0
            # ограничение максимального угла поворота
            agent.car.steering = max(-agent.car.max_steering, min(agent.car.steering, agent.car.max_steering))

            fitness_delta = 0
            collided = False
            cr = agent.get_distance()
            if cr <= 1:
                if agent.car.acceleration <= 0.01:
                    fitness_delta = 10000
                else:
                    fitness_delta = 10
            elif cr < agent.pr:
                fitness_delta = 1/cr
                agent.pr = cr

            for sensor in agent.car.sensors:
                if sensor.collision():
                    fitness_delta = -100
                    collided = True

            ge[i].fitness += fitness_delta
            if collided or timer > 15:
                agents.pop(i)
                nets.pop(i)
                ge.pop(i)

        draw_window(win, target, agents)


def run(config_path):
    config = neat.config.Config(neat.DefaultGenome, neat.DefaultReproduction,
                                neat.DefaultSpeciesSet, neat.DefaultStagnation,
                                config_path)

    pop = neat.Population(config)

    pop.add_reporter(neat.StdOutReporter(True))
    stats = neat.StatisticsReporter()
    pop.add_reporter(stats)

    winner = pop.run(main, 50)
    with open("winner.pkl", "wb") as f:
        pickle.dump(winner, f)
        f.close()

    quit()


def load_genome(config_path):

    with open("winner.pkl", "rb") as f:
        genome = pickle.load(f)

    config = neat.config.Config(neat.DefaultGenome, neat.DefaultReproduction,
                                neat.DefaultSpeciesSet, neat.DefaultStagnation,
                                config_path)
    genomes = [(1, genome)]
    main(genomes, config)


if __name__ == '__main__':
    local_dir = os.path.dirname(__file__)
    config_path = os.path.join(local_dir, "config-feedforward.txt")
    run(config_path)
    load_genome(config_path)
