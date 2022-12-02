
import numpy as np 
import random

class Maze():
    def __init__(self, map_width=12, map_height=12):
        self.map_width = map_width
        self.map_height = map_height
        self.maze =  np.random.randint(0, 5, size=(self.map_width+1, self.map_height+1))

    def distance_to_walls(self, x, y):
        '''
        Measure the distance of coordinates to nearest walls at four directions.
        Return: (up, right, down, left)
        '''
        i = int(y // self.map_height)
        j = int(x // self.map_width)
        d1 = y - y // self.map_height * self.map_height
        while self.permissibilities(cell = (i,j))[0]:
            i -= 1
            d1 += self.map_height

        i = int(y // self.map_height)
        j = int(x // self.map_width)
        d2 = self.map_width - (x - x // self.map_width * self.map_width)
        while self.permissibilities(cell = (i,j))[1]:
            j += 1
            d2 += self.map_width

        i = int(y // self.map_height)
        j = int(x // self.map_width)
        d3 = self.map_height - (y - y // self.map_height * self.map_height)
        while self.permissibilities(cell = (i,j))[2]:
            i += 1
            d3 += self.map_height

        i = int(y // self.map_height)
        j = int(x // self.map_width)
        d4 = x - x // self.map_width * self.map_width
        while self.permissibilities(cell = (i,j))[3]:
            j -= 1
            d4 += self.map_width

        return [d1, d2, d3, d4]
    
    def permissibilities(self, cell):
        '''
        Check if the directions of a given cell are permissible.
        Return: (up, right, down, left)
        '''
        x, y = cell 
        if(cell[0] > self.map_width or cell[0]< 0):
            x = self.map_width -1
        if(cell[1] > self.map_height or cell[1]< 0):
            y = self.map_height -1

        cell_value = self.maze[x, y]
        return (cell_value & 1 == 0, cell_value & 2 == 0, cell_value & 3 == 0, cell_value & 4 == 0)
    

class Particle():
    def __init__(self, maze, x, y, heading = None, weight = 1.0, sensor_limit = None, noisy = False):
        if heading is None:
            heading = np.random.uniform(0, 360)

        self.x = x
        self.y = y
        self.maze = maze
        self.heading = heading
        self.weight = weight
        self.sensor_limit = sensor_limit

        if noisy:
            std = max(self.maze.map_height, self.maze.map_width) * 0.2
            self.x = self.add_noise(x = self.x, std = std)
            self.y = self.add_noise(x = self.y, std = std)
            self.heading = self.add_noise(x = self.heading, std = 360 * 0.05)
        self.fix_invalid_particles()


    def fix_invalid_particles(self):
        # Fix invalid particles
        if self.x < 0:
            self.x = 0
        if self.x > self.weight:
            self.x = self.weight * 0.9999
        if self.y < 0:
            self.y = 0
        if self.y > self.maze.map_height:
            self.y = self.maze.map_height * 0.9999
        self.heading = self.heading % 360

    def state(self):
        return (self.x, self.y, self.heading)

    def add_noise(self, x, std):
        return x + np.random.normal(0, std)

    def read_sensor(self):

        readings = self.maze.distance_to_walls(self.x, self.y)
        heading = self.heading % 360
        # Remove the compass from particle
        if heading >= 45 and heading < 135:
            readings = readings
        elif heading >= 135 and heading < 225:
            readings = readings[-1:] + readings[:-1]
        elif heading >= 225 and heading < 315:
            readings = readings[-2:] + readings[:-2]
        else:
            readings = readings[-3:] + readings[:-3]
        if self.sensor_limit is not None:
            for i in range(len(readings)):
                if readings[i] > self.sensor_limit:
                    readings[i] = self.sensor_limit

        return readings

    def try_move(self, speed, noisy = False):

        heading = self.heading
        heading_rad = np.radians(heading)

        dx = np.sin(heading_rad) * speed
        dy = np.cos(heading_rad) * speed

        x = self.x + dx
        y = self.y + dy

        gj1 = int(self.x //self.maze.map_width)
        gi1 = int(self.y //self.maze.map_height)
        gj2 = int(x //self.maze.map_width)
        gi2 = int(y //self.maze.map_height)

        # Check if the particle is still in the maze
        if gi2 < 0 or gi2 >= self.maze.map_width or gj2 < 0 or gj2 >= self.maze.map_height:
            return False

        # Move in the same grid
        if gi1 == gi2 and gj1 == gj2:
            self.x = x
            self.y = y
            return True

        # Move across one grid vertically
        elif abs(gi1 - gi2) == 1 and abs(gj1 - gj2) == 0:
            if self.maze.maze[min(gi1, gi2), gj1] & 4 != 0:
                return False
            else:
                self.x = x
                self.y = y
                return True
        
        # Move across one grid horizonally
        elif abs(gi1 - gi2) == 0 and abs(gj1 - gj2) == 1:
            if self.maze.maze[gi1, min(gj1, gj2)] & 2 != 0:
                return False
            else:
                self.x = x
                self.y = y
                return True
        
        # Move across grids both vertically and horizonally
        elif abs(gi1 - gi2) == 1 and abs(gj1 - gj2) == 1:

            x0 = max(gj1, gj2) * self.maze.map_width
            y0 = (y - self.y) / (x - self.x) * (x0 - self.x) + self.y

            if self.maze.maze[int(y0 //self.maze.map_height), min(gj1, gj2)] & 2 != 0:
                return False

            y0 = max(gi1, gi2) *self.maze.map_height
            x0 = (x - self.x) / (y - self.y) * (y0 - self.y) + self.x

            if self.maze.maze[min(gi1, gi2), int(x0 // self.maze.map_width)] & 4 != 0:
                return False

            self.x = x
            self.y = y
            return True

        else:
            raise Exception('Unexpected collision detection.')

class Robot(Particle):
    def __init__(self, x, y, maze, heading = None, speed = 1.0, sensor_limit = None, noisy = True):
        super(Robot, self).__init__(x = x, y = y, maze = maze, heading = heading, sensor_limit = sensor_limit, noisy = noisy)
        self.step_count = 0
        self.noisy = noisy
        self.time_step = 0
        self.speed = speed

    def choose_random_direction(self):
        self.heading = np.random.uniform(0, 360)

    def add_sensor_noise(self, x, z = 0.05):
        readings = list(x)
        for i in range(len(readings)):
            std = readings[i] * z / 2
            readings[i] = readings[i] + np.random.normal(0, std)
        return readings

    def read_sensor(self):
        readings = super(Robot, self).read_sensor()
        if self.noisy == True:
            readings = self.add_sensor_noise(x = readings)
        return readings

    def move(self):
        while True:
            self.time_step += 1
            if self.try_move(speed = self.speed, noisy = False):
                break
            self.choose_random_direction()


    def weight_gaussian_kernel(self, x1, x2, std = 10):
        # TODO to generate weights, use weighted Gaussian kernel, i.e, np.exp(-(|x1-x2|)^2/(2*std)) 
        return np.exp(-(abs(x1-x2))**2/(2*std)) 


class ResamplingDistribution(object):
    def __init__(self, particles):
        accum = 0.0
        self.particles = particles
        self.distribution = list()
        # TODO append cumulative sum of the weights to self.distribution 
        # A cumulative sum array is one whose value at each index is the sum of all previous indexes plus itself
        # (e.g., [1,2,3,4] becomes [1,3,6,10] )
        self.distribution.append(np.cumsum(self.particles))

        self.N = len(self.particles)
        positions = (np.random.random() + np.arange(self.N)) / self.N
        self.indexes = np.zeros(self.N, 'i')
        i, j = 0, 0
        while i < self.N:
            if positions[i] < self.distribution[j]:
                self.indexes[i] = j
                i += 1
            else:
                j += 1

    def random_select(self):
        index = int(np.random.uniform(0, self.N))
        return self.particles[self.indexes[index]]


x_init = 2
y_init = 4
robot_speed = 2.1 
num_particles = 100
kernel_sigma = 500 # Standard deviation for Gaussian distance kernel
num_iter = 100
maze = Maze(20, 20)
particle = Particle(maze, x_init, y_init)
particle.try_move(2.0)
particle.read_sensor()

x = np.random.uniform(0, maze.map_width)
y = np.random.uniform(0, maze.map_height)

sensor_limit_ratio = 0.3 # distance limit of sensors (real value: 0 - 1). 0: useless sensor; 1: perfect sensor
sensor_limit =  sensor_limit_ratio*max(maze.map_height, maze.map_width)

bob = Robot(x = x, y = y, maze = maze, speed = robot_speed, sensor_limit = sensor_limit)

particles = list()
for i in range(num_particles):
    x = np.random.uniform(0, maze.map_width)
    y = np.random.uniform(0, maze.map_height)
    particles.append(Particle(x = x, y = y, maze = maze, sensor_limit = sensor_limit))

for j in range(0, num_iter):

        readings_robot = bob.read_sensor()
        particle_weight_total = 0
        for particle in particles:
            readings_particle = particle.read_sensor()
            # TODO to calculate weights, decide what would be for x1 and x2, think over how to estimate innovation or measurement error 
            particle.weight = bob.weight_gaussian_kernel(x1 = , x2 = ,  std = kernel_sigma)
            # TODO update the total particle weight 
            particle_weight_total = particle.weight / np.sum(weights)

        # make sure normalization is not divided by zero
        if particle_weight_total == 0:
            particle_weight_total = 1e-8

        # TODO normalize the each of the particle weights
        
       
        # Resampling particles
        distribution = ResamplingDistribution(particles = particles)
        particles_new = list()

        for i in range(num_particles):
            particle = distribution.random_select()
            particles_new.append(Particle(x = particle.x, y = particle.y, maze = maze, heading = particle.heading, sensor_limit = sensor_limit, noisy = True))

        particles = particles_new

        heading_old = bob.heading
        bob.move()
        heading_new = bob.heading
        dh = heading_new - heading_old

        for particle in particles:
            particle.heading = (particle.heading + dh) % 360
            particle.try_move(speed = bob.speed)
