class MassSpringDamper:

    def __init__(self, m, k, b):
        self.m = m
        self.k = k
        self.b = b
        self.x = 0.0  # Current Position State Value
        self.v = 0.0  # Current Velocity State Value

    def step(self, force, dt=0.01):
        # Acceleration derived from system state equations: a = (F - b*v - k*x) / m
        a = (force - (self.b * self.v) - (self.k * self.x)) / self.m
        self.v += a * dt
        self.x += self.v * dt
        return self.x