class DCMotorPositionSystem:

    def __init__(self, R=2.0, L=0.5, J=0.02, b=0.1, Kt=0.01, Ke=0.01):
        self.R, self.L, self.J, self.b, self.Kt, self.Ke = R, L, J, b, Kt, Ke
        self.theta = 0.0
        self.omega = 0.0
        self.current = 0.0

    def step(self, voltage_input, dt=0.001):
        # State equations: di/dt = (V - R*i - Ke*omega) / L
        di_dt = (voltage_input - (self.R * self.current) - (self.Ke * self.omega)) / self.L
        # d(omega)/dt = (Kt*i - b*omega) / J
        domega_dt = ((self.Kt * self.current) - (self.b * self.omega)) / self.J

        self.current += di_dt * dt
        self.omega += domega_dt * dt
        self.theta += self.omega * dt
        return self.theta