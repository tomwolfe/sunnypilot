class FirstOrderFilter:
  def __init__(self, x0, rc, dt, initialized=True):
    self.x = x0
    self.dt = dt
    self.rc = rc
    self.alpha = dt / (rc + dt)  # Pre-compute alpha
    self.one_minus_alpha = 1.0 - self.alpha  # Pre-compute for performance
    self.initialized = initialized

  def update_alpha(self, rc):
    self.rc = rc
    self.alpha = self.dt / (rc + self.dt)
    self.one_minus_alpha = 1.0 - self.alpha

  def update(self, x):
    if self.initialized:
      self.x = self.one_minus_alpha * self.x + self.alpha * x
    else:
      self.initialized = True
      self.x = x
    return self.x
