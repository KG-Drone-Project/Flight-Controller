pub struct Pid {
    kp: f32,
    ki: f32,
    kd: f32,

    previous_error: f32,
    integral: f32,
    derivative: f32,
}

impl Pid {
    pub fn new(kp: f32, ki: f32, kd: f32) -> Self {
        Pid {
            kp, ki, kd,
            previous_error: 0.,
            integral: 0.,
            derivative: 0.,
        }
    }

    pub fn compute(&mut self, error: f32, dt: f32) -> f32 {
        self.integral = (self.integral + error * dt).clamp(-1.0, 1.0);
        self.derivative = (error - self.previous_error) / dt;
        self.previous_error = error;

        self.kp * error + self.ki * self.integral + self.kd * self.derivative
    }

    pub fn reset(&mut self) {
        self.previous_error = 0.0;
        self.integral = 0.0;
    }

    pub fn set_tunings(&mut self, kp: f32, ki: f32, kd: f32) {
        self.kp = kp;
        self.ki = ki;
        self.kd = kd;
    }
}
