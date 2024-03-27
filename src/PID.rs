pub struct PID {
    Kp: f32,
    Ki: f32,
    Kd: f32,

    prev_error: f32,
    integral: f32,
    derivative: f32,
}

impl PID {
    pub fn new(Kp: f32, Ki: f32, Kd: f32) -> Self {
        PID {
            Kp, Ki, Kd,
            prev_error: 0.,
            integral: 0.,
            derivative: 0.,
        }
    }

    pub fn compute(&mut self, error: f32, dt: f32) -> f32 {
        self.integral = (self.integral + error * dt).clamp(-1.0, 1.0);
        let derivative = (error - self.previous_error) / dt;
        self.previous_error = error;

        self.Kp * error + self.Ki * self.integral + self.Kd * derivative
    }

    pub fn reset(&mut self) {
        self.previous_error = 0.0;
        self.integral = 0.0;
    }

    pub fn set_tunings(&mut self, Kp: f32, Ki: f32, Kd: f32) {
        self.Kp = Kp;
        self.Ki = Ki;
        self.Kd = Kd;
    }
}
