
pub struct KalmanFilter {

    // Posterior state estimate
    x_post: [f32; 2],

    // Prior state estimate
    x_prior: [f32; 2],

    p: [f32; 4],

    q_ang: f32,

    q_gyro: f32,

    r: f32,

}


impl KalmanFilter {
    pub fn new() -> Self {
        KalmanFilter {
            q_ang: 0.001,
            q_gyro: 0.003,
            r: 0.03,

            x_post: [0., 0.],
            x_prior: [0., 0.],

            p: [0., 0., 0., 0.],
        }
    }
    /*
    pub fn predict(&mut self, new_rate: f32, dt:f32) {

        // theta + dt (new gyro - gyro bias)
        self.x_prior[0] += dt * (new_rate - self.x_post[1]);

        self.p_prior[0] += dt * (dt * self.p_post[3] - self.p_post[1] - self.p_post[2] + self.q_ang);
        self.p_prior[1] -= dt * self.p_post[3];
        self.p_prior[2] -= dt * self.p_post[3];
        self.p_prior[3] += self.q_gyro * dt;
    }

    pub fn correct(&mut self, new_angle: f32) -> f32 {

        let s: f32 = self.p_prior[0] + self.r;

        // Kalman gain
        let mut k: [f32; 2] = [0., 0.];

        k[0] = self.p_prior[0] / s;
        k[1] = self.p_prior[2] / s;

        let y = new_angle - self.x_prior[0];

        self.x_post[0] += k[0] * y;
        self.x_post[1] += k[1] * y;

        let p00_temp = self.p_prior[0];
        let p01_temp = self.p_prior[1];

        self.p_post[0] = self.p_prior[0] - k[0] * p00_temp;
        self.p_post[1] = self.p_prior[1] - k[0] * p01_temp;
        self.p_post[2] = self.p_prior[2] - k[1] * p00_temp;
        self.p_post[3] = self.p_prior[3] - k[1] * p01_temp;

        return self.x_post[0];
    }*/

    pub fn process_posterior_state(&mut self, new_rate: f32, new_angle:f32, dt:f32) {

        // theta + dt (new gyro - gyro bias)
        self.x_prior[0] += dt * (new_rate - self.x_post[1]);

        self.p[0] += dt * (dt * self.p[3] - self.p[1] - self.p[2] + self.q_ang);
        self.p[1] -= dt * self.p[3];
        self.p[2] -= dt * self.p[3];
        self.p[3] += self.q_gyro * dt;

        let s: f32 = self.p[0] + self.r;

        // Kalman gain
        let mut k: [f32; 2] = [0., 0.];

        k[0] = self.p[0] / s;
        k[1] = self.p[2] / s;

        let y = new_angle - self.x_prior[0];

        self.x_post[0] = self.x_prior[0] + k[0] * y;
        self.x_post[1] = self.x_prior[1] + k[1] * y;

        let p00_temp = self.p[0];
        let p01_temp = self.p[1];

        self.p[0] -= k[0] * p00_temp;
        self.p[1] -= k[0] * p01_temp;
        self.p[2] -= k[1] * p00_temp;
        self.p[3] -= k[1] * p01_temp;

    }

    pub fn get_angle(&mut self) -> f32 {
        return self.x_post[0];
    }
}
