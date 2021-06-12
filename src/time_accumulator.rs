use bevy::utils::Duration;

pub struct TimeAccumulator {
    accumulated_time: Duration,
    frame_number: u64,
    num_steps: u32,
    max_steps: u32,
    update_rate: Duration,
    time_dilation: f64,
}

impl TimeAccumulator {
    pub fn new() -> Self {
        TimeAccumulator {
            accumulated_time: Duration::from_nanos(0),
            frame_number: 0,
            num_steps: 0,
            max_steps: 4,
            update_rate: Duration::from_secs(1) / 60,
            time_dilation: 1.0,
        }
    }

    pub fn update(&mut self, delta: Duration) {
        let modified_delta = if self.time_dilation < 1.0 {
            let divisor = (1.0 / self.time_dilation) as u32;
            delta / divisor
        } else {
            delta * self.time_dilation as u32
        };
        self.frame_number += 1;
        self.accumulated_time += modified_delta;
        self.num_steps = (self.accumulated_time.as_nanos() / self.update_rate.as_nanos()) as u32;
        if self.num_steps > self.max_steps {
            eprintln!(
                "capping physics steps {} from time {} accumulated {} at rate {}",
                self.num_steps,
                delta.as_secs_f64(),
                self.accumulated_time.as_secs_f64(),
                self.update_rate.as_secs_f64(),
            );
            self.accumulated_time = Duration::from_nanos(0);
            self.num_steps = self.max_steps;
        } else {
            self.accumulated_time -= self.update_rate * self.num_steps;
        }
    }

    pub fn step_secs(&self) -> f32 {
        self.update_rate.as_secs_f32()
    }

    pub fn num_steps(&self) -> u32 {
        self.num_steps
    }

    pub fn time_dilation(&self) -> f64 {
        self.time_dilation
    }

    pub fn set_time_dilation(&mut self, dilation: f64) {
        self.time_dilation = dilation;
    }
}
