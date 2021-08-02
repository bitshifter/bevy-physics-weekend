use super::{Constraint, ConstraintConfig};
use crate::body::BodyArena;

pub struct ConstraintMoverSimple {
    config: ConstraintConfig,
    time: f32,
}

impl ConstraintMoverSimple {
    pub fn new(config: ConstraintConfig) -> Self {
        ConstraintMoverSimple { config, time: 0.0 }
    }
}

impl Constraint for ConstraintMoverSimple {
    fn pre_solve(&mut self, bodies: &mut BodyArena, dt_sec: f32) {
        self.time += dt_sec;

        let body_a = bodies.get_body_mut(self.config.handle_a);
        body_a.linear_velocity.z = f32::cos(self.time * 0.25) * 4.0;
    }

    fn solve(&mut self, _bodies: &mut BodyArena) {}
}
