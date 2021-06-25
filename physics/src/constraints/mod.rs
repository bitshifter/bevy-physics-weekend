mod constraint_distance;

use crate::{
    body::{BodyArena, BodyHandle},
    math::{MatMN, VecN},
};
use constraint_distance::ConstraintDistance;
use glam::Vec3;

pub struct ConstraintArena {
    constraints: Vec<Constraint>,
}

impl ConstraintArena {
    pub fn new() -> Self {
        ConstraintArena {
            constraints: Vec::new(),
        }
    }

    pub fn clear(&mut self) {
        self.constraints.clear();
    }

    pub fn add_distance_constraint(&mut self, config: ConstraintConfig) {
        let constraint = Constraint {
            constraint: Box::new(ConstraintDistance::new()),
            config,
        };
        self.constraints.push(constraint);
    }

    pub fn solve(&mut self, bodies: &mut BodyArena, dt_sec: f32, max_iters: u32) {
        for constraint in &mut self.constraints {
            constraint.pre_solve(bodies, dt_sec);
        }

        for _ in 0..max_iters {
            for constraint in &mut self.constraints {
                constraint.solve(bodies);
            }
        }

        for constraint in &mut self.constraints {
            constraint.post_solve();
        }
    }
}

pub trait ConstraintTrait: Send + Sync {
    fn pre_solve(&mut self, config: &ConstraintConfig, bodies: &mut BodyArena, dt_sec: f32);
    fn solve(&mut self, config: &ConstraintConfig, bodies: &mut BodyArena);
    fn post_solve(&mut self) {}
}

#[derive(Copy, Clone, Debug)]
pub struct ConstraintConfig {
    pub handle_a: BodyHandle,
    pub handle_b: BodyHandle,

    pub anchor_a: Vec3, // the anchor location in body_a's space
    pub axis_a: Vec3,   // the axis direction in body_a's space

    pub anchor_b: Vec3, // the anchor location in body_b's space
    pub axis_b: Vec3,   // the axis direction in body_b's space
}

pub struct Constraint {
    constraint: Box<dyn ConstraintTrait>,
    config: ConstraintConfig,
}

impl Constraint {
    pub fn pre_solve(&mut self, bodies: &mut BodyArena, dt_sec: f32) {
        self.constraint.pre_solve(&self.config, bodies, dt_sec);
    }
    pub fn solve(&mut self, bodies: &mut BodyArena) {
        self.constraint.solve(&self.config, bodies);
    }
    pub fn post_solve(&mut self) {
        self.constraint.post_solve();
    }
}

impl ConstraintConfig {
    fn get_inverse_mass_matrix(&self, bodies: &BodyArena) -> MatMN<12, 12> {
        let mut inv_mass_matrix = MatMN::zero();

        {
            let body_a = bodies.get_body(self.handle_a);

            inv_mass_matrix.rows[0][0] = body_a.inv_mass;
            inv_mass_matrix.rows[1][1] = body_a.inv_mass;
            inv_mass_matrix.rows[2][2] = body_a.inv_mass;

            let inv_intertia_a = body_a.inv_intertia_tensor_world();
            for i in 0..3 {
                inv_mass_matrix.rows[3 + i][3] = inv_intertia_a.col(i)[0];
                inv_mass_matrix.rows[3 + i][3 + 1] = inv_intertia_a.col(i)[1];
                inv_mass_matrix.rows[3 + i][3 + 2] = inv_intertia_a.col(i)[2];
            }
        }

        {
            let body_b = bodies.get_body(self.handle_b);
            inv_mass_matrix.rows[6][6] = body_b.inv_mass;
            inv_mass_matrix.rows[7][7] = body_b.inv_mass;
            inv_mass_matrix.rows[8][8] = body_b.inv_mass;

            let inv_intertia_b = body_b.inv_intertia_tensor_world();
            for i in 0..3 {
                inv_mass_matrix.rows[9 + i][9] = inv_intertia_b.col(i)[0];
                inv_mass_matrix.rows[9 + i][9 + 1] = inv_intertia_b.col(i)[1];
                inv_mass_matrix.rows[9 + i][9 + 2] = inv_intertia_b.col(i)[2];
            }
        }

        inv_mass_matrix
    }

    fn get_velocities(&self, bodies: &BodyArena) -> VecN<12> {
        let mut q_dt = VecN::zero();

        {
            let body_a = bodies.get_body(self.handle_a);

            q_dt[0] = body_a.linear_velocity.x;
            q_dt[1] = body_a.linear_velocity.y;
            q_dt[2] = body_a.linear_velocity.z;

            q_dt[3] = body_a.angular_velocity.x;
            q_dt[4] = body_a.angular_velocity.y;
            q_dt[5] = body_a.angular_velocity.z;
        }

        {
            let body_b = bodies.get_body(self.handle_b);

            q_dt[6] = body_b.linear_velocity.x;
            q_dt[7] = body_b.linear_velocity.y;
            q_dt[8] = body_b.linear_velocity.z;

            q_dt[9] = body_b.angular_velocity.x;
            q_dt[10] = body_b.angular_velocity.y;
            q_dt[11] = body_b.angular_velocity.z;
        }

        q_dt
    }

    fn apply_impulses(&self, bodies: &mut BodyArena, impulses: &VecN<12>) {
        {
            let force_internal_a = Vec3::from_slice(&impulses[0..]);
            let torque_internal_a = Vec3::from_slice(&impulses[3..]);
            let body_a = bodies.get_body_mut(self.handle_a);
            body_a.apply_impulse_linear(force_internal_a);
            body_a.apply_impulse_angular(torque_internal_a);
        }

        {
            let force_internal_b = Vec3::from_slice(&impulses[6..]);
            let torque_internal_b = Vec3::from_slice(&impulses[9..]);
            let body_b = bodies.get_body_mut(self.handle_b);
            body_b.apply_impulse_linear(force_internal_b);
            body_b.apply_impulse_angular(torque_internal_b);
        }
    }
}
