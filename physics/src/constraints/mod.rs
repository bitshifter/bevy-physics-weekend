#![allow(dead_code)]
mod constraint_distance;

use crate::{
    math::{MatMN, VecN},
    scene::{BodyHandle, PhysicsScene},
};
use glam::Vec3;

pub trait ConstraintTrait {
    fn pre_solve(&mut self, config: &ConstraintConfig, scene: &PhysicsScene, dt_sec: f32);
    fn solve(&mut self, config: &ConstraintConfig, scene: &mut PhysicsScene);
    fn post_solve(&mut self) {}
}

pub struct ConstraintConfig {
    constraint: Box<dyn ConstraintTrait>,

    body_a: BodyHandle,
    body_b: BodyHandle,

    anchor_a: Vec3, // the anchor location in body_a's space
    axis_a: Vec3,   // the axis direction in body_a's space

    anchor_b: Vec3, // the anchor location in body_b's space
    axis_b: Vec3,   // the axis direction in body_b's space
}

pub struct Constraint {
    constraint: Box<dyn ConstraintTrait>,
    config: ConstraintConfig,
}

impl ConstraintConfig {
    fn get_inverse_mass_matrix(&self, scene: &PhysicsScene) -> MatMN<12, 12> {
        let mut inv_mass_matrix = MatMN::zero();

        {
            let body_a = scene.get_body(&self.body_a);

            inv_mass_matrix.cols[0][0] = body_a.inv_mass;
            inv_mass_matrix.cols[1][1] = body_a.inv_mass;
            inv_mass_matrix.cols[2][2] = body_a.inv_mass;

            let inv_intertia_a = body_a.inv_intertia_tensor_world();
            for i in 0..3 {
                inv_mass_matrix.cols[3 + i][3 + 0] = inv_intertia_a.col(i)[0];
                inv_mass_matrix.cols[3 + i][3 + 1] = inv_intertia_a.col(i)[1];
                inv_mass_matrix.cols[3 + i][3 + 2] = inv_intertia_a.col(i)[2];
            }
        }

        {
            let body_b = scene.get_body(&self.body_b);
            inv_mass_matrix.cols[6][6] = body_b.inv_mass;
            inv_mass_matrix.cols[7][7] = body_b.inv_mass;
            inv_mass_matrix.cols[8][8] = body_b.inv_mass;

            let inv_intertia_b = body_b.inv_intertia_tensor_world();
            for i in 0..3 {
                inv_mass_matrix.cols[9 + i][9 + 0] = inv_intertia_b.col(i)[0];
                inv_mass_matrix.cols[9 + i][9 + 1] = inv_intertia_b.col(i)[1];
                inv_mass_matrix.cols[9 + i][9 + 2] = inv_intertia_b.col(i)[2];
            }
        }

        inv_mass_matrix
    }

    fn get_velocities(&self, scene: &PhysicsScene) -> VecN<12> {
        let mut q_dt = VecN::zero();

        {
            let body_a = scene.get_body(&self.body_a);

            q_dt[0] = body_a.linear_velocity.x;
            q_dt[1] = body_a.linear_velocity.y;
            q_dt[2] = body_a.linear_velocity.z;

            q_dt[3] = body_a.angular_velocity.x;
            q_dt[4] = body_a.angular_velocity.y;
            q_dt[5] = body_a.angular_velocity.z;
        }

        {
            let body_b = scene.get_body(&self.body_b);

            q_dt[6] = body_b.linear_velocity.x;
            q_dt[7] = body_b.linear_velocity.y;
            q_dt[8] = body_b.linear_velocity.z;

            q_dt[9] = body_b.angular_velocity.x;
            q_dt[10] = body_b.angular_velocity.y;
            q_dt[11] = body_b.angular_velocity.z;
        }

        q_dt
    }

    fn apply_impulses(&self, scene: &mut PhysicsScene, impulses: &VecN<12>) {
        {
            let force_internal_a = Vec3::from_slice(&impulses[0..]);
            let torque_internal_a = Vec3::from_slice(&impulses[3..]);
            let body_a = scene.get_body_mut(&self.body_a);
            body_a.apply_impulse_linear(force_internal_a);
            body_a.apply_impulse_angular(torque_internal_a);
        }

        {
            let force_internal_b = Vec3::from_slice(&impulses[6..]);
            let torque_internal_b = Vec3::from_slice(&impulses[9..]);
            let body_b = scene.get_body_mut(&self.body_b);
            body_b.apply_impulse_linear(force_internal_b);
            body_b.apply_impulse_angular(torque_internal_b);
        }
    }
}
