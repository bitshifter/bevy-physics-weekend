use super::{ConstraintConfig, ConstraintTrait};
use crate::{
    math::{lcp_gauss_seidel, MatMN, MatN},
    scene::BodyArena,
};

pub struct ConstraintDistance {
    jacobian: MatMN<1, 12>,
}

impl ConstraintTrait for ConstraintDistance {
    fn pre_solve(&mut self, config: &ConstraintConfig, bodies: &BodyArena, _dt_sec: f32) {
        let body_a = bodies.get_body(config.body_a);
        let body_b = bodies.get_body(config.body_b);

        let world_anchor_a = body_a.local_to_world(config.anchor_a);
        let world_anchor_b = body_b.local_to_world(config.anchor_b);

        let ra = world_anchor_a - body_a.centre_of_mass_world();
        let rb = world_anchor_b - body_b.centre_of_mass_world();
        let a = world_anchor_a;
        let b = world_anchor_b;

        {
            let j1 = (a - b) * 2.0;
            self.jacobian.rows[0][0] = j1.x;
            self.jacobian.rows[0][1] = j1.y;
            self.jacobian.rows[0][2] = j1.z;
        }

        {
            let j2 = ra.cross((a - b) * 2.0);
            self.jacobian.rows[0][3] = j2.x;
            self.jacobian.rows[0][4] = j2.y;
            self.jacobian.rows[0][5] = j2.z;
        }

        {
            let j3 = (b - a) * 2.0;
            self.jacobian.rows[0][6] = j3.x;
            self.jacobian.rows[0][7] = j3.y;
            self.jacobian.rows[0][8] = j3.z;
        }

        {
            let j4 = rb.cross((b - a) * 2.0);
            self.jacobian.rows[0][9] = j4.x;
            self.jacobian.rows[0][10] = j4.y;
            self.jacobian.rows[0][11] = j4.z;
        }
    }

    fn solve(&mut self, config: &ConstraintConfig, bodies: &mut BodyArena) {
        let jacobian_transpose = self.jacobian.transpose();

        // build the system of equations
        let q_dt = config.get_velocities(bodies);
        let inv_mass_matrix = config.get_inverse_mass_matrix(bodies);
        let j_w_jt = self.jacobian * inv_mass_matrix * jacobian_transpose;
        let rhs = self.jacobian * q_dt * -1.0;

        // solve for the Lagrange multipliers
        let lambda_n = lcp_gauss_seidel(&MatN::from(j_w_jt), &rhs);

        // apply the impulses
        let impulses = jacobian_transpose * lambda_n;
        config.apply_impulses(bodies, &impulses);
    }
}
