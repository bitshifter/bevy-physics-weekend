use super::{Constraint, ConstraintConfig};
use crate::{
    body::BodyArena,
    math::{lcp_gauss_seidel, MatMN, MatN, VecN},
};

pub struct ConstraintDistance {
    config: ConstraintConfig,
    jacobian: MatMN<1, 12>,
    cached_lambda: VecN<1>,
    baumgarte: f32,
}

impl ConstraintDistance {
    pub fn new(config: ConstraintConfig) -> Self {
        ConstraintDistance {
            config,
            jacobian: MatMN::zero(),
            cached_lambda: VecN::zero(),
            baumgarte: 0.0,
        }
    }
}

impl Constraint for ConstraintDistance {
    fn pre_solve(&mut self, bodies: &mut BodyArena, dt_sec: f32) {
        let body_a = bodies.get_body(self.config.handle_a);
        let body_b = bodies.get_body(self.config.handle_b);

        // get the world space position of the hinge from body_a's orientation
        let world_anchor_a = body_a.local_to_world(self.config.anchor_a);

        // get the world space position of the hinge from body_b's orientation
        let world_anchor_b = body_b.local_to_world(self.config.anchor_b);

        let r = world_anchor_b - world_anchor_a;
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

        // apply warm starting from the last frame
        let impulses = self.jacobian.transpose() * self.cached_lambda;
        self.config.apply_impulses(bodies, impulses);

        // calculate the baumgarte stabilization
        let mut c = r.dot(r);
        c = f32::max(0.0, c - 0.01);
        let beta = 0.05;
        self.baumgarte = (beta / dt_sec) * c;
    }

    fn solve(&mut self, bodies: &mut BodyArena) {
        let jacobian_transpose = self.jacobian.transpose();

        // build the system of equations
        let q_dt = self.config.get_velocities(bodies);
        let inv_mass_matrix = self.config.get_inverse_mass_matrix(bodies);
        let j_w_jt = self.jacobian * inv_mass_matrix * jacobian_transpose;
        let mut rhs = self.jacobian * q_dt * -1.0;
        rhs[0] -= self.baumgarte;

        // solve for the Lagrange multipliers
        let lambda_n = lcp_gauss_seidel(&MatN::from(j_w_jt), &rhs);

        // apply the impulses
        let impulses = jacobian_transpose * lambda_n;
        self.config.apply_impulses(bodies, impulses);

        // accumulate the impulses for warm starting
        self.cached_lambda += lambda_n;
    }

    fn post_solve(&mut self) {
        // limit the warm starting to reasonable limits
        if !self.cached_lambda[0].is_finite() {
            self.cached_lambda[0] = 0.0
        }

        const LIMIT: f32 = 1e5;
        if self.cached_lambda[0] > LIMIT {
            self.cached_lambda[0] = LIMIT;
        }
        if self.cached_lambda[0] < -LIMIT {
            self.cached_lambda[0] = -LIMIT;
        }
    }
}
