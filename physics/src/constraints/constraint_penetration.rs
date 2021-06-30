use super::{ConstraintConfig, ConstraintTrait};
use crate::{
    body::BodyArena,
    math::{lcp_gauss_seidel, MatMN, MatN, VecN},
};
use glam::Vec3;

pub struct ConstraintPenetration {
    jacobian: MatMN<3, 12>,
    cached_lambda: VecN<3>,
    normal: Vec3, // in body A's local space
    baumgarte: f32,
    friction: f32,
}

impl ConstraintPenetration {
    pub fn new(normal: Vec3) -> Self {
        Self {
            jacobian: MatMN::zero(),
            cached_lambda: VecN::zero(),
            normal,
            baumgarte: 0.0,
            friction: 0.0,
        }
    }
}

impl ConstraintTrait for ConstraintPenetration {
    fn pre_solve(&mut self, config: &ConstraintConfig, bodies: &mut BodyArena, dt_sec: f32) {
        let body_a = bodies.get_body(config.handle_a);
        let body_b = bodies.get_body(config.handle_b);

        // get the world space position of the hinge from body_a's orientation
        let world_anchor_a = body_a.local_to_world(config.anchor_a);

        // get the world space position of the hinge from body_b's orientation
        let world_anchor_b = body_b.local_to_world(config.anchor_b);

        let ra = world_anchor_a - body_a.centre_of_mass_world();
        let rb = world_anchor_b - body_b.centre_of_mass_world();
        let a = world_anchor_a;
        let b = world_anchor_b;

        self.friction = body_a.friction * body_b.friction;

        // should be equivalent to Vec3::GetOrtho() from the book
        let (mut u, mut v) = self.normal.any_orthonormal_pair();

        // convert tangent space from model space to world space
        let normal = body_a.orientation * self.normal;
        u = body_a.orientation * u;
        v = body_a.orientation * v;

        // penetration constraint
        self.jacobian = MatMN::zero();

        // first row is the primary distance constraint that holds the anchor points together
        {
            let j1 = -normal;
            self.jacobian.rows[0][0] = j1.x;
            self.jacobian.rows[0][1] = j1.y;
            self.jacobian.rows[0][2] = j1.z;
        }

        {
            let j2 = ra.cross(-normal);
            self.jacobian.rows[0][3] = j2.x;
            self.jacobian.rows[0][4] = j2.y;
            self.jacobian.rows[0][5] = j2.z;
        }

        {
            let j3 = normal;
            self.jacobian.rows[0][6] = j3.x;
            self.jacobian.rows[0][7] = j3.y;
            self.jacobian.rows[0][8] = j3.z;
        }

        {
            let j4 = rb.cross(normal);
            self.jacobian.rows[0][9] = j4.x;
            self.jacobian.rows[0][10] = j4.y;
            self.jacobian.rows[0][11] = j4.z;
        }

        // friction jacobians
        if self.friction > 0.0 {
            {
                let j1 = -u;
                self.jacobian.rows[1][0] = j1.x;
                self.jacobian.rows[1][1] = j1.y;
                self.jacobian.rows[1][2] = j1.z;
            }
            {
                let j2 = ra.cross(-u);
                self.jacobian.rows[1][3] = j2.x;
                self.jacobian.rows[1][4] = j2.y;
                self.jacobian.rows[1][5] = j2.z;
            }
            {
                let j3 = u;
                self.jacobian.rows[1][6] = j3.x;
                self.jacobian.rows[1][7] = j3.y;
                self.jacobian.rows[1][8] = j3.z;
            }
            {
                let j4 = rb.cross(u);
                self.jacobian.rows[1][9] = j4.x;
                self.jacobian.rows[1][10] = j4.y;
                self.jacobian.rows[1][11] = j4.z;
            }

            {
                let j1 = -v;
                self.jacobian.rows[2][0] = j1.x;
                self.jacobian.rows[2][1] = j1.y;
                self.jacobian.rows[2][2] = j1.z;
            }
            {
                let j2 = ra.cross(-v);
                self.jacobian.rows[2][3] = j2.x;
                self.jacobian.rows[2][4] = j2.y;
                self.jacobian.rows[2][5] = j2.z;
            }
            {
                let j3 = v;
                self.jacobian.rows[2][6] = j3.x;
                self.jacobian.rows[2][7] = j3.y;
                self.jacobian.rows[2][8] = j3.z;
            }
            {
                let j4 = rb.cross(v);
                self.jacobian.rows[2][9] = j4.x;
                self.jacobian.rows[2][10] = j4.y;
                self.jacobian.rows[2][11] = j4.z;
            }
        }

        // apply warm starting from last frame
        let impulses = self.jacobian.transpose() * self.cached_lambda;
        config.apply_impulses(bodies, impulses);

        // calculate the baumgarte stabilization
        let mut c = (b - a).dot(normal);
        c = f32::min(0.0, c + 0.02); // add slop
        let beta = 0.25;
        self.baumgarte = beta * c / dt_sec;
    }

    fn solve(&mut self, config: &ConstraintConfig, bodies: &mut BodyArena) {
        let jacobian_transpose = self.jacobian.transpose();

        // build the system of equations
        let q_dt = config.get_velocities(bodies);
        let inv_mass_matrix = config.get_inverse_mass_matrix(bodies);
        let j_w_jt = self.jacobian * inv_mass_matrix * jacobian_transpose;
        let mut rhs = self.jacobian * q_dt * -1.0;
        rhs[0] -= self.baumgarte;

        // solve for the Lagrange multipliers
        let mut lambda_n = lcp_gauss_seidel(&MatN::from(j_w_jt), &rhs);

        // accumulate the impulses and clamp within the constraint limits
        let old_lambda = self.cached_lambda;
        self.cached_lambda += lambda_n;
        let lambda_limit = 0.0;
        if self.cached_lambda[0] < lambda_limit {
            self.cached_lambda[0] = lambda_limit;
        }

        if self.friction > 0.0 {
            let body_a = bodies.get_body(config.handle_a);
            let body_b = bodies.get_body(config.handle_b);
            let umg = self.friction * 10.0 * 1.0 / (body_a.inv_mass + body_b.inv_mass);
            let normal_force = (lambda_n[0] * self.friction).abs();
            let max_force = if umg > normal_force {
                umg
            } else {
                normal_force
            };

            if self.cached_lambda[1] > max_force {
                self.cached_lambda[1] = max_force;
            }
            if self.cached_lambda[1] < -max_force {
                self.cached_lambda[1] = -max_force;
            }

            if self.cached_lambda[2] > max_force {
                self.cached_lambda[2] = max_force;
            }
            if self.cached_lambda[2] < -max_force {
                self.cached_lambda[2] = -max_force;
            }
        }
        lambda_n = self.cached_lambda - old_lambda;

        // apply the impulses
        let impulses = jacobian_transpose * lambda_n;
        config.apply_impulses(bodies, impulses);
    }
}
