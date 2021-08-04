use super::{quat_left, quat_right, Constraint, ConstraintConfig};
use crate::{
    body::BodyArena,
    math::{lcp_gauss_seidel, MatMN, MatN, VecN},
};
use glam::{Mat4, Quat, Vec3, Vec4};

pub struct ConstraintConstantVelocity {
    config: ConstraintConfig,
    // the initial relative quaternion q1^-1 * q2
    q0: Quat,
    jacobian: MatMN<2, 12>,
    cached_lambda: VecN<2>,
    baumgarte: f32,
}

impl ConstraintConstantVelocity {
    pub fn new(config: ConstraintConfig, q0: Quat) -> Self {
        Self {
            config,
            q0,
            jacobian: MatMN::zero(),
            cached_lambda: VecN::zero(),
            baumgarte: 0.0,
        }
    }
}

impl Constraint for ConstraintConstantVelocity {
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

        // get the orientation information of the bodies
        let q1 = body_a.orientation;
        let q2 = body_b.orientation;
        let q0_inv = self.q0.inverse();
        let q1_inv = q1.inverse();

        // the axis is defined in the local space of body_a
        let cv = self.config.axis_a;

        let p = Mat4::from_cols(Vec4::ZERO, Vec4::Y, Vec4::Z, Vec4::W);
        let p_t = p.transpose(); // pointless but self documenting

        let mat_a = p * quat_left(q1_inv) * quat_right(q2 * q0_inv) * p_t * -0.5;
        let mat_b = p * quat_left(q1_inv) * quat_right(q2 * q0_inv) * p_t * 0.5;

        self.jacobian = MatMN::zero();

        // distance constraint
        {
            let j1 = (a - b) * 2.0;
            self.jacobian.rows[0][0] = j1.x;
            self.jacobian.rows[0][1] = j1.y;
            self.jacobian.rows[0][2] = j1.z;

            let j2 = ra.cross((a - b) * 2.0);
            self.jacobian.rows[0][3] = j2.x;
            self.jacobian.rows[0][4] = j2.y;
            self.jacobian.rows[0][5] = j2.z;

            let j3 = (b - a) * 2.0;
            self.jacobian.rows[0][6] = j3.x;
            self.jacobian.rows[0][7] = j3.y;
            self.jacobian.rows[0][8] = j3.z;

            let j4 = rb.cross((b - a) * 2.0);
            self.jacobian.rows[0][9] = j4.x;
            self.jacobian.rows[0][10] = j4.y;
            self.jacobian.rows[0][11] = j4.z;
        }

        // the quaternion jacobian
        {
            const IDX: usize = 1;

            let j1 = Vec3::ZERO;
            self.jacobian.rows[1][0] = j1.x;
            self.jacobian.rows[1][1] = j1.y;
            self.jacobian.rows[1][2] = j1.z;

            let tmp = mat_a * Vec4::from((0.0, cv)) * -0.5;
            let j2 = Vec3::new(tmp[IDX + 0], tmp[IDX + 1], tmp[IDX + 2]);
            self.jacobian.rows[1][3] = j2.x;
            self.jacobian.rows[1][4] = j2.y;
            self.jacobian.rows[1][5] = j2.z;

            let j3 = Vec3::ZERO;
            self.jacobian.rows[1][6] = j3.x;
            self.jacobian.rows[1][7] = j3.y;
            self.jacobian.rows[1][8] = j3.z;

            let tmp = mat_b * Vec4::from((0.0, cv)) * 0.5;
            let j4 = Vec3::new(tmp[IDX + 0], tmp[IDX + 1], tmp[IDX + 2]);
            self.jacobian.rows[1][9] = j4.x;
            self.jacobian.rows[1][10] = j4.y;
            self.jacobian.rows[1][11] = j4.z;
        }

        // apply warm starting from last frame
        let impulses = self.jacobian.transpose() * self.cached_lambda;
        self.config.apply_impulses(bodies, impulses);

        // calculate the baumgarte stabilization
        let c = r.dot(r);
        let c = f32::max(0.0, c - 0.01);
        const BETA: f32 = 0.05;
        self.baumgarte = (BETA / dt_sec) * c;
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
        for cached_lambda in self.cached_lambda.iter_mut() {
            if !cached_lambda.is_finite() {
                *cached_lambda = 0.0
            }

            const LIMIT: f32 = 20.0;
            if *cached_lambda > LIMIT {
                *cached_lambda = LIMIT;
            }
            if *cached_lambda < -LIMIT {
                *cached_lambda = -LIMIT;
            }
        }
    }
}

pub struct ConstraintConstantVelocityLimited {
    config: ConstraintConfig,
    // the initial relative quaternion q1^-1 * q2
    q0: Quat,
    jacobian: MatMN<4, 12>,
    cached_lambda: VecN<4>,
    baumgarte: f32,
    angle_u: f32,
    angle_v: f32,
    is_angle_violated_u: bool,
    is_angle_violated_v: bool,
}

impl ConstraintConstantVelocityLimited {
    pub fn new(config: ConstraintConfig, q0: Quat) -> Self {
        Self {
            config,
            q0,
            jacobian: MatMN::zero(),
            cached_lambda: VecN::zero(),
            baumgarte: 0.0,
            angle_u: 0.0,
            angle_v: 0.0,
            is_angle_violated_u: false,
            is_angle_violated_v: false,
        }
    }
}

impl Constraint for ConstraintConstantVelocityLimited {
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

        // get the orientation information of the bodies
        let q1 = body_a.orientation;
        let q2 = body_b.orientation;
        let q0_inv = self.q0.inverse();
        let q1_inv = q1.inverse();

        // the axis is defined in the local space of body_a
        let cv = self.config.axis_a;
        let (u, v) = cv.any_orthonormal_pair();

        let p = Mat4::from_cols(Vec4::ZERO, Vec4::Y, Vec4::Z, Vec4::W);
        let p_t = p.transpose(); // pointless but self documenting

        let mat_a = p * quat_left(q1_inv) * quat_right(q2 * q0_inv) * p_t * -0.5;
        let mat_b = p * quat_left(q1_inv) * quat_right(q2 * q0_inv) * p_t * 0.5;

        // check the constraint's angular limits
        let qr = q1_inv * q2;
        let qrr = qr * q0_inv;
        self.angle_u = 2.0 * qrr.xyz().dot(u).asin().to_degrees();
        self.angle_v = 2.0 * qrr.xyz().dot(v).asin().to_degrees();

        self.is_angle_violated_u = self.angle_u.abs() > 45.0;
        self.is_angle_violated_v = self.angle_v.abs() > 45.0;

        self.jacobian = MatMN::zero();

        // first row is primary distance constraint that holds the anchor points together
        {
            let j1 = (a - b) * 2.0;
            self.jacobian.rows[0][0] = j1.x;
            self.jacobian.rows[0][1] = j1.y;
            self.jacobian.rows[0][2] = j1.z;

            let j2 = ra.cross((a - b) * 2.0);
            self.jacobian.rows[0][3] = j2.x;
            self.jacobian.rows[0][4] = j2.y;
            self.jacobian.rows[0][5] = j2.z;

            let j3 = (b - a) * 2.0;
            self.jacobian.rows[0][6] = j3.x;
            self.jacobian.rows[0][7] = j3.y;
            self.jacobian.rows[0][8] = j3.z;

            let j4 = rb.cross((b - a) * 2.0);
            self.jacobian.rows[0][9] = j4.x;
            self.jacobian.rows[0][10] = j4.y;
            self.jacobian.rows[0][11] = j4.z;
        }

        // the quaternion jacobian
        {
            const IDX: usize = 1;

            let j1 = Vec3::ZERO;
            self.jacobian.rows[1][0] = j1.x;
            self.jacobian.rows[1][1] = j1.y;
            self.jacobian.rows[1][2] = j1.z;

            let tmp = mat_a * Vec4::from((0.0, cv)) * -0.5;
            let j2 = Vec3::new(tmp[IDX + 0], tmp[IDX + 1], tmp[IDX + 2]);
            self.jacobian.rows[1][3] = j2.x;
            self.jacobian.rows[1][4] = j2.y;
            self.jacobian.rows[1][5] = j2.z;

            let j3 = Vec3::ZERO;
            self.jacobian.rows[1][6] = j3.x;
            self.jacobian.rows[1][7] = j3.y;
            self.jacobian.rows[1][8] = j3.z;

            let tmp = mat_b * Vec4::from((0.0, cv)) * 0.5;
            let j4 = Vec3::new(tmp[IDX + 0], tmp[IDX + 1], tmp[IDX + 2]);
            self.jacobian.rows[1][9] = j4.x;
            self.jacobian.rows[1][10] = j4.y;
            self.jacobian.rows[1][11] = j4.z;
        }

        if self.is_angle_violated_u {
            const IDX: usize = 1;

            let j1 = Vec3::ZERO;
            self.jacobian.rows[2][0] = j1.x;
            self.jacobian.rows[2][1] = j1.y;
            self.jacobian.rows[2][2] = j1.z;

            let tmp = mat_a * Vec4::from((0.0, u)) * -0.5;
            let j2 = Vec3::new(tmp[IDX + 0], tmp[IDX + 1], tmp[IDX + 2]);
            self.jacobian.rows[2][3] = j2.x;
            self.jacobian.rows[2][4] = j2.y;
            self.jacobian.rows[2][5] = j2.z;

            let j3 = Vec3::ZERO;
            self.jacobian.rows[2][6] = j3.x;
            self.jacobian.rows[2][7] = j3.y;
            self.jacobian.rows[2][8] = j3.z;

            let tmp = mat_b * Vec4::from((0.0, u)) * 0.5;
            let j4 = Vec3::new(tmp[IDX + 0], tmp[IDX + 1], tmp[IDX + 2]);
            self.jacobian.rows[2][9] = j4.x;
            self.jacobian.rows[2][10] = j4.y;
            self.jacobian.rows[2][11] = j4.z;
        }

        if self.is_angle_violated_v {
            const IDX: usize = 1;

            let j1 = Vec3::ZERO;
            self.jacobian.rows[3][0] = j1.x;
            self.jacobian.rows[3][1] = j1.y;
            self.jacobian.rows[3][2] = j1.z;

            let tmp = mat_a * Vec4::from((0.0, v)) * -0.5;
            let j2 = Vec3::new(tmp[IDX + 0], tmp[IDX + 1], tmp[IDX + 2]);
            self.jacobian.rows[3][3] = j2.x;
            self.jacobian.rows[3][4] = j2.y;
            self.jacobian.rows[3][5] = j2.z;

            let j3 = Vec3::ZERO;
            self.jacobian.rows[3][6] = j3.x;
            self.jacobian.rows[3][7] = j3.y;
            self.jacobian.rows[3][8] = j3.z;

            let tmp = mat_b * Vec4::from((0.0, v)) * 0.5;
            let j4 = Vec3::new(tmp[IDX + 0], tmp[IDX + 1], tmp[IDX + 2]);
            self.jacobian.rows[3][9] = j4.x;
            self.jacobian.rows[3][10] = j4.y;
            self.jacobian.rows[3][11] = j4.z;
        }

        // apply warm starting from last frame
        let impulses = self.jacobian.transpose() * self.cached_lambda;
        self.config.apply_impulses(bodies, impulses);

        // calculate the baumgarte stabilization
        let c = r.dot(r);
        let c = f32::max(0.0, c - 0.01);
        const BETA: f32 = 0.05;
        self.baumgarte = (BETA / dt_sec) * c;
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
        let mut lambda_n = lcp_gauss_seidel(&MatN::from(j_w_jt), &rhs);

        // clamp the torque from the angle constraint
        // we need to make sure it's a restorative torque
        if self.is_angle_violated_u {
            if self.angle_u > 0.0 {
                lambda_n[2] = f32::min(0.0, lambda_n[2]);
            } else if self.angle_u < 0.0 {
                lambda_n[2] = f32::max(0.0, lambda_n[2]);
            }
        }
        if self.is_angle_violated_v {
            if self.angle_v > 0.0 {
                lambda_n[3] = f32::min(0.0, lambda_n[3]);
            } else if self.angle_v < 0.0 {
                lambda_n[3] = f32::max(0.0, lambda_n[3]);
            }
        }

        // apply the impulses
        let impulses = jacobian_transpose * lambda_n;
        self.config.apply_impulses(bodies, impulses);

        // accumulate the impulses for warm starting
        self.cached_lambda += lambda_n;
    }

    fn post_solve(&mut self) {
        // limit the warm starting to reasonable limits
        for cached_lambda in self.cached_lambda.iter_mut() {
            if !cached_lambda.is_finite() {
                *cached_lambda = 0.0
            }

            const LIMIT: f32 = 20.0;
            if *cached_lambda > LIMIT {
                *cached_lambda = LIMIT;
            }
            if *cached_lambda < -LIMIT {
                *cached_lambda = -LIMIT;
            }
        }
    }
}
