use super::{quat_left, quat_right, Constraint, ConstraintConfig};
use crate::{
    body::BodyArena,
    math::glam_ext::QuatExt,
    math::{lcp_gauss_seidel, MatMN, MatN, VecN},
};
use glam::{Mat4, Quat, Vec3, Vec4};

pub struct ConstraintMotor {
    config: ConstraintConfig,
    jacobian: MatMN<4, 12>,
    q0: Quat,
    motor_axis: Vec3,
    baumgarte: Vec3,
    motor_speed: f32,
}

impl ConstraintMotor {
    pub fn new(config: ConstraintConfig, q0: Quat, motor_axis: Vec3, motor_speed: f32) -> Self {
        Self {
            config,
            jacobian: MatMN::zero(),
            q0,
            motor_axis,
            motor_speed,
            baumgarte: Vec3::ZERO,
        }
    }
}

impl Constraint for ConstraintMotor {
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
        let motor_axis = body_a.orientation * self.motor_axis;
        let (motor_u, motor_v) = motor_axis.any_orthonormal_pair();

        let u = motor_u;
        let v = motor_v;
        let w = motor_axis;

        let p = Mat4::from_cols(Vec4::ZERO, Vec4::Y, Vec4::Z, Vec4::W);
        let p_t = p.transpose(); // pointless but self documenting

        let mat_a = p * quat_left(q1_inv) * quat_right(q2 * q0_inv) * p_t * -0.5;
        let mat_b = p * quat_left(q1_inv) * quat_right(q2 * q0_inv) * p_t * 0.5;

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

        // the quaternion jacobians
        const IDX: usize = 1;

        {
            let j1 = Vec3::ZERO;
            self.jacobian.rows[1][0] = j1.x;
            self.jacobian.rows[1][1] = j1.y;
            self.jacobian.rows[1][2] = j1.z;

            let tmp = mat_a * Vec4::new(0.0, u.x, u.y, u.z);
            let j2 = Vec3::new(tmp[IDX + 0], tmp[IDX + 1], tmp[IDX + 2]);
            self.jacobian.rows[1][3] = j2.x;
            self.jacobian.rows[1][4] = j2.y;
            self.jacobian.rows[1][5] = j2.z;

            let j3 = Vec3::ZERO;
            self.jacobian.rows[1][6] = j3.x;
            self.jacobian.rows[1][7] = j3.y;
            self.jacobian.rows[1][8] = j3.z;

            let tmp = mat_b * Vec4::new(0.0, u.x, u.y, u.z);
            let j4 = Vec3::new(tmp[IDX + 0], tmp[IDX + 1], tmp[IDX + 2]);
            self.jacobian.rows[1][9] = j4.x;
            self.jacobian.rows[1][10] = j4.y;
            self.jacobian.rows[1][11] = j4.z;
        }
        {
            let j1 = Vec3::ZERO;
            self.jacobian.rows[2][0] = j1.x;
            self.jacobian.rows[2][1] = j1.y;
            self.jacobian.rows[2][2] = j1.z;

            let tmp = mat_a * Vec4::new(0.0, v.x, v.y, v.z);
            let j2 = Vec3::new(tmp[IDX + 0], tmp[IDX + 1], tmp[IDX + 2]);
            self.jacobian.rows[2][3] = j2.x;
            self.jacobian.rows[2][4] = j2.y;
            self.jacobian.rows[2][5] = j2.z;

            let j3 = Vec3::ZERO;
            self.jacobian.rows[2][6] = j3.x;
            self.jacobian.rows[2][7] = j3.y;
            self.jacobian.rows[2][8] = j3.z;

            let tmp = mat_b * Vec4::new(0.0, v.x, v.y, v.z);
            let j4 = Vec3::new(tmp[IDX + 0], tmp[IDX + 1], tmp[IDX + 2]);
            self.jacobian.rows[2][9] = j4.x;
            self.jacobian.rows[2][10] = j4.y;
            self.jacobian.rows[2][11] = j4.z;
        }
        {
            let j1 = Vec3::ZERO;
            self.jacobian.rows[3][0] = j1.x;
            self.jacobian.rows[3][1] = j1.y;
            self.jacobian.rows[3][2] = j1.z;

            let tmp = mat_a * Vec4::new(0.0, w.x, w.y, w.z);
            let j2 = Vec3::new(tmp[IDX + 0], tmp[IDX + 1], tmp[IDX + 2]);
            self.jacobian.rows[3][3] = j2.x;
            self.jacobian.rows[3][4] = j2.y;
            self.jacobian.rows[3][5] = j2.z;

            let j3 = Vec3::ZERO;
            self.jacobian.rows[3][6] = j3.x;
            self.jacobian.rows[3][7] = j3.y;
            self.jacobian.rows[3][8] = j3.z;

            let tmp = mat_b * Vec4::new(0.0, w.x, w.y, w.z);
            let j4 = Vec3::new(tmp[IDX + 0], tmp[IDX + 1], tmp[IDX + 2]);
            self.jacobian.rows[3][9] = j4.x;
            self.jacobian.rows[3][10] = j4.y;
            self.jacobian.rows[3][11] = j4.z;
        }

        // calculate the baumgarte stabilization
        let beta = 0.05;
        let c = r.dot(r);

        let qr = body_a.orientation.inverse() * body_b.orientation;
        let qr_a = qr * q0_inv; // relative orientation in body_a's space

        // get the world space axis for the relative rotation
        let axis_a = body_a.orientation * qr_a.xyz();

        self.baumgarte[0] = (beta / dt_sec) * c;
        self.baumgarte[1] = motor_u.dot(axis_a) * (beta / dt_sec);
        self.baumgarte[2] = motor_v.dot(axis_a) * (beta / dt_sec);
    }

    fn solve(&mut self, bodies: &mut BodyArena) {
        let body_a = bodies.get_body(self.config.handle_a);
        let motor_axis = body_a.orientation * self.motor_axis;

        let mut w_dt = VecN::zero();
        w_dt[3] = motor_axis[0] * -self.motor_speed;
        w_dt[4] = motor_axis[1] * -self.motor_speed;
        w_dt[5] = motor_axis[2] * -self.motor_speed;
        w_dt[9] = motor_axis[0] * self.motor_speed;
        w_dt[10] = motor_axis[1] * self.motor_speed;
        w_dt[11] = motor_axis[2] * self.motor_speed;

        let jacobian_transpose = self.jacobian.transpose();

        // build the system of equations
        let q_dt = self.config.get_velocities(bodies) - w_dt; // by subtracting by the desired velocity, the solver is tricked into applying the impulse to give us that velocity
        let inv_mass_matrix = self.config.get_inverse_mass_matrix(bodies);
        let j_w_jt = self.jacobian * inv_mass_matrix * jacobian_transpose;
        let mut rhs = self.jacobian * q_dt * -1.0;
        rhs[0] -= self.baumgarte[0];
        rhs[1] -= self.baumgarte[1];
        rhs[2] -= self.baumgarte[2];

        // solve for the lagrange multipliers
        let lambda_n = lcp_gauss_seidel(&MatN::from(j_w_jt), &rhs);

        // apply the impulses
        let impulses = jacobian_transpose * lambda_n;
        self.config.apply_impulses(bodies, impulses);
    }
}
