#![allow(dead_code)]

use core::f32::consts::PI;
use glam::{const_vec3, Quat, Vec3};

fn get_quat_axis0(q: Quat) -> Vec3 {
    let x2 = q.x * 2.0;
    let w2 = q.w * 2.0;
    Vec3::new(
        (q.w * w2) - 1.0 + q.x * x2,
        (q.z * w2) + q.y * x2,
        (-q.y * w2) + q.z * x2,
    )
}

fn get_quat_axis1(q: Quat) -> Vec3 {
    let y2 = q.y * 2.0;
    let w2 = q.w * 2.0;
    Vec3::new(
        (-q.z * w2) + q.x * y2,
        (q.w * w2) - 1.0 + q.y * y2,
        (q.x * w2) + q.z * y2,
    )
}

fn get_quat_axis2(q: Quat) -> Vec3 {
    let z2 = q.z * 2.0;
    let w2 = q.w * 2.0;
    Vec3::new(
        (q.y * w2) + q.x * z2,
        (-q.x * w2) + q.y * z2,
        (q.w * w2) - 1.0 + q.z * z2,
    )
}

#[derive(Copy, Clone, Debug)]
pub enum Shape {
    Box { size: Vec3 },
}

impl Shape {
    fn calc_inv_mass_and_intertia(&self, density: f32) -> (f32, Vec3) {
        match self {
            Shape::Box { size } => {
                let mut mass = size.x * size.y * size.z * density;
                let inv_mass = mass.recip();
                mass /= 12.0;
                let inv_inertia = Vec3::new(
                    1.0 / (size.y * size.y + size.z * size.z) / mass,
                    1.0 / (size.z * size.z + size.x * size.x) / mass,
                    1.0 / (size.x * size.x + size.y * size.y) / mass,
                );
                (inv_mass, inv_inertia)
            }
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub struct Pose {
    p: Vec3,
    q: Quat,
}

impl Default for Pose {
    fn default() -> Self {
        Self {
            p: Vec3::ZERO,
            q: Quat::IDENTITY,
        }
    }
}

// TODO: change transform methdods to return instead of mutating input
impl Pose {
    pub const IDENTITY: Self = Self {
        p: Vec3::ZERO,
        q: Quat::IDENTITY,
    };

    fn rotate(&self, v: &mut Vec3) {
        *v = self.q * *v;
    }

    fn inv_rotate(&self, v: &mut Vec3) {
        let inv_q = self.q.inverse();
        *v = inv_q * *v;
    }

    fn transform(&self, v: &mut Vec3) {
        *v = self.q * *v;
        *v += self.p;
    }

    fn inv_transform(&self, v: &mut Vec3) {
        *v = *v - self.p;
        self.inv_rotate(v);
    }

    fn transform_pose(&self, pose: &mut Pose) {
        pose.q = self.q * pose.q;
        self.rotate(&mut pose.p);
        pose.p += self.p;
    }
}

#[derive(Copy, Clone, Debug)]
pub struct Body {
    pose: Pose,
    prev_pose: Pose,
    orig_pose: Pose,
    vel: Vec3,
    omega: Vec3,
    inv_mass: f32,
    inv_inertia: Vec3,
    shape: Shape,
}

impl Default for Body {
    fn default() -> Self {
        let shape = Shape::Box { size: Vec3::ONE };
        let density = 1.0;
        let (inv_mass, inv_inertia) = shape.calc_inv_mass_and_intertia(density);
        Self {
            pose: Pose::IDENTITY,
            prev_pose: Pose::IDENTITY,
            orig_pose: Pose::IDENTITY,
            vel: Vec3::ZERO,
            omega: Vec3::ZERO,
            inv_mass,
            inv_inertia,
            shape,
        }
    }
}

impl Body {
    const MAX_ROTATION_PER_SUBSTEP: f32 = 0.5;

    pub fn new(pose: Pose, shape: Shape, density: f32) -> Self {
        let (inv_mass, inv_inertia) = shape.calc_inv_mass_and_intertia(density);
        Self {
            pose,
            prev_pose: pose,
            orig_pose: pose,
            inv_mass,
            inv_inertia,
            ..Self::default()
        }
    }

    #[inline]
    pub fn position(&self) -> Vec3 {
        self.pose.p
    }

    #[inline]
    pub fn orientation(&self) -> Quat {
        self.pose.q
    }

    #[inline]
    pub fn shape(&self) -> &Shape {
        &self.shape
    }

    fn apply_rotation(&mut self, rot: Vec3, mut scale: f32) {
        // safety clamping.
        // This happens very rarely of the solver wants to turn the body by more than 30 degrees in
        // the orders of milliseconds

        const MAX_PHI: f32 = 0.5;
        let phi = rot.length();
        if phi * scale > Body::MAX_ROTATION_PER_SUBSTEP {
            scale = Body::MAX_ROTATION_PER_SUBSTEP / phi;
        }

        let mut dq = Quat::from_xyzw(rot.x * scale, rot.y * scale, rot.z * scale, 0.0);
        dq *= self.pose.q;
        // TODO: add AddAssign to glam Quat?
        self.pose.q = (self.pose.q + dq * 0.5).normalize();
    }

    fn velocity_at(&self, pos: Vec3) -> Vec3 {
        let mut vel = pos - self.pose.p;
        vel = vel.cross(self.omega);
        self.vel - vel
    }

    fn inverse_mass(&self, normal: Vec3, pos: Option<&Vec3>) -> f32 {
        let mut n = if let Some(&pos) = pos {
            let n = pos - self.pose.p;
            n.cross(normal)
        } else {
            normal
        };
        self.pose.inv_rotate(&mut n);
        let w = n * n * self.inv_inertia;
        let mut w = w.x + w.y + w.z;
        if pos.is_some() {
            w += self.inv_mass;
        }
        w
    }

    fn apply_correction(&mut self, corr: Vec3, pos: Option<&Vec3>, velocity_level: bool) {
        let mut dq = if let Some(&pos) = pos {
            if velocity_level {
                self.vel += corr * self.inv_mass;
            } else {
                self.pose.p += corr * self.inv_mass;
            }
            let dq = pos - self.pose.p;
            dq.cross(corr)
        } else {
            corr
        };
        self.pose.inv_rotate(&mut dq);
        if velocity_level {
            self.omega += dq;
        } else {
            self.apply_rotation(dq, 1.0);
        }
    }

    fn integrate(&mut self, dt: f32, gravity: Vec3) {
        self.prev_pose = self.pose;
        self.vel += gravity * dt;
        self.pose.p += self.vel * dt;
        self.apply_rotation(self.omega, dt);
    }

    fn update(&mut self, dt: f32) {
        let inv_dt = dt.recip();
        self.vel = (self.pose.p - self.prev_pose.p) * inv_dt;
        let dq = self.pose.q * self.prev_pose.q.inverse();
        self.omega = 2.0 * inv_dt * dq.xyz();
        if dq.w < 0.0 {
            self.omega = -self.omega;
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct BodyHandle(u32);

impl Default for BodyHandle {
    fn default() -> Self {
        Self::INVALID
    }
}

impl BodyHandle {
    const INVALID: Self = Self(u32::MAX);
    fn is_valid(self) -> bool {
        self != Self::INVALID
    }
}

#[derive(Clone, Debug)]
struct BodyArena {
    bodies: Vec<Body>,
    handles: Vec<BodyHandle>,
}

impl Default for BodyArena {
    fn default() -> Self {
        Self::new()
    }
}

impl BodyArena {
    pub fn new() -> Self {
        Self {
            bodies: Vec::new(),
            handles: Vec::new(),
        }
    }

    pub fn add(&mut self, body: Body) -> BodyHandle {
        let handle = BodyHandle(self.bodies.len() as u32);
        self.bodies.push(body);
        self.handles.push(handle);
        handle
    }

    pub fn integrate(&mut self, dt: f32, gravity: Vec3) {
        for body in self.bodies.iter_mut() {
            body.integrate(dt, gravity);
        }
    }

    pub fn update(&mut self, dt: f32) {
        for body in self.bodies.iter_mut() {
            body.update(dt);
        }
    }

    pub fn iter(&self) -> core::slice::Iter<Body> {
        self.bodies.iter()
    }

    pub fn iter_mut(&mut self) -> core::slice::IterMut<Body> {
        self.bodies.iter_mut()
    }

    pub fn clear(&mut self) {
        self.bodies.clear();
        self.handles.clear();
    }

    pub fn len(&self) -> usize {
        self.bodies.len()
    }

    pub fn is_empty(&self) -> bool {
        self.bodies.is_empty()
    }

    pub fn get_body(&self, handle: BodyHandle) -> &Body {
        &self.bodies[handle.0 as usize]
    }

    pub fn get_body_mut(&mut self, handle: BodyHandle) -> &mut Body {
        &mut self.bodies[handle.0 as usize]
    }

    pub fn handles(&self) -> &Vec<BodyHandle> {
        &self.handles
    }

    fn apply_body_pair_correction(
        &mut self,
        body0: BodyHandle,
        body1: BodyHandle,
        corr: Vec3,
        compliance: f32,
        dt: f32,
        pos0: Option<&Vec3>,
        pos1: Option<&Vec3>,
        velocity_level: bool,
    ) {
        let c = corr.length();
        if c == 0.0 {
            return;
        }

        let mut normal = corr.normalize();

        let w0 = if body0.is_valid() {
            self.get_body_mut(body0).inverse_mass(normal, pos0)
        } else {
            0.0
        };
        let w1 = if body1.is_valid() {
            self.get_body_mut(body1).inverse_mass(normal, pos1)
        } else {
            0.0
        };

        let w = w0 + w1;
        if w == 0.0 {
            return;
        }

        let lambda = -c / (w + compliance / dt / dt);
        normal *= -lambda;

        if body0.is_valid() {
            self.get_body_mut(body0)
                .apply_correction(normal, pos0, velocity_level);
        }
        if body1.is_valid() {
            self.get_body_mut(body1)
                .apply_correction(normal, pos1, velocity_level);
        }
    }

    fn limit_angle(
        &mut self,
        body0: BodyHandle,
        body1: BodyHandle,
        n: Vec3,
        a: Vec3,
        b: Vec3,
        min_angle: f32,
        max_angle: f32,
        compliance: f32,
        dt: f32,
        max_corr: f32,
    ) {
        let c = a.cross(b);

        let mut phi = c.dot(n).asin();
        if a.dot(b) < 0.0 {
            phi = PI - phi;
        }

        if phi > PI {
            phi -= 2.0 * PI;
        }
        if phi < -PI {
            phi += 2.0 * PI;
        }

        if phi < min_angle || phi > max_angle {
            phi = phi.clamp(min_angle, max_angle);

            debug_assert!(n.is_normalized());
            let q = Quat::from_axis_angle(n, phi);

            let mut omega = q * a;
            omega = omega.cross(b);

            phi = omega.length();
            if phi > max_corr {
                omega *= max_corr / phi;
            }

            self.apply_body_pair_correction(body0, body1, omega, compliance, dt, None, None, false);
        }
    }
}

#[derive(Copy, Clone, Debug)]
struct JointData {
    body0: BodyHandle,
    body1: BodyHandle,
    localpose0: Pose,
    localpose1: Pose,
    globalpose0: Pose,
    globalpose1: Pose,
    compliance: f32,
    rot_damping: f32,
    pos_damping: f32,
    has_swing_limits: bool,
    min_swing_angle: f32,
    max_swing_angle: f32,
    swing_limits_compliance: f32,
    has_twist_limits: bool,
    min_twist_angle: f32,
    max_twist_angle: f32,
    twist_limit_compliance: f32,
}

impl JointData {
    fn update_global_poses(&mut self, bodies: &BodyArena) {
        self.globalpose0 = self.localpose0;
        self.globalpose1 = self.localpose1;
        if self.body0.is_valid() {
            bodies
                .get_body(self.body0)
                .pose
                .transform_pose(&mut self.globalpose0);
        }
        if self.body1.is_valid() {
            bodies
                .get_body(self.body1)
                .pose
                .transform_pose(&mut self.globalpose1);
        }
    }
}

#[derive(Copy, Clone, Debug)]
enum Joint {
    Spherical(JointData),
    // Hinge(JointData),
    // Fixed(JointData),
}

impl Joint {
    fn get_data(&self) -> &JointData {
        match self {
            Joint::Spherical(data) => data,
            // Joint::Hinge(data) => data,
            // Joint::Fixed(data) => data,
        }
    }

    fn get_data_mut(&mut self) -> &mut JointData {
        match self {
            Joint::Spherical(data) => data,
            // Joint::Hinge(data) => data,
            // Joint::Fixed(data) => data,
        }
    }

    fn update_global_poses(&mut self, bodies: &BodyArena) {
        self.get_data_mut().update_global_poses(bodies);
    }

    fn solve_pos(&mut self, dt: f32, bodies: &mut BodyArena) {
        // happens anyway
        // self.update_global_poses(bodies);

        // orientation

        match self {
            Joint::Spherical(joint) => {
                // swing limits
                if joint.has_swing_limits {
                    joint.update_global_poses(bodies);
                    let a0 = get_quat_axis0(joint.globalpose0.q);
                    let a1 = get_quat_axis0(joint.globalpose1.q);
                    let n = a0.cross(a1).normalize();
                    bodies.limit_angle(joint.body0, joint.body1, n, a0, a1, joint.min_swing_angle, joint.max_swing_angle, joint.swing_limits_compliance, dt, PI)
                }
                // twist limits
                if joint.has_twist_limits {
                    joint.update_global_poses(bodies);
                    let n0 = get_quat_axis0(joint.globalpose0.q);
                    let n1 = get_quat_axis0(joint.globalpose1.q);
                    let n = (n0 + n1).normalize();
                    let mut a0 = get_quat_axis1(joint.globalpose0.q);
                    a0 += n * -n.dot(a0);
                    a0 = a0.normalize();
                    let mut a1 = get_quat_axis1(joint.globalpose1.q);
                    a1 += n * -n.dot(a1);
                    a1 = a1.normalize();

                    // handle gimbal lock problem
                    let max_corr = if n0.dot(n1) > -0.5 {
                        2.0 * PI
                    } else {
                        1.0 * dt
                    };

                    bodies.limit_angle(joint.body0, joint.body1, n, a0, a1, joint.min_twist_angle, joint.max_twist_angle, joint.twist_limit_compliance, dt, max_corr);
                }
            }
            // Joint::Hinge(j) => {
            // },
            // Joint::Fixed(j) => {
            //     // let q = j.globalpose1.q * j.globalpose0.q.inverse();
            // },
        }

        // position

        // simple attachment

        let joint = self.get_data_mut();
        joint.update_global_poses(bodies);
        let corr = joint.globalpose1.p - joint.globalpose0.p;
        bodies.apply_body_pair_correction(
            joint.body0,
            joint.body1,
            corr,
            joint.compliance,
            dt,
            Some(&joint.globalpose0.p),
            Some(&joint.globalpose1.p),
            false,
        );
    }

    fn solve_vel(&mut self, dt: f32, bodies: &mut BodyArena) {
        // gauss-seidel lets us make damping unconditionally stable in a very simple way. We clamp
        // the correction for each constraint to the magnitude of the current velocity making sure
        // that we never subtract more than it actually is

        let joint = self.get_data_mut();
        if joint.rot_damping > 0.0 {
            let mut omega = Vec3::ZERO;
            if joint.body0.is_valid() {
                omega -= bodies.get_body(joint.body0).omega;
            }
            if joint.body1.is_valid() {
                omega += bodies.get_body(joint.body1).omega;
            }
            omega *= f32::min(1.0, joint.rot_damping * dt);
            bodies.apply_body_pair_correction(
                joint.body0,
                joint.body1,
                omega,
                0.0,
                dt,
                None,
                None,
                true,
            );
        }
        if joint.pos_damping > 0.0 {
            joint.update_global_poses(bodies);
            let mut vel = Vec3::ZERO;
            if joint.body0.is_valid() {
                vel -= bodies
                    .get_body(joint.body0)
                    .velocity_at(joint.globalpose0.p);
            }
            if joint.body1.is_valid() {
                vel += bodies
                    .get_body(joint.body1)
                    .velocity_at(joint.globalpose1.p);
            }
            vel += f32::min(1.0, joint.pos_damping * dt);
            bodies.apply_body_pair_correction(
                joint.body0,
                joint.body1,
                vel,
                0.0,
                dt,
                Some(&joint.globalpose0.p),
                Some(&joint.globalpose1.p),
                true,
            );
        }
    }
}

#[derive(Clone, Debug)]
struct JointArena {
    joints: Vec<Joint>,
}

impl JointArena {
    fn new() -> Self {
        Self { joints: Vec::new() }
    }

    fn solve_pos(&mut self, dt: f32, bodies: &mut BodyArena) {
        for joint in self.joints.iter_mut() {
            joint.solve_pos(dt, bodies);
        }
    }

    fn solve_vel(&mut self, dt: f32, bodies: &mut BodyArena) {
        for joint in self.joints.iter_mut() {
            joint.solve_vel(dt, bodies);
        }
    }
}

pub struct PhysicsScene {
    bodies: BodyArena,
    joints: JointArena,
    pub num_substeps: u32,
    pub paused: bool,
}

impl Default for PhysicsScene {
    fn default() -> Self {
        Self::new()
    }
}

impl PhysicsScene {
    pub fn new() -> Self {
        Self {
            bodies: BodyArena::new(),
            joints: JointArena::new(),
            num_substeps: 40,
            paused: true,
        }
    }

    pub fn reset(&mut self) {
        self.bodies.clear();
        let num_objects = 100;
        let objects_size = Vec3::new(0.02, 0.04, 0.02);
        let last_objects_size = Vec3::new(0.2, 0.04, 0.2);

        let pos = Vec3::new(
            0.0,
            (num_objects as f32 * objects_size.y + last_objects_size.y) * 1.4 + 0.2,
            0.0,
        );

        let mut pose = Pose::IDENTITY;
        let mut last_body = None;
        let mut joint_pose0 = Pose::default();
        let mut joint_pose1 = Pose::default();
        joint_pose0.q = Quat::from_axis_angle(Vec3::Z, 0.5 * core::f32::consts::PI);
        joint_pose1.q = Quat::from_axis_angle(Vec3::Z, 0.5 * core::f32::consts::PI);

        let mut last_size = objects_size;
        for i in 0..num_objects {
            let size = if i < num_objects - 1 {
                objects_size
            } else {
                last_objects_size
            };

            pose.p = Vec3::new(pos.x, pos.y - i as f32 * objects_size.y, pos.z);

            let box_body = Body::new(pose, Shape::Box { size }, 1.0);
            self.bodies.add(box_body);

            last_body = Some(box_body);
            last_size = size;
        }
    }

    pub fn update(&mut self, timestep: f32) {
        const GRAVITY: Vec3 = const_vec3!([0.0, -10.0, 0.0]);
        self.simulate(timestep, self.num_substeps, GRAVITY);
    }

    fn simulate(&mut self, timestep: f32, num_substeps: u32, gravity: Vec3) {
        let dt = timestep / num_substeps as f32;
        for _ in 0..num_substeps {
            self.bodies.integrate(dt, gravity);
            self.joints.solve_pos(dt, &mut self.bodies);
            self.bodies.update(dt);
            self.joints.solve_vel(dt, &mut self.bodies);
        }
    }

    pub fn get_body(&self, handle: BodyHandle) -> &Body {
        self.bodies.get_body(handle)
    }

    pub fn iter_body_handles(&self) -> core::slice::Iter<BodyHandle> {
        self.bodies.handles().iter()
    }
}
