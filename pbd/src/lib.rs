use glam::{Vec3, Quat};

#[derive(Copy, Clone, Debug)]
struct Pose {
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

impl Pose {
    pub const IDENTITY: Self = Self { p: Vec3::ZERO, q: Quat::IDENTITY };
}

#[derive(Copy, Clone, Debug)]
struct Body {
    pose: Pose,
    prev_pose: Pose,
    orig_pose: Pose,
    vel: Vec3,
    omega: Vec3,
    inv_mass: f32,
    inv_inertia: Vec3,
}

impl Default for Body {
    fn default() -> Self {
        Self {
            pose: Pose::IDENTITY,
            prev_pose: Pose::IDENTITY,
            orig_pose: Pose::IDENTITY,
            vel: Vec3::ZERO,
            omega: Vec3::ZERO,
            inv_mass: 1.0,
            inv_inertia: Vec3::ONE,
        }
    }
}

impl Body {
    const MAX_ROTATION_PER_SUBSTEP: f32 = 0.5;

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
        // TODO: add AddAssign to glam Quat
        self.pose.q = (self.pose.q + dq * 0.5).normalize();
    }

    pub fn integrate(&mut self, dt: f32, gravity: Vec3) {
        self.prev_pose = self.pose;
        self.vel += gravity * dt;
        self.pose.p += self.vel * dt;
        self.apply_rotation(self.omega, dt);
    }

    pub fn update(&mut self, dt: f32) {
        let inv_dt = dt.recip();
        self.vel = (self.pose.p - self.prev_pose.p) * inv_dt;
        let dq = self.pose.q * self.prev_pose.q.inverse();
        self.omega = 2.0 * inv_dt * dq.xyz();
        if dq.w < 0.0 {
            self.omega = -omega;
        }
    }
}

#[derive(Copy, Clone, Debug)]
struct BodyHandle(u32);

struct BodyArena {
    bodies: Vec<Body>,
    handles: Vec<BodyHandle>,
}

impl BodyArena {
    pub fn new() -> Self {
        Self {
            bodies: Vec::new(),
            handles: Vec::new()
        }
    }

    pub fn add(&mut self, body: Body) -> BodyHandle {
        let handle = BodyHandle(self.bodies.len() as u32);
        self.bodies.push(body);
        self.handles.push(handle);
        handle
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
}

