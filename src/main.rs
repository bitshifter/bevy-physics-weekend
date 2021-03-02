use bevy::{prelude::*, utils::Duration};

#[derive(Copy, Clone, Debug, PartialEq)]
struct Bounds {
    mins: Vec3,
    maxs: Vec3,
}

impl Bounds {
    // fn new() -> Bounds {
    //     Bounds {
    //         mins: Vec3::splat(std::f32::MAX),
    //         maxs: Vec3::splat(-std::f32::MAX),
    //     }
    // }
    // fn clear(&mut self) {
    //     *self = Self::new();
    // }

    // fn does_intersect(&self, rhs: &Self) -> bool {
    //     if self.maxs.cmplt(rhs.mins).any() {
    //         false
    //     } else if rhs.maxs.cmplt(self.mins).any() {
    //         false
    //     } else {
    //         true
    //     }
    // }

    // fn expand_by_points(&mut self, points: &[Vec3]) {
    //     for point in points {
    //         self.expand_by_point(*point);
    //     }
    // }

    fn expand_by_point(&mut self, rhs: Vec3) {
        self.mins = Vec3::select(rhs.cmplt(self.mins), rhs, self.mins);
        self.maxs = Vec3::select(rhs.cmpgt(self.maxs), rhs, self.maxs);
    }

    // fn expand_by_bounds(&mut self, rhs: &Self) {
    //     self.expand_by_point(rhs.mins);
    //     self.expand_by_point(rhs.maxs);
    // }

    // fn width_x(&self) -> f32 {
    //     self.maxs.x - self.mins.x
    // }

    // fn width_y(&self) -> f32 {
    //     self.maxs.y - self.mins.y
    // }

    // fn width_z(&self) -> f32 {
    //     self.maxs.z - self.mins.z
    // }
}

#[derive(Copy, Clone, Debug)]
enum Shape {
    Sphere { radius: f32 },
}

impl Default for Shape {
    fn default() -> Shape {
        Shape::Sphere { radius: 1.0 }
    }
}

impl Shape {
    fn centre_of_mass(&self) -> Vec3 {
        match self {
            Shape::Sphere { .. } => Vec3::ZERO,
        }
    }
    fn inertia_tensor(&self) -> Mat3 {
        match self {
            Shape::Sphere { radius } => {
                let i = 2.0 * radius * radius / 5.0;
                Mat3::from_diagonal(Vec3::splat(i))
            }
        }
    }

    fn bounds(&self, translation: Vec3, _orientation: Quat) -> Bounds {
        match self {
            Shape::Sphere { radius } => Bounds {
                mins: Vec3::splat(-radius) + translation,
                maxs: Vec3::splat(*radius) + translation,
            },
        }
    }
}

fn ray_sphere_intersect(
    ray_start: Vec3,
    ray_dir: Vec3,
    sphere_centre: Vec3,
    sphere_radius: f32,
) -> Option<(f32, f32)> {
    let m = sphere_centre - ray_start;
    let a = ray_dir.dot(ray_dir);
    let b = m.dot(ray_dir);
    let c = m.dot(m) - sphere_radius * sphere_radius;

    let delta = b * b - a * c;

    if delta < 0.0 {
        None
    } else {
        let inv_a = 1.0 / a;
        let delta_root = delta.sqrt();
        let t1 = inv_a * (b - delta_root);
        let t2 = inv_a * (b + delta_root);
        Some((t1, t2))
    }
}

fn sphere_sphere_dynamic(
    radius_a: f32,
    radius_b: f32,
    pos_a: Vec3,
    pos_b: Vec3,
    vel_a: Vec3,
    vel_b: Vec3,
    dt: f32,
) -> Option<(Vec3, Vec3, f32)> {
    let relative_velocity = vel_a - vel_b;

    let start_pt_a = pos_a;
    let end_pt_a = pos_a + relative_velocity * dt;
    let ray_dir = end_pt_a - start_pt_a;

    let mut t0 = 0.0;
    let mut t1 = 0.0;

    const EPSILON: f32 = 0.001;
    const EPSILON_SQ: f32 = EPSILON * EPSILON;
    if ray_dir.length_squared() < EPSILON_SQ {
        // ray is too short, just check if intersecting
        let ab = pos_b - pos_a;
        let radius = radius_a + radius_b + EPSILON;
        if ab.length_squared() > radius * radius {
            return None;
        }
    } else if let Some(toi) = ray_sphere_intersect(pos_a, ray_dir, pos_b, radius_a + radius_b) {
        t0 = toi.0;
        t1 = toi.1;
    } else {
        return None;
    }

    // Change from [0,1] range to [0,dt] range
    t0 *= dt;
    t1 *= dt;

    // If the collision is only in the past, then there's not future collision this frame
    if t1 < 0.0 {
        return None;
    }

    // Get the earliest positive time of impact
    let toi = if t0 < 0.0 { 0.0 } else { t0 };

    // If the earliest collision is too far in the future, then there's no collision this frame
    if toi > dt {
        return None;
    }

    // get the points on the respective points of collision
    let new_pos_a = pos_a + vel_a * toi;
    let new_pos_b = pos_b + vel_b * toi;
    let ab = (new_pos_b - new_pos_a).normalize();

    let pt_on_a = new_pos_a + ab * radius_a;
    let pt_on_b = new_pos_b - ab * radius_b;

    Some((pt_on_a, pt_on_b, toi))
}

#[derive(Copy, Clone, Debug)]
struct Contact {
    world_point_a: Vec3,
    world_point_b: Vec3,
    local_point_a: Vec3,
    local_point_b: Vec3,
    normal: Vec3,

    separation_dist: f32,
    time_of_impact: f32,

    handle_a: BodyHandle,
    handle_b: BodyHandle,
}

#[derive(Copy, Clone, Debug)]
struct Body {
    position: Vec3,
    orientation: Quat,
    linear_velocity: Vec3,
    angular_velocity: Vec3,
    inv_mass: f32,
    elasticity: f32,
    friction: f32,
    shape: Shape,
}

impl Default for Body {
    fn default() -> Self {
        Self {
            position: Vec3::ZERO,
            orientation: Quat::IDENTITY,
            linear_velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
            inv_mass: 1.0,
            elasticity: 0.5,
            friction: 0.5,
            shape: Shape::default(),
        }
    }
}

impl Body {
    pub fn centre_of_mass_world(&self) -> Vec3 {
        let com = self.shape.centre_of_mass();
        self.position + self.orientation * com
    }

    // pub fn centre_of_mass_local(&self) -> Vec3 {
    //     self.shape.centre_of_mass()
    // }

    pub fn world_to_local(&self, world_point: Vec3) -> Vec3 {
        let tmp = world_point - self.centre_of_mass_world();
        let inv_orientation = self.orientation.conjugate();
        let body_space = inv_orientation * tmp;
        body_space
    }

    pub fn local_to_world(&self, body_point: Vec3) -> Vec3 {
        let world_point = self.centre_of_mass_world() + self.orientation * body_point;
        world_point
    }

    pub fn inv_intertia_tensor_world(&self) -> Mat3 {
        let inv_inertia_tensor = self.inv_intertia_tensor_local();
        let orientation = Mat3::from_quat(self.orientation);
        orientation * inv_inertia_tensor * orientation.transpose()
    }

    pub fn inv_intertia_tensor_local(&self) -> Mat3 {
        self.shape.inertia_tensor().inverse() * self.inv_mass
    }

    pub fn apply_impulse(&mut self, impulse_point: Vec3, impulse: Vec3) {
        if self.has_infinite_mass() {
            return;
        }

        // impulse_point is the world space location of the application of the impulse
        // impulse is the world space direction and magnitude of the impulse
        self.apply_impulse_linear(impulse);

        // applying impulses must produce torques through the centre of mass
        let position = self.centre_of_mass_world();

        let r = impulse_point - position;
        // this is in world space
        let dl = r.cross(impulse);

        self.apply_impulse_angular(dl);
    }

    pub fn apply_impulse_angular(&mut self, impulse: Vec3) {
        if self.has_infinite_mass() {
            return;
        }

        // L = I w = r x p
        // dL = I dw = r x J
        // => dw = I^-1 * (r x J)
        self.angular_velocity += self.inv_intertia_tensor_world() * impulse;

        // clamp angular_velocity - 30 rad/s is fast enough for us
        const MAX_ANGULAR_SPEED: f32 = 30.0;
        const MAX_ANGULAR_SPEED_SQ: f32 = MAX_ANGULAR_SPEED * MAX_ANGULAR_SPEED;
        if self.angular_velocity.length_squared() > MAX_ANGULAR_SPEED_SQ {
            self.angular_velocity = self.angular_velocity.normalize() * MAX_ANGULAR_SPEED;
        }
    }

    pub fn apply_impulse_linear(&mut self, impulse: Vec3) {
        if self.has_infinite_mass() {
            return;
        }

        // p = mv
        // dp = m dv = J
        // => dv = J / m
        self.linear_velocity += impulse * self.inv_mass;
    }

    pub fn update(&mut self, delta_seconds: f32) {
        self.position += self.linear_velocity * delta_seconds;

        // we have an angular velocity around the centre of mass, this needs to be converted to
        // relative body position. This way we can properly update the orientation of the model

        let position_com = self.centre_of_mass_world();
        let com_to_position = self.position - position_com;

        // total torque is equal to external applied torques + internal torque (precession)
        // T = T_external + omega x I * omega
        // T_external = 0 because it was applied in the collision response function
        // T = Ia = w x I * w
        // a = I^-1 (w x I * w)
        let orientation = Mat3::from_quat(self.orientation);
        let inertia_tensor = orientation * self.shape.inertia_tensor() * orientation.transpose();
        let alpha = inertia_tensor.inverse()
            * (self
                .angular_velocity
                .cross(inertia_tensor * self.angular_velocity));
        self.angular_velocity += alpha * delta_seconds;

        // update orientation
        let d_angle = self.angular_velocity * delta_seconds;
        let angle = d_angle.length();
        let dq = Quat::from_axis_angle(d_angle, angle);
        self.orientation = (dq * self.orientation).normalize();

        // now get the new body position
        self.position = position_com + dq * com_to_position;
    }

    pub fn has_infinite_mass(&self) -> bool {
        return self.inv_mass == 0.0;
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
struct BodyHandle(u32);

#[derive(Copy, Clone, Debug)]
struct PsuedoBody {
    handle: BodyHandle,
    value: f32,
    is_min: bool,
}

fn compare_sat(a: &PsuedoBody, b: &PsuedoBody) -> std::cmp::Ordering {
    if a.value < b.value {
        std::cmp::Ordering::Less
    } else if a.value > b.value {
        std::cmp::Ordering::Greater
    } else {
        std::cmp::Ordering::Equal
    }
}

fn sort_bodies_bounds(bodies: &[Body], dt_sec: f32) -> Vec<PsuedoBody> {
    // TODO: allocation on sort
    let mut sorted_bodies = Vec::with_capacity(bodies.len() * 2);

    let axis = Vec3::ONE.normalize();
    for (i, body) in bodies.iter().enumerate() {
        let mut bounds = body.shape.bounds(body.position, body.orientation);

        // expand the bounds by the linear velocity
        bounds.expand_by_point(bounds.mins + body.linear_velocity * dt_sec);
        bounds.expand_by_point(bounds.maxs + body.linear_velocity * dt_sec);

        const BOUNDS_EPS: f32 = 0.01;
        bounds.expand_by_point(bounds.mins - Vec3::splat(BOUNDS_EPS));
        bounds.expand_by_point(bounds.maxs + Vec3::splat(BOUNDS_EPS));

        sorted_bodies.push(PsuedoBody {
            handle: BodyHandle(i as u32),
            value: axis.dot(bounds.mins),
            is_min: true,
        });
        sorted_bodies.push(PsuedoBody {
            handle: BodyHandle(i as u32),
            value: axis.dot(bounds.maxs),
            is_min: false,
        });
    }

    sorted_bodies.sort_unstable_by(compare_sat);

    sorted_bodies
}

#[derive(Copy, Clone, Debug)]
struct CollisionPair {
    a: BodyHandle,
    b: BodyHandle,
}

impl PartialEq for CollisionPair {
    fn eq(&self, other: &Self) -> bool {
        (self.a == other.a && self.b == other.b) || (self.a == other.b && self.b == other.a)
    }
}

impl Eq for CollisionPair {}

fn build_pairs(sorted_bodies: &[PsuedoBody]) -> Vec<CollisionPair> {
    let mut collision_pairs = Vec::new();

    // Now that the bodies are sorted, build the collision pairs
    for i in 0..sorted_bodies.len() {
        let a = &sorted_bodies[i];
        if !a.is_min {
            continue;
        }

        for j in (i + 1)..sorted_bodies.len() {
            let b = &sorted_bodies[j];
            // if we've hit the end of the a element then we're done creating pairs with a
            if b.handle == a.handle {
                break;
            }

            if !b.is_min {
                continue;
            }

            collision_pairs.push(CollisionPair {
                a: a.handle,
                b: b.handle,
            });
        }
    }

    collision_pairs
}

fn sweep_and_prune_1d(bodies: &[Body], dt_sec: f32) -> Vec<CollisionPair> {
    let sorted_bodies = sort_bodies_bounds(bodies, dt_sec);
    build_pairs(&sorted_bodies)
}

fn broadphase(bodies: &[Body], dt_sec: f32) -> Vec<CollisionPair> {
    sweep_and_prune_1d(bodies, dt_sec)
}

struct PhysicsScene {
    bodies: Vec<Body>,
    colors: Vec<Color>,
    handles: Vec<BodyHandle>,
    contacts: Vec<Contact>,
}

impl PhysicsScene {
    pub fn new() -> Self {
        let mut bodies = Vec::with_capacity(6 * 6 + 3 * 3);
        let mut colors = Vec::with_capacity(bodies.capacity());

        // dynamic bodies
        for x in 0..6 {
            let radius = 0.5;
            let xx = ((x as f32) - 1.0) * radius * 1.5;
            for z in 0..6 {
                let zz = ((z as f32) - 1.0) * radius * 1.5;
                bodies.push(Body {
                    position: Vec3::new(xx, 10.0, zz),
                    orientation: Quat::IDENTITY,
                    linear_velocity: Vec3::ZERO,
                    angular_velocity: Vec3::ZERO,
                    inv_mass: 1.0,
                    elasticity: 0.5,
                    friction: 0.5,
                    shape: Shape::Sphere { radius },
                });
                colors.push(Color::rgb(0.8, 0.7, 0.6));
                // break; // HACK
            }
            // break; // HACK
        }

        // static floor
        for x in 0..3 {
            let radius = 80.0;
            let xx = ((x as f32) - 1.0) * radius * 0.25;
            for z in 0..3 {
                let zz = ((z as f32) - 1.0) * radius * 0.25;
                bodies.push(Body {
                    position: Vec3::new(xx, -radius, zz),
                    orientation: Quat::IDENTITY,
                    linear_velocity: Vec3::ZERO,
                    angular_velocity: Vec3::ZERO,
                    inv_mass: 0.0,
                    elasticity: 0.99,
                    friction: 0.5,
                    shape: Shape::Sphere { radius },
                });
                colors.push(Color::rgb(0.3, 0.5, 0.3));
            }
        }

        /*
        // dynamic body
        bodies.push(Body {
            position: Vec3::new(0.0, 10.0, 0.0),
            linear_velocity: Vec3::new(1.0, 0.0, 0.0),
            inv_mass: 1.0,
            elasticity: 0.0,
            friction: 0.5,
            shape: Shape::Sphere { radius: 1.0 },
            ..Default::default()
        });
        colors.push(Color::rgb(0.8, 0.7, 0.6));

        // ground body
        bodies.push(Body {
            position: Vec3::new(0.0, -1000.0, 0.0),
            inv_mass: 0.0,
            elasticity: 0.99,
            friction: 0.5,
            shape: Shape::Sphere { radius: 1000.0 },
            ..Default::default()
        });
        colors.push(Color::rgb(0.3, 0.5, 0.3));
        */

        let handles = bodies
            .iter()
            .enumerate()
            .map(|(index, _body)| BodyHandle(index as u32))
            .collect();

        let contacts = Vec::with_capacity(bodies.len() * bodies.len());

        Self {
            bodies,
            colors,
            handles,
            contacts,
        }
    }

    fn intersect(
        &mut self,
        handle_a: BodyHandle,
        handle_b: BodyHandle,
        delta_seconds: f32,
    ) -> Option<Contact> {
        let (body_a, body_b) = self.get_body_pair_mut(handle_a, handle_b);

        // skip body pairs with infinite mass
        if body_a.has_infinite_mass() && body_b.has_infinite_mass() {
            return None;
        }

        let shapes = (body_a.shape, body_b.shape);
        match shapes {
            (Shape::Sphere { radius: radius_a }, Shape::Sphere { radius: radius_b }) => {
                if let Some((world_point_a, world_point_b, time_of_impact)) = sphere_sphere_dynamic(
                    radius_a,
                    radius_b,
                    body_a.position,
                    body_b.position,
                    body_a.linear_velocity,
                    body_b.linear_velocity,
                    delta_seconds,
                ) {
                    // step bodies forward to get local space collision points
                    body_a.update(time_of_impact);
                    body_b.update(time_of_impact);

                    // convert world space contacts to local space
                    let local_point_a = body_a.world_to_local(world_point_a);
                    let local_point_b = body_b.world_to_local(world_point_b);

                    let normal = (body_a.position - body_b.position).normalize();

                    // unwind time step
                    body_a.update(-time_of_impact);
                    body_b.update(-time_of_impact);

                    // calculate the separation distance
                    let ab = body_a.position - body_b.position;
                    let separation_dist = ab.length() - (radius_a + radius_b);

                    Some(Contact {
                        world_point_a,
                        world_point_b,
                        local_point_a,
                        local_point_b,
                        normal,
                        separation_dist,
                        time_of_impact,
                        handle_a,
                        handle_b,
                    })
                } else {
                    None
                }
            }
        }
    }

    fn resolve_contact(&mut self, contact: &Contact) {
        let (body_a, body_b) = self.get_body_pair_mut(contact.handle_a, contact.handle_b);
        debug_assert!(!body_a.has_infinite_mass() || !body_b.has_infinite_mass());

        let point_on_a = body_a.local_to_world(contact.local_point_a);
        let point_on_b = body_b.local_to_world(contact.local_point_b);

        let elasticity = body_a.elasticity * body_b.elasticity;

        let inv_inertia_world_a = body_a.inv_intertia_tensor_world();
        let inv_inertia_world_b = body_b.inv_intertia_tensor_world();

        let ra = point_on_a - body_a.centre_of_mass_world();
        let rb = point_on_b - body_b.centre_of_mass_world();

        let angular_j_a = (inv_inertia_world_a * ra.cross(contact.normal)).cross(ra);
        let angular_j_b = (inv_inertia_world_b * rb.cross(contact.normal)).cross(rb);
        let angular_factor = (angular_j_a + angular_j_b).dot(contact.normal);

        // calculate the world space velocity of the motion and rotation
        let vel_a = body_a.linear_velocity + body_a.angular_velocity.cross(ra);
        let vel_b = body_b.linear_velocity + body_b.angular_velocity.cross(rb);

        // calculate the collision impulse
        let vab = vel_a - vel_b;
        let total_inv_mass = body_a.inv_mass + body_b.inv_mass;
        let impulse_j =
            (1.0 + elasticity) * vab.dot(contact.normal) / (total_inv_mass + angular_factor);
        let vec_impulse_j = contact.normal * impulse_j;

        body_a.apply_impulse(point_on_a, -vec_impulse_j);
        body_b.apply_impulse(point_on_b, vec_impulse_j);

        // calculate the impulse caused by friction
        let friction = body_a.friction * body_b.friction;

        // find the normal direction of the velocity with respect to the normal of the collision
        let vel_normal = contact.normal * contact.normal.dot(vab);

        // find the tangent direction of the velocity with respect to the normal of the collision
        let vel_tan = vab - vel_normal;

        // get the tangential velocities relative to the other body
        let rel_vel_tan = vel_tan.normalize_or_zero();

        let inertia_a = (inv_inertia_world_a * ra.cross(rel_vel_tan)).cross(ra);
        let inertia_b = (inv_inertia_world_b * rb.cross(rel_vel_tan)).cross(rb);
        let inv_inertia = (inertia_a + inertia_b).dot(rel_vel_tan);

        // calculate the tangential impulse for friction
        let reduced_mass = 1.0 / (total_inv_mass + inv_inertia);
        let impulse_friction = vel_tan * reduced_mass * friction;

        // apply kinetic friction
        body_a.apply_impulse(point_on_a, -impulse_friction);
        body_b.apply_impulse(point_on_b, impulse_friction);

        // also move colliding objects to just outside of each other (projection method)
        if contact.time_of_impact == 0.0 {
            let ds = point_on_b - point_on_a;

            let rcp_total_inv_mass = 1.0 / total_inv_mass;
            let t_a = body_a.inv_mass * rcp_total_inv_mass;
            let t_b = body_b.inv_mass * rcp_total_inv_mass;

            body_a.position += ds * t_a;
            body_b.position -= ds * t_b;
        }
    }

    pub fn update(&mut self, delta_seconds: f32) {
        for body in &mut self.bodies {
            if !body.has_infinite_mass() {
                // gravity needs to be an impulse
                // I = dp, F = dp/dt => dp = F * dt => I = F * dt
                // F = mgs
                let impulse_gravity =
                    Vec3::new(0.0, -10.0, 0.0) * body.inv_mass.recip() * delta_seconds;
                body.apply_impulse_linear(impulse_gravity);
            }
        }

        // broadphase
        let collision_pairs = broadphase(&self.bodies, delta_seconds);

        // move contacts ownership to local
        let mut contacts = Vec::new();
        std::mem::swap(&mut contacts, &mut self.contacts);
        contacts.clear();
        let max_contacts = self.bodies.len() * self.bodies.len();
        contacts.reserve(max_contacts);

        // narrowphase (perform actual collision detection)
        for pair in collision_pairs {
            if let Some(contact) = self.intersect(pair.a, pair.b, delta_seconds) {
                contacts.push(contact)
            }
        }

        // sort the times of impact from earliest to latest
        contacts.sort_unstable_by(|a, b| {
            if a.time_of_impact < b.time_of_impact {
                std::cmp::Ordering::Less
            } else if a.time_of_impact == b.time_of_impact {
                std::cmp::Ordering::Equal
            } else {
                std::cmp::Ordering::Greater
            }
        });

        let mut accumulated_time = 0.0;
        for contact in &contacts {
            let contact_time = contact.time_of_impact - accumulated_time;

            // position update
            for body in &mut self.bodies {
                body.update(contact_time)
            }

            self.resolve_contact(contact);
            accumulated_time += contact_time;
        }

        // update positions for the rest of this frame's time
        let time_remaining = delta_seconds - accumulated_time;
        for body in &mut self.bodies {
            body.update(time_remaining);
        }

        // for (index, body) in self.bodies.iter().enumerate() {
        //     if !body.has_infinite_mass() {
        //         println!(
        //             "index: {} position: {} linvel: {} angvel: {}",
        //             index, body.position, body.linear_velocity, body.angular_velocity
        //         );
        //     }
        // }

        // move contacts ownership back to self to avoid re-allocating next update
        std::mem::swap(&mut contacts, &mut self.contacts);
    }

    fn get_body_pair_mut_from_indices(
        &mut self,
        index_a: usize,
        index_b: usize,
    ) -> (&mut Body, &mut Body) {
        match index_a.cmp(&index_b) {
            std::cmp::Ordering::Less => {
                let mut iter = self.bodies.iter_mut();
                let body_a = iter.nth(index_a).unwrap();
                let body_b = iter.nth(index_b - index_a - 1).unwrap();
                (body_a, body_b)
            }
            std::cmp::Ordering::Greater => {
                let mut iter = self.bodies.iter_mut();
                let body_b = iter.nth(index_b).unwrap();
                let body_a = iter.nth(index_a - index_b - 1).unwrap();
                (body_a, body_b)
            }
            std::cmp::Ordering::Equal => {
                panic!("get_body_pair_mut called with the same index {}", index_a)
            }
        }
    }

    pub fn get_body_pair_mut(
        &mut self,
        index_a: BodyHandle,
        index_b: BodyHandle,
    ) -> (&mut Body, &mut Body) {
        self.get_body_pair_mut_from_indices(index_a.0 as usize, index_b.0 as usize)
    }

    pub fn get_body(&self, handle: &BodyHandle) -> &Body {
        &self.bodies[handle.0 as usize]
    }

    pub fn handles(&self) -> &Vec<BodyHandle> {
        &self.handles
    }

    pub fn get_color(&self, handle: &BodyHandle) -> Color {
        self.colors[handle.0 as usize]
    }
}

struct TimeAccumulator {
    accumulated_time: Duration,
    frame: u64,
}

impl TimeAccumulator {
    fn new() -> Self {
        TimeAccumulator {
            accumulated_time: Duration::from_nanos(0),
            frame: 0,
        }
    }
}

fn physics_update_system(
    time: Res<Time>,
    mut accum: ResMut<TimeAccumulator>,
    mut scene: ResMut<PhysicsScene>,
) {
    let delta = time.delta();
    accum.accumulated_time += delta;
    accum.frame += 1;
    let update_rate = Duration::from_secs(1) / 60;
    let mut num_steps = (accum.accumulated_time.as_nanos() / update_rate.as_nanos()) as u32;
    const MAX_STEPS: u32 = 4;
    if num_steps > MAX_STEPS {
        eprintln!(
            "capping physics steps {} from time {} accumulated {} at rate {}",
            num_steps,
            delta.as_secs_f64(),
            accum.accumulated_time.as_secs_f64(),
            update_rate.as_secs_f64(),
        );
        num_steps = MAX_STEPS;
        accum.accumulated_time = Duration::from_nanos(0);
    } else {
        accum.accumulated_time -= update_rate * num_steps;
        // eprintln!(
        //     "{}: delta: {} accum: {} steps: {} rate: {}",
        //     accum.frame,
        //     delta.as_secs_f64(),
        //     accum.accumulated_time.as_secs_f64(),
        //     num_steps,
        //     update_rate.as_secs_f64()
        // );
    }

    let delta_seconds = update_rate.as_secs_f32();
    for _ in 0..num_steps {
        // the game physics weekend application is doing 2 sub steps
        for _ in 0..2 {
             scene.update(delta_seconds * 0.5);
        }
    }
}

fn copy_transforms_system(
    physics_scene: Res<PhysicsScene>,
    mut query: Query<(&BodyHandle, &mut Transform)>,
) {
    for (body_handle, mut transform) in query.iter_mut() {
        let body = physics_scene.get_body(body_handle);
        transform.translation = body.position;
        transform.rotation = body.orientation;
    }
}

fn setup_rendering(
    commands: &mut Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    physics_scene: Res<PhysicsScene>,
) {
    commands
        // // plane
        // .spawn(PbrBundle {
        //     mesh: meshes.add(Mesh::from(shape::Plane { size: 100.0 })),
        //     material: materials.add(Color::rgb(0.3, 0.5, 0.3).into()),
        //     ..Default::default()
        // })
        // sphere
        // .spawn(PbrBundle {
        //     mesh: meshes.add(Mesh::from(shape::Icosphere {
        //         radius: 1.0,
        //         ..Default::default()
        //     })),
        //     material: materials.add(Color::rgb(0.8, 0.7, 0.6).into()),
        //     transform: Transform::from_translation(Vec3::new(0.0, 2.0, 0.0)),
        //     ..Default::default()
        // })
        // .with(Body {
        //     shape: Shape::Sphere { radius: 1.0 },
        //     position: Vec3::new(0.0, 2.0, 0.0),
        //     ..Default::default()
        // })
        // ground "sphere" that won't fall under influence of gravity
        // .spawn(PbrBundle {
        //     mesh: meshes.add(Mesh::from(shape::Icosphere {
        //         radius: 1000.0,
        //         ..Default::default()
        //     })),
        //     material: materials.add(Color::rgb(0.3, 0.5, 0.3).into()),
        //     ..Default::default()
        // })
        // .with(Body {
        //     shape: Shape::Sphere { radius: 1000.0 },
        //     position: Vec3::new(0.0, -1000.0, 0.0),
        //     inv_mass: 0.0,
        //     ..Default::default()
        // })
        // // light
        .spawn(LightBundle {
            transform: Transform::from_translation(Vec3::new(4.0, 8.0, 4.0)),
            ..Default::default()
        })
        // camera
        .spawn(Camera3dBundle {
            transform: Transform::from_translation(Vec3::new(-20.0, 7.0, 0.0))
                .looking_at(Vec3::ZERO, Vec3::Y),
            ..Default::default()
        });

    for body_handle in physics_scene.handles().iter() {
        let body = physics_scene.get_body(body_handle);
        let color = physics_scene.get_color(body_handle);
        let mesh = match body.shape {
            Shape::Sphere { radius } => {
                let subdivisions = (radius as usize).max(10).min(50);
                meshes.add(Mesh::from(shape::Icosphere {
                    radius,
                    subdivisions,
                }))
            }
        };
        commands
            .spawn(PbrBundle {
                mesh,
                material: materials.add(color.into()),
                ..Default::default()
            })
            .with(*body_handle);
    }
}

fn main() {
    App::build()
        .add_resource(Msaa { samples: 4 })
        .add_resource(PhysicsScene::new())
        .add_resource(TimeAccumulator::new())
        .add_plugins(DefaultPlugins)
        .add_startup_system(setup_rendering.system())
        .add_system(physics_update_system.system())
        .add_system(copy_transforms_system.system())
        .run();
}
