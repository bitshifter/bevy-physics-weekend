use crate::{
    body::Body,
    broadphase::broadphase,
    constraints::{ConstraintArena, ConstraintConfig},
    intersect::intersect_dynamic,
    scene_shapes::*,
};
use glam::{Quat, Vec3};

fn add_standard_sandbox(bodies: &mut BodyArena) {
    let wall_color = Vec3::splat(0.5);

    let box_ground = make_box_ground();
    let box_wall0 = make_box_wall0();
    let box_wall1 = make_box_wall1();

    bodies.add_with_color(
        Body {
            position: Vec3::ZERO,
            orientation: Quat::IDENTITY,
            linear_velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
            inv_mass: 0.0,
            elasticity: 0.5,
            friction: 0.5,
            shape: box_ground,
        },
        Vec3::new(0.3, 0.5, 0.3),
    );

    bodies.add_with_color(
        Body {
            position: Vec3::new(50.0, 0.0, 0.0),
            orientation: Quat::IDENTITY,
            linear_velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
            inv_mass: 0.0,
            elasticity: 0.5,
            friction: 0.0,
            shape: box_wall0.clone(),
        },
        wall_color,
    );

    bodies.add_with_color(
        Body {
            position: Vec3::new(-50.0, 0.0, 0.0),
            orientation: Quat::IDENTITY,
            linear_velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
            inv_mass: 0.0,
            elasticity: 0.5,
            friction: 0.0,
            shape: box_wall0,
        },
        wall_color,
    );

    bodies.add_with_color(
        Body {
            position: Vec3::new(0.0, 0.0, 25.0),
            orientation: Quat::IDENTITY,
            linear_velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
            inv_mass: 0.0,
            elasticity: 0.5,
            friction: 0.0,
            shape: box_wall1.clone(),
        },
        wall_color,
    );

    bodies.add_with_color(
        Body {
            position: Vec3::new(0.0, 0.0, -25.0),
            orientation: Quat::IDENTITY,
            linear_velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
            inv_mass: 0.0,
            elasticity: 0.5,
            friction: 0.0,
            shape: box_wall1,
        },
        wall_color,
    );
}

#[derive(Copy, Clone, Debug)]
pub struct Contact {
    pub world_point_a: Vec3,
    pub world_point_b: Vec3,
    pub local_point_a: Vec3,
    pub local_point_b: Vec3,
    pub normal: Vec3,

    pub separation_dist: f32,
    pub time_of_impact: f32,

    pub handle_a: BodyHandle,
    pub handle_b: BodyHandle,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct BodyHandle(pub u32);

#[derive(Debug)]
pub struct BodyArena {
    bodies: Vec<Body>,
    handles: Vec<BodyHandle>,
    colors: Vec<Vec3>,
}

impl BodyArena {
    pub fn new() -> Self {
        BodyArena {
            bodies: Vec::new(),
            handles: Vec::new(),
            colors: Vec::new(),
        }
    }

    pub fn add(&mut self, body: Body) -> BodyHandle {
        self.add_with_color(body, Vec3::new(0.3, 0.5, 0.3))
    }

    pub fn add_with_color(&mut self, body: Body, rgb: Vec3) -> BodyHandle {
        let handle = BodyHandle(self.bodies.len() as u32);
        self.bodies.push(body);
        self.handles.push(handle);
        self.colors.push(rgb);
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
        self.colors.clear();
    }

    pub fn len(&self) -> usize {
        self.bodies.len()
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

    pub fn get_body_mut(&mut self, handle: BodyHandle) -> &mut Body {
        &mut self.bodies[handle.0 as usize]
    }

    pub fn get_body(&self, handle: BodyHandle) -> &Body {
        &self.bodies[handle.0 as usize]
    }

    pub fn get_body_with_color(&self, handle: BodyHandle) -> (&Body, Vec3) {
        (
            &self.bodies[handle.0 as usize],
            self.colors[handle.0 as usize],
        )
    }

    pub fn handles(&self) -> &Vec<BodyHandle> {
        &self.handles
    }

    pub fn get_color(&self, handle: BodyHandle) -> Vec3 {
        self.colors[handle.0 as usize]
    }
}

// impl<'a> IntoIterator for &'a BodyArena {
//     type Item = &'a Body;
//     type IntoIter = std::slice::Iter<'a, Body>;

//     fn into_iter(self) -> Self::IntoIter {
//         self.bodies.iter()
//     }
// }

// impl<'a> IntoIterator for &'a mut BodyArena {
//     type Item = &'a mut Body;
//     type IntoIter = std::slice::IterMut<'a, Body>;

//     fn into_iter(self) -> Self::IntoIter {
//         self.bodies.iter_mut()
//     }
// }

pub struct PhysicsScene {
    bodies: BodyArena,
    constraints: ConstraintArena,
    contacts: Option<Vec<Contact>>,
    step_num: u64,
    pub paused: bool,
}

impl PhysicsScene {
    pub fn new() -> Self {
        let mut scene = PhysicsScene {
            bodies: BodyArena::new(),
            constraints: ConstraintArena::new(),
            contacts: None,
            step_num: 0,
            paused: true,
        };
        scene.reset();
        scene
    }

    pub fn reset(&mut self) {
        self.step_num = 0;
        // let num_bodies = 6 * 6 + 3 * 3;
        self.bodies.clear();
        self.constraints.clear();

        /*
        let ball_shape = Shape::make_sphere(0.5);

        // dynamic bodies
        for x in 0..6 {
            let radius = 0.5;
            let xx = ((x as f32) - 1.0) * radius * 1.5;
            for z in 0..6 {
                let zz = ((z as f32) - 1.0) * radius * 1.5;
                self.bodies.push(Body {
                    position: Vec3::new(xx, 10.0, zz),
                    orientation: Quat::IDENTITY,
                    linear_velocity: Vec3::ZERO,
                    angular_velocity: Vec3::ZERO,
                    inv_mass: 1.0,
                    elasticity: 0.5,
                    friction: 0.5,
                    shape: ball_shape.clone(),
                });
                self.colors.push(Vec3::new(0.8, 0.7, 0.6));
                // break; // HACK
            }
            // break; // HACK
        }
        */

        let handle_a = self.bodies.add(Body {
            position: Vec3::new(0.0, 5.0, 0.0),
            orientation: Quat::IDENTITY,
            linear_velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
            inv_mass: 0.0,
            elasticity: 1.0,
            friction: 0.5,
            shape: make_box_small(),
        });

        let handle_b = self.bodies.add(Body {
            position: Vec3::new(1.0, 5.0, 0.0),
            orientation: Quat::IDENTITY,
            linear_velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
            inv_mass: 1.0,
            elasticity: 1.0,
            friction: 0.5,
            shape: make_box_small(),
        });

        let body_a = self.bodies.get_body(handle_a);
        let body_b = self.bodies.get_body(handle_b);
        let joint_world_space_anchor = body_a.position;

        let anchor_a = body_a.world_to_local(joint_world_space_anchor);
        let anchor_b = body_b.world_to_local(joint_world_space_anchor);

        self.constraints.add_distance_constraint(ConstraintConfig {
            handle_a,
            handle_b,
            anchor_a,
            axis_a: Vec3::ZERO,
            anchor_b,
            axis_b: Vec3::ZERO,
        });

        add_standard_sandbox(&mut self.bodies);

        let max_contacts = self.bodies.len() * self.bodies.len();
        self.contacts.replace(Vec::with_capacity(max_contacts));

        self.paused = true;
    }

    fn resolve_contact(&mut self, contact: &Contact) {
        let (body_a, body_b) = self
            .bodies
            .get_body_pair_mut(contact.handle_a, contact.handle_b);
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
        self.step_num += 1;

        for body in self.bodies.iter_mut() {
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
        let mut contacts = self.contacts.take().expect("Contacts storage missing!");
        contacts.clear();

        // narrowphase (perform actual collision detection)
        for pair in collision_pairs {
            let (body_a, body_b) = self.bodies.get_body_pair_mut(pair.a, pair.b);

            // skip body pairs with infinite mass
            if body_a.has_infinite_mass() && body_b.has_infinite_mass() {
                continue;
            }

            if let Some(contact) = intersect_dynamic(pair.a, body_a, pair.b, body_b, delta_seconds)
            {
                contacts.push(contact)
            }
        }

        // sort the times of impact from earliest to latest
        contacts.sort_unstable_by(|a, b| {
            // TODO: fix lint?
            #[allow(clippy::float_cmp)]
            if a.time_of_impact < b.time_of_impact {
                std::cmp::Ordering::Less
            } else if a.time_of_impact == b.time_of_impact {
                std::cmp::Ordering::Equal
            } else {
                std::cmp::Ordering::Greater
            }
        });

        // solve constraints
        self.constraints.pre_solve(&self.bodies, delta_seconds);

        self.constraints.solve(&mut self.bodies);

        self.constraints.post_solve();

        // apply ballistic impulses
        let mut accumulated_time = 0.0;
        for contact in &contacts {
            let contact_time = contact.time_of_impact - accumulated_time;

            // position update
            for body in self.bodies.iter_mut() {
                body.update(contact_time)
            }

            self.resolve_contact(contact);
            accumulated_time += contact_time;
        }

        // update positions for the rest of this frame's time
        let time_remaining = delta_seconds - accumulated_time;
        if time_remaining > 0.0 {
            for body in self.bodies.iter_mut() {
                body.update(time_remaining);
            }
        }

        for (index, body) in self.bodies.iter().enumerate() {
            if !body.has_infinite_mass() {
                println!(
                    "step: {} dt: {} index: {} pos: {} rot: {} lin: {} ang: {}",
                    self.step_num,
                    delta_seconds,
                    index,
                    body.position,
                    body.orientation,
                    body.linear_velocity,
                    body.angular_velocity
                );
            }
        }

        // move contacts ownership back to self to avoid re-allocating next update
        self.contacts.replace(contacts);
    }

    pub fn get_body(&self, handle: BodyHandle) -> &Body {
        self.bodies.get_body(handle)
    }

    pub fn get_body_with_color(&self, handle: BodyHandle) -> (&Body, Vec3) {
        self.bodies.get_body_with_color(handle)
    }

    pub fn iter_body_handles(&self) -> core::slice::Iter<BodyHandle> {
        self.bodies.handles().iter()
    }
}

impl Default for PhysicsScene {
    fn default() -> Self {
        Self::new()
    }
}
