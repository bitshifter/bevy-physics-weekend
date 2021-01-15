use bevy::prelude::*;

trait Mat3Ex {
    fn from_diagonal(diagonal: Vec3) -> Self;
}

impl Mat3Ex for Mat3 {
    fn from_diagonal(diagonal: Vec3) -> Self {
        Self::from_cols(
            Vec3::new(diagonal.x, 0.0, 0.0),
            Vec3::new(0.0, diagonal.y, 0.0),
            Vec3::new(0.0, 0.0, diagonal.z),
        )
    }
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
            Shape::Sphere { .. } => Vec3::zero(),
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
}

#[derive(Copy, Clone, Debug)]
struct Contact {
    world_point_a: Vec3,
    world_point_b: Vec3,
    // local_point_a: Vec3,
    // local_point_b: Vec3,
    normal: Vec3,

    // separation_dist: f32,
    // time_of_impact: f32,
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
    shape: Shape,
}

impl Default for Body {
    fn default() -> Self {
        Self {
            position: Vec3::zero(),
            orientation: Quat::identity(),
            linear_velocity: Vec3::zero(),
            angular_velocity: Vec3::zero(),
            inv_mass: 1.0,
            elasticity: 0.5,
            shape: Shape::default(),
        }
    }
}

impl Body {
    pub fn centre_of_mass_world(&self) -> Vec3 {
        let com = self.shape.centre_of_mass();
        self.position + self.orientation * com
    }

    pub fn centre_of_mass_local(&self) -> Vec3 {
        self.shape.centre_of_mass()
    }

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

    pub fn integrate(&mut self, delta_seconds: f32) {
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
        let inv_angle = angle.recip();
        let dq = if inv_angle.is_finite() {
            Quat::from_axis_angle(d_angle * inv_angle, angle)
        } else {
            Quat::identity()
        };
        self.orientation = (dq * self.orientation).normalize();

        // now get the new body position
        self.position = position_com + dq * com_to_position;
    }

    pub fn has_infinite_mass(&self) -> bool {
        return self.inv_mass == 0.0;
    }
}

#[derive(Copy, Clone, Debug)]
struct BodyHandle(u32);

struct PhysicsScene {
    bodies: Vec<Body>,
    colors: Vec<Color>,
    handles: Vec<BodyHandle>,
}

impl PhysicsScene {
    pub fn new() -> Self {
        let mut bodies = Vec::with_capacity(2);
        let mut colors = Vec::with_capacity(2);
        bodies.push(Body {
            position: Vec3::new(0.0, 10.0, 0.0),
            inv_mass: 1.0,
            elasticity: 0.5,
            shape: Shape::Sphere { radius: 1.0 },
            ..Default::default()
        });
        colors.push(Color::rgb(0.8, 0.7, 0.6));

        // ground body
        bodies.push(Body {
            position: Vec3::new(0.0, -1000.0, 0.0),
            inv_mass: 0.0,
            elasticity: 1.0,
            shape: Shape::Sphere { radius: 1000.0 },
            ..Default::default()
        });
        colors.push(Color::rgb(0.3, 0.5, 0.3));

        let handles = bodies
            .iter()
            .enumerate()
            .map(|(index, _body)| BodyHandle(index as u32))
            .collect();

        Self {
            bodies,
            colors,
            handles,
        }
    }

    fn intersect(&mut self, index_a: usize, index_b: usize) -> Option<Contact> {
        let (body_a, body_b) = self.get_body_pair_mut_from_indices(index_a, index_b);

        // skip body pairs with infinite mass
        if body_a.has_infinite_mass() && body_b.has_infinite_mass() {
            return None;
        }

        // let body_a = &self.bodies[index_a];
        // let body_b = &self.bodies[index_b];
        let shapes = (body_a.shape, body_b.shape);
        match shapes {
            (Shape::Sphere { radius: radius_a }, Shape::Sphere { radius: radius_b }) => {
                let ab = body_b.position - body_a.position;
                let radius_ab = radius_a + radius_b;
                let radius_ab_sq = radius_ab * radius_ab;
                let ab_len_sq = ab.length_squared();
                if ab_len_sq <= radius_ab_sq {
                    let normal = ab.normalize();
                    Some(Contact {
                        world_point_a: body_a.position + normal * radius_a,
                        world_point_b: body_b.position - normal * radius_b,
                        normal,
                        // local_point_a: Vec3::zero(),
                        // local_point_b: Vec3::zero(),
                        handle_a: BodyHandle(index_a as u32),
                        handle_b: BodyHandle(index_b as u32),
                        // body_a,
                        // body_b,
                    })
                } else {
                    None
                }
            } // _ => unreachable!(),
        }
    }

    fn resolve_contact(&mut self, contact: &Contact) {
        let (body_a, body_b) = self.get_body_pair_mut(&contact.handle_a, &contact.handle_b);

        let elasticity = body_a.elasticity * body_b.elasticity;

        let inv_inertia_world_a = body_a.inv_intertia_tensor_world();
        let inv_inertia_world_b = body_b.inv_intertia_tensor_world();

        let ra = contact.world_point_a - body_a.centre_of_mass_world();
        let rb = contact.world_point_b - body_b.centre_of_mass_world();

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
            -(1.0 + elasticity) * vab.dot(contact.normal) / (total_inv_mass + angular_factor);
        let vec_impulse_j = contact.normal * impulse_j;

        body_a.apply_impulse(contact.world_point_a, vec_impulse_j);
        body_b.apply_impulse(contact.world_point_b, -vec_impulse_j);

        // also move colliding objects to just outside of each other
        let rcp_total_inv_mass = 1.0 / (body_a.inv_mass + body_b.inv_mass);
        let t_a = body_a.inv_mass * rcp_total_inv_mass;
        let t_b = body_b.inv_mass * rcp_total_inv_mass;

        let ds = contact.world_point_b - contact.world_point_a;

        body_a.position += ds * t_a;
        body_b.position -= ds * t_b;
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

        // TODO: is there a more idomatic Rust way of doing this?
        for a in 0..self.bodies.len() {
            for b in (a + 1)..self.bodies.len() {
                if let Some(contact) = self.intersect(a, b) {
                    self.resolve_contact(&contact);
                }
            }
        }

        for body in &mut self.bodies {
            body.integrate(delta_seconds);
        }

        for (index, body) in self.bodies.iter().enumerate() {
            println!(
                "index: {} position: {} dt: {}",
                index, body.position, delta_seconds
            );
        }
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
        index_a: &BodyHandle,
        index_b: &BodyHandle,
    ) -> (&mut Body, &mut Body) {
        self.get_body_pair_mut_from_indices(index_a.0 as usize, index_b.0 as usize)
    }

    pub fn get_body(&self, handle: &BodyHandle) -> &Body {
        &self.bodies[handle.0 as usize]
    }

    pub fn get_body_mut(&mut self, handle: &BodyHandle) -> &mut Body {
        &mut self.bodies[handle.0 as usize]
    }

    pub fn handles(&self) -> &Vec<BodyHandle> {
        &self.handles
    }

    pub fn get_color(&self, handle: &BodyHandle) -> Color {
        self.colors[handle.0 as usize]
    }
}

// fn body_update(time: Res<Time>, mut body_query: Query<(&mut Body, &mut Transform)>, collide: Query<&Body>) {
//     let delta_seconds = f32::min(1.0, time.delta_seconds());

//     for (mut body, mut transform) in body_query.iter_mut() {
//         if body.inv_mass != 0.0 {
//             // gravity needs to be an impulse
//             // I = dp, F = dp/dt => dp = F * dt => I = F * dt
//             // F = mgs
//             let impulse_gravity =
//                 Vec3::new(0.0, -10.0, 0.0) * body.inv_mass.recip() * delta_seconds;
//             body.apply_impulse_linear(impulse_gravity);
//         }
//         // collision check - sub optimal
//         for other_body in collide.iter() {
//             if body.id != other_body.id {
//                 if body.intersect(other_body) {
//                 }
//             }
//         }
//         // position update
//         body.integrate(delta_seconds);
//         transform.translation = body.position;
//         transform.rotation = body.orientation;
//     }
// }

fn physics_update_system(time: Res<Time>, mut scene: ResMut<PhysicsScene>) {
    let delta_seconds = f32::min(1.0, time.delta_seconds());
    scene.update(delta_seconds);
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
            transform: Transform::from_translation(Vec3::new(-15.0, 5.0, 0.0))
                .looking_at(Vec3::zero(), Vec3::unit_y()),
            ..Default::default()
        });

    for body_handle in physics_scene.handles().iter() {
        let body = physics_scene.get_body(body_handle);
        let color = physics_scene.get_color(body_handle);
        let mesh = match body.shape {
            Shape::Sphere { radius } => {
                let subdivisions = usize::max(5, (radius / 10.0) as usize).min(10);
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
        .add_plugins(DefaultPlugins)
        .add_startup_system(setup_rendering.system())
        .add_system(physics_update_system.system())
        .add_system(copy_transforms_system.system())
        .run();
}
