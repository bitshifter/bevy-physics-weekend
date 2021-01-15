use bevy::prelude::*;

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
}

#[derive(Copy, Clone, Debug)]
struct Body {
    position: Vec3,
    orientation: Quat,
    linear_velocity: Vec3,
    inv_mass: f32,
    shape: Shape,
}

impl Default for Body {
    fn default() -> Self {
        Self {
            position: Vec3::default(),
            orientation: Quat::default(),
            linear_velocity: Vec3::default(),
            inv_mass: 1.0,
            shape: Shape::default(),
        }
    }
}

impl Body {
    pub fn centre_of_mass_world_space(&self) -> Vec3 {
        let com = self.shape.centre_of_mass();
        self.position + self.orientation * com
    }
    pub fn centre_of_mass_model_space(&self) -> Vec3 {
        self.shape.centre_of_mass()
    }
    pub fn world_space_to_body_space(&self, world_point: Vec3) -> Vec3 {
        let tmp = world_point - self.centre_of_mass_world_space();
        let inv_orientation = self.orientation.conjugate();
        let body_space = inv_orientation * tmp;
        body_space
    }
    pub fn body_space_to_world_space(&self, body_point: Vec3) -> Vec3 {
        let world_point = self.centre_of_mass_world_space() + self.orientation * body_point;
        world_point
    }
    pub fn apply_impulse_linear(&mut self, impulse: Vec3) {
        if self.inv_mass != 0.0 {
            // p = mv
            // dp = m dv = J
            // => dv = J / m
            self.linear_velocity += impulse * self.inv_mass;
        }
    }
    pub fn integrate(&mut self, delta_seconds: f32) {
        self.position += self.linear_velocity * delta_seconds;
    }
    pub fn intersects(&self, other: &Self) -> bool {
        let shapes = (self.shape, other.shape);
        match shapes {
            (Shape::Sphere { radius: radius_a }, Shape::Sphere { radius: radius_b }) => {
                let ab = self.position - other.position;
                let radius_ab = radius_a + radius_b;
                let radius_ab_sq = radius_ab * radius_ab;
                let ab_len_sq = ab.length_squared();
                return ab_len_sq <= radius_ab_sq;
            } // _ => unreachable!(),
        }
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
            shape: Shape::Sphere { radius: 1.0 },
            ..Default::default()
        });
        colors.push(Color::rgb(0.8, 0.7, 0.6));

        // ground body
        bodies.push(Body {
            position: Vec3::new(0.0, -1000.0, 0.0),
            inv_mass: 0.0,
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

    pub fn update(&mut self, delta_seconds: f32) {
        for body in &mut self.bodies {
            if body.inv_mass != 0.0 {
                // gravity needs to be an impulse
                // I = dp, F = dp/dt => dp = F * dt => I = F * dt
                // F = mgs
                let impulse_gravity =
                    Vec3::new(0.0, -10.0, 0.0) * body.inv_mass.recip() * delta_seconds;
                body.apply_impulse_linear(impulse_gravity);
            }
        }

        // TODO: is there a more idomatic Rust way of doing this?
        for i in 0..self.bodies.len() {
            // let body_a = &mut self.bodies[i];
            for j in (i + 1)..self.bodies.len() {
                // let body_b = &mut self.bodies[j];
                if self.bodies[i].intersects(&self.bodies[j]) {
                    self.bodies[i].linear_velocity = Vec3::zero();
                    self.bodies[j].linear_velocity = Vec3::zero();
                }
            }
        }

        for body in &mut self.bodies {
            body.integrate(delta_seconds);
        }
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
            Shape::Sphere { radius } => meshes.add(Mesh::from(shape::Icosphere {
                radius,
                ..Default::default()
            })),
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
        // .add_plugin(HelloPlugin)
        .add_startup_system(setup_rendering.system())
        .add_system(physics_update_system.system())
        .add_system(copy_transforms_system.system())
        .run();
}
