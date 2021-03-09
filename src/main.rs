mod time_accumulator;

use bevy::{prelude::*, render::mesh::shape::Icosphere};
use bevy_flycam::PlayerPlugin;
use physics::{
    scene::{BodyHandle, PhysicsScene},
    shapes::Shape,
};
use std::borrow::Borrow;
use time_accumulator::TimeAccumulator;

fn physics_update_system(
    keys: Res<Input<KeyCode>>,
    time: Res<Time>,
    mut accum: ResMut<TimeAccumulator>,
    mut scene: ResMut<PhysicsScene>,
) {
    let delta = time.delta();
    accum.update(delta);

    // T pauses the sim
    if keys.just_released(KeyCode::T) {
        scene.paused = !scene.paused;
    }

    let num_steps = if scene.paused {
        // y substeps when paused
        if keys.just_released(KeyCode::Y) {
            1
        } else {
            0
        }
    } else {
        accum.num_steps()
    };

    // R resets the scene
    if keys.just_released(KeyCode::R) {
        scene.reset();
    }

    let step_secs = accum.step_secs();
    for _ in 0..num_steps {
        // the game physics weekend application is doing 2 sub steps
        for _ in 0..2 {
            scene.update(step_secs * 0.5);
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
            ..Default::default() // })
                                 // // camera
                                 // .spawn(Camera3dBundle {
                                 //     transform: Transform::from_translation(Vec3::new(-40.0, 25.0, 0.0))
                                 //         .looking_at(Vec3::ZERO, Vec3::Y),
                                 //     ..Default::default()
        });
    // .with(FlyCamera {
    //     sensitivity: 10.0,
    //     ..Default::default()});

    for body_handle in physics_scene.handles().iter() {
        let body = physics_scene.get_body(body_handle);
        let color = physics_scene.get_color(body_handle);
        let color = Color::rgb(color.x, color.y, color.z);
        let mesh = match body.shape.borrow() {
            Shape::Sphere(sphere) => {
                let radius = sphere.radius;
                let subdivisions = (radius as usize).max(10).min(50);
                meshes.add(Mesh::from(Icosphere {
                    radius,
                    subdivisions,
                }))
            }
            _ => unimplemented!(),
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
        .add_plugin(PlayerPlugin)
        .add_startup_system(setup_rendering.system())
        .add_system(physics_update_system.system())
        .add_system(copy_transforms_system.system())
        .run();
}
