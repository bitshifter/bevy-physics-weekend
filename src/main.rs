mod render;
mod time_accumulator;

use bevy::prelude::*;
use bevy_flycam::PlayerPlugin;
use physics::scene::{BodyHandle, PhysicsScene};
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
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    physics_scene: Res<PhysicsScene>,
) {
    commands
        .spawn_bundle(LightBundle {
            transform: Transform::from_translation(Vec3::new(4.0, 8.0, 4.0)),
            ..Default::default()
        });

    for body_handle in physics_scene.handles().iter() {
        let body = physics_scene.get_body(body_handle);
        let color = physics_scene.get_color(body_handle);
        let color = Color::rgb(color.x, color.y, color.z);
        let mesh = meshes.add(render::create_mesh_from_shape(body.shape.borrow()));
        commands
            .spawn_bundle(PbrBundle {
                mesh,
                material: materials.add(color.into()),
                ..Default::default()
            })
            .insert(*body_handle);
    }
}

fn main() {
    App::build()
        .insert_resource(Msaa { samples: 4 })
        .insert_resource(PhysicsScene::new())
        .insert_resource(TimeAccumulator::new())
        .add_plugins(DefaultPlugins)
        .add_plugin(PlayerPlugin)
        .add_startup_system(setup_rendering.system())
        .add_system(physics_update_system.system())
        .add_system(copy_transforms_system.system())
        .run();
}
