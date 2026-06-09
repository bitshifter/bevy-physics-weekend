mod checkerboard_material;
mod render;
mod time_accumulator;

use std::borrow::Borrow;
use std::ops::{Deref, DerefMut};

use bevy::camera_controller::free_camera::{FreeCamera, FreeCameraPlugin};
use bevy::prelude::*;
use checkerboard_material::CheckerboardMaterial;
use physics::{body::BodyHandle, scene::PhysicsScene};
use time_accumulator::TimeAccumulator;

#[derive(Resource)]
struct PhysicsSceneResource(PhysicsScene);

impl Deref for PhysicsSceneResource {
    type Target = PhysicsScene;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for PhysicsSceneResource {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

#[derive(Component)]
struct BodyHandleComponent(BodyHandle);

fn physics_update_system(
    keys: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
    mut accum: ResMut<TimeAccumulator>,
    mut scene: ResMut<PhysicsSceneResource>,
) {
    if keys.just_released(KeyCode::KeyT) {
        scene.paused = !scene.paused;
    }

    let mut dilation_change = None;
    if keys.just_released(KeyCode::BracketLeft) {
        dilation_change = Some(accum.time_dilation() * 0.5);
    }

    if keys.just_released(KeyCode::BracketRight) {
        dilation_change = Some(accum.time_dilation() * 2.0);
    }

    if keys.just_released(KeyCode::Backslash) {
        dilation_change = Some(1.0);
    }

    if let Some(dilation) = dilation_change {
        accum.set_time_dilation(dilation);
        println!("time dilation: {}", dilation);
    }

    let delta = time.delta();
    accum.update(delta);

    let num_steps = if scene.paused {
        if keys.just_released(KeyCode::KeyY) {
            1
        } else {
            0
        }
    } else {
        accum.num_steps()
    };

    if keys.just_released(KeyCode::KeyR) {
        scene.reset();
    }

    let step_secs = accum.step_secs();
    for _ in 0..num_steps {
        for _ in 0..2 {
            scene.update(step_secs * 0.5);
        }
    }
}

fn copy_transforms_system(
    physics_scene: Res<PhysicsSceneResource>,
    mut query: Query<(&BodyHandleComponent, &mut Transform)>,
) {
    for (body_handle, mut transform) in query.iter_mut() {
        let body = physics_scene.get_body(body_handle.0);
        transform.translation = body.position;
        transform.rotation = body.orientation;
    }
}

fn setup_rendering(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<CheckerboardMaterial>>,
    physics_scene: Res<PhysicsSceneResource>,
) {
    commands.spawn((
        Camera3d::default(),
        FreeCamera {
            key_up: KeyCode::Space,
            key_down: KeyCode::ShiftLeft,
            key_run: KeyCode::ControlLeft,
            keyboard_key_toggle_cursor_grab: KeyCode::Escape,

            sensitivity: 0.5,
            walk_speed: 10.0,
            run_speed: 30.0,
            ..Default::default()
        },
        Transform::from_translation(Vec3::new(-10.0, 5.0, 10.0)).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.spawn((
        PointLight {
            intensity: 100_000.0,
            shadow_maps_enabled: true,
            ..Default::default()
        },
        Transform::from_translation(Vec3::new(4.0, 8.0, 4.0)),
    ));

    for &body_handle in physics_scene.iter_body_handles() {
        let body = physics_scene.get_body(body_handle);
        let mesh = meshes.add(render::create_mesh_from_shape(body.shape.borrow()));
        commands
            .spawn((
                Mesh3d(mesh),
                MeshMaterial3d(materials.add(CheckerboardMaterial::new())),
                Transform::default(),
                Visibility::default(),
            ))
            .insert(BodyHandleComponent(body_handle));
    }
}

fn main() {
    App::new()
        .insert_resource(PhysicsSceneResource(PhysicsScene::new()))
        .insert_resource(TimeAccumulator::new())
        .add_plugins(DefaultPlugins)
        .add_plugins(MaterialPlugin::<CheckerboardMaterial>::default())
        .add_plugins(FreeCameraPlugin)
        .add_systems(Startup, setup_rendering)
        .add_systems(Update, (physics_update_system, copy_transforms_system))
        .run();
}
