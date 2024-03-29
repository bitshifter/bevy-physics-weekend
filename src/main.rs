mod render;
mod time_accumulator;

use bevy::{
    prelude::*,
    render::{
        pipeline::{PipelineDescriptor, RenderPipeline},
        shader::ShaderStages,
    },
};

use bevy_flycam::PlayerPlugin;
use physics::{body::BodyHandle, scene::PhysicsScene};
use std::borrow::Borrow;
use time_accumulator::TimeAccumulator;

fn physics_update_system(
    keys: Res<Input<KeyCode>>,
    time: Res<Time>,
    mut accum: ResMut<TimeAccumulator>,
    mut scene: ResMut<PhysicsScene>,
) {
    // T pauses the sim
    if keys.just_released(KeyCode::T) {
        scene.paused = !scene.paused;
    }

    let mut dilation_change = None;
    if keys.just_released(KeyCode::LBracket) {
        dilation_change = Some(accum.time_dilation() * 0.5);
    }

    if keys.just_released(KeyCode::RBracket) {
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
    for (&body_handle, mut transform) in query.iter_mut() {
        let body = physics_scene.get_body(body_handle);
        transform.translation = body.position;
        transform.rotation = body.orientation;
    }
}

fn setup_rendering(
    mut commands: Commands,
    asset_server: ResMut<AssetServer>,
    mut pipelines: ResMut<Assets<PipelineDescriptor>>,
    mut meshes: ResMut<Assets<Mesh>>,
    physics_scene: Res<PhysicsScene>,
) {
    // watch for changes
    asset_server.watch_for_changes().unwrap();

    // Create a new shader pipeline
    let pipeline_handle = pipelines.add(PipelineDescriptor::default_config(ShaderStages {
        vertex: asset_server.load::<Shader, _>("shaders/checkerboard.vert"),
        fragment: Some(asset_server.load::<Shader, _>("shaders/checkerboard.frag")),
    }));

    commands.spawn_bundle(LightBundle {
        light: Light {
            fov: f32::to_radians(75.0),
            ..Light::default()
        },
        transform: Transform::from_translation(Vec3::new(4.0, 8.0, 4.0)),
        ..Default::default()
    });

    for &body_handle in physics_scene.iter_body_handles() {
        let body = physics_scene.get_body(body_handle);
        // let base_color = Color::rgb(color.x, color.y, color.z);
        let mesh = meshes.add(render::create_mesh_from_shape(body.shape.borrow()));
        commands
            .spawn_bundle(PbrBundle {
                mesh,
                render_pipelines: RenderPipelines::from_pipelines(vec![RenderPipeline::new(
                    pipeline_handle.clone(),
                )]),
                ..Default::default()
            })
            // .insert(material)
            .insert(body_handle);
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
