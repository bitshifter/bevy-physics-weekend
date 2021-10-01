mod flycam;
#[cfg(not(feature = "pbd"))]
mod render;
mod shaders;
mod time_accumulator;

use bevy::{
    diagnostic::{Diagnostic, DiagnosticId, Diagnostics, FrameTimeDiagnosticsPlugin},
    prelude::*,
    render::{
        pipeline::{PipelineDescriptor, RenderPipeline},
        shader::ShaderStages,
    },
    utils::Instant,
};
use bevy_egui::{egui, EguiContext, EguiPlugin};
use flycam::{FlyCam, NoCameraPlayerPlugin};
#[cfg(feature = "pbd")]
use pbd::{BodyHandle, PhysicsScene};
#[cfg(not(feature = "pbd"))]
use physics::{body::BodyHandle, scene::PhysicsScene};
use time_accumulator::TimeAccumulator;

// #[cfg(target_arch = "wasm32")]
// use console_error_panic_hook;

const PHYSICS_UPDATE_TIME: DiagnosticId =
    DiagnosticId::from_u128(337040787172757619024841343456040760896);

// trait BodyAccessor {
//     fn position(&self) -> Vec3;
//     fn orientation(&self) -> Quat;
// }

fn setup_diagnostic_system(mut diagnostics: ResMut<Diagnostics>) {
    diagnostics
        .add(Diagnostic::new(PHYSICS_UPDATE_TIME, "physics_update_time", 20).with_suffix("s"));
}

fn physics_update_system(
    keys: Res<Input<KeyCode>>,
    time: Res<Time>,
    mut accum: ResMut<TimeAccumulator>,
    mut scene: ResMut<PhysicsScene>,
    mut diagnostics: ResMut<Diagnostics>,
) {
    let now = Instant::now();

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
        scene.update(step_secs);
    }

    let elapsed = Instant::now().duration_since(now);
    diagnostics.add_measurement(PHYSICS_UPDATE_TIME, elapsed.as_secs_f64());
}

fn copy_transforms_system(
    physics_scene: Res<PhysicsScene>,
    mut query: Query<(&BodyHandle, &mut Transform)>,
) {
    for (&body_handle, mut transform) in query.iter_mut() {
        let body = physics_scene.get_body(body_handle);
        transform.translation = body.position();
        transform.rotation = body.orientation();
    }
}

#[cfg(feature = "pbd")]
fn create_mesh_from_body(body: &pbd::Body) -> Mesh {
    match body.shape() {
        &pbd::Shape::Box { size } => {
            let extent = size * 0.5;
            Mesh::from(shape::Box {
                min_x: -extent.x,
                max_x: extent.x,
                min_y: -extent.y,
                max_y: extent.y,
                min_z: -extent.z,
                max_z: extent.z,
            })
        }
    }
}

#[cfg(not(feature = "pbd"))]
fn create_mesh_from_body(body: physics::Body) -> Mesh {
    use std::borrow::Borrow;
    render::create_mesh_from_shape(body.shape.borrow())
}

#[cfg(not(target_arch = "wasm32"))]
fn setup_rendering_native(
    mut commands: Commands,
    asset_server: ResMut<AssetServer>,
    mut pipelines: ResMut<Assets<PipelineDescriptor>>,
    shaders: ResMut<Assets<Shader>>,
    mut meshes: ResMut<Assets<Mesh>>,
    physics_scene: Res<PhysicsScene>,
) {
    let (vertex, fragment) = shaders::load_shaders(asset_server, shaders);

    // Create a new shader pipeline
    let pipeline_handle = pipelines.add(PipelineDescriptor::default_config(ShaderStages {
        vertex,
        fragment: Some(fragment),
    }));

    commands.spawn_bundle(PointLightBundle {
        transform: Transform::from_translation(Vec3::new(4.0, 8.0, 4.0)),
        ..Default::default()
    });

    for &body_handle in physics_scene.iter_body_handles() {
        let body = physics_scene.get_body(body_handle);
        let mesh = meshes.add(create_mesh_from_body(body));
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

#[cfg(target_arch = "wasm32")]
fn setup_rendering_wasm(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    physics_scene: Res<PhysicsScene>,
) {
    commands.spawn_bundle(PointLightBundle {
        transform: Transform::from_translation(Vec3::new(4.0, 8.0, 4.0)),
        ..Default::default()
    });

    for &body_handle in physics_scene.iter_body_handles() {
        let body = physics_scene.get_body(body_handle);
        let color = if body.has_infinite_mass() {
            Color::rgb(0.3, 0.5, 0.3)
        } else {
            Color::rgb(0.8, 0.7, 0.6)
        };
        let mesh = meshes.add(render::create_mesh_from_shape(body.shape.borrow()));
        commands
            .spawn_bundle(PbrBundle {
                mesh,
                material: materials.add(color.into()),
                ..Default::default()
            })
            .insert(body_handle);
    }
}

fn get_diagnostic_average(
    diagnostics: &Diagnostics,
    diagnostic_id: DiagnosticId,
    default: f64,
) -> f64 {
    if let Some(diagnostic) = diagnostics.get(diagnostic_id) {
        if let Some(avg) = diagnostic.average() {
            return avg;
        }
    }
    return default;
}

fn egui_window(time: Res<Time>, diagnostics: Res<Diagnostics>, egui_context: Res<EguiContext>) {
    egui::Window::new("Physics Demo").show(egui_context.ctx(), |ui| {
        let fps = get_diagnostic_average(&diagnostics, FrameTimeDiagnosticsPlugin::FPS, 0.0);
        let frame_time = get_diagnostic_average(
            &diagnostics,
            FrameTimeDiagnosticsPlugin::FRAME_TIME,
            time.delta_seconds_f64(),
        );

        let physics_time = get_diagnostic_average(&diagnostics, PHYSICS_UPDATE_TIME, 0.0);

        ui.label(format!("avg fps: {:.1}", fps));
        ui.label(format!("avg frame/ms: {:.3}", frame_time * 1000.0));
        ui.label(format!("avg physics/ms: {:.3}", physics_time * 1000.0));
    });
}

/// Spawns the `Camera3dBundle` to be controlled
fn setup_player(mut commands: Commands) {
    commands
        .spawn_bundle(PerspectiveCameraBundle {
            transform: Transform::from_xyz(0.0, 10.0, 20.0)
                .looking_at(Vec3::new(0.0, 5.0, 0.0), Vec3::Y),
            ..Default::default()
        })
        .insert(FlyCam);
}

fn main() {
    // #[cfg(target_arch = "wasm32")]
    // console_error_panic_hook::set_once();

    let mut app = App::new();
    app.insert_resource(Msaa { samples: 4 });
    app.insert_resource(PhysicsScene::new());
    app.insert_resource(TimeAccumulator::new());
    app.add_plugins(DefaultPlugins);
    app.add_plugin(FrameTimeDiagnosticsPlugin);
    app.add_plugin(NoCameraPlayerPlugin);
    app.add_plugin(EguiPlugin);
    #[cfg(target_arch = "wasm32")]
    app.add_plugin(bevy_webgl2::WebGL2Plugin);
    #[cfg(target_arch = "wasm32")]
    app.add_startup_system(setup_rendering_wasm.system());
    #[cfg(not(target_arch = "wasm32"))]
    app.add_startup_system(setup_rendering_native.system());
    app.add_startup_system(setup_diagnostic_system.system());
    app.add_startup_system(setup_player.system());
    app.add_system(physics_update_system.system());
    app.add_system(copy_transforms_system.system());
    app.add_system(egui_window.system());
    app.run();
}
