mod render;
mod time_accumulator;

use bevy::{
    prelude::*,
    render::{
        pipeline::{PipelineDescriptor, RenderPipeline},
        shader::{ShaderStage, ShaderStages},
    },
};

use bevy_flycam::PlayerPlugin;
use physics::{body::BodyHandle, scene::PhysicsScene};
use std::borrow::Borrow;
use time_accumulator::TimeAccumulator;

const VERTEX_SHADER: &str = r#"
#version 450
layout(location = 0) in vec3 Vertex_Position;
layout(location = 1) in vec3 Vertex_Normal;

layout(location = 0) out vec4 v_WorldNormal;
layout(location = 1) out vec4 v_ModelPos;
layout(location = 2) out vec3 v_ModelNormal;

layout(set = 0, binding = 0) uniform CameraViewProj {
    mat4 ViewProj;
};
layout(set = 1, binding = 0) uniform Transform {
    mat4 Model;
};
void main() {
    vec3 normal = 2.0 * (Vertex_Normal.xyz - vec3(0.5));
    v_ModelNormal = normal;
    v_ModelPos = vec4(Vertex_Position, 1.0);

    v_WorldNormal = Model * vec4(normal.xyz, 0.0);

    vec4 world_position = Model * vec4(Vertex_Position, 1.0);

    gl_Position = ViewProj * world_position;
}
"#;

const FRAGMENT_SHADER: &str = r#"
#version 450

layout(location = 0) in vec4 v_WorldNormal;
layout(location = 1) in vec4 v_ModelPos;
layout(location = 2) in vec3 v_ModelNormal;

layout(location = 0) out vec4 o_Target;

vec3 GetColorFromPositionAndNormal(in vec3 worldPosition, in vec3 normal) {
    const float pi = 3.141519;

    vec3 scaledPos = worldPosition.xyz * pi * 2.0;
    vec3 scaledPos2 = worldPosition.xyz * pi * 2.0 / 10.0 + vec3(pi / 4.0);
    float s = cos(scaledPos2.x) * cos(scaledPos2.y) * cos(scaledPos2.z);  // [-1,1] range
    float t = cos(scaledPos.x) * cos(scaledPos.y) * cos(scaledPos.z);     // [-1,1] range

    vec3 colorMultiplier = vec3(0.5, 0.5, 1.0);
    if (abs(normal.x) > abs(normal.y) && abs(normal.x) > abs(normal.z)) {
        colorMultiplier = vec3(1.0, 0.5, 0.5);
    } else if (abs(normal.y) > abs(normal.x) && abs(normal.y) > abs(normal.z)) {
        colorMultiplier = vec3(0.5, 1.0, 0.5);
    }

    t = ceil(t * 0.9);
    s = (ceil(s * 0.9) + 3.0) * 0.25;
    vec3 colorB = vec3(0.85, 0.85, 0.85);
    vec3 colorA = vec3(1.0, 1.0, 1.0);
    vec3 finalColor = mix(colorA, colorB, t) * s;

    return colorMultiplier * finalColor;
}

void main() {
    vec3 dirToLight = normalize(vec3(1.0, 1.0, 1.0));

    // This is better than before, but it still has Moore patterns
    float dx = 0.25;
    float dy = 0.25;
    vec3 colorMultiplier = vec3(0.0, 0.0, 0.0);
    for (float y = 0.0; y < 1.0; y += dy) {
        for (float x = 0.0; x < 1.0; x += dx) {
            vec4 samplePos = v_ModelPos + dFdx(v_ModelPos) * x + dFdy(v_ModelPos) * y;
            colorMultiplier += GetColorFromPositionAndNormal(samplePos.xyz, v_ModelNormal.xyz) * dx * dy;
        }
    }
    
    float ambient = 0.5;
    float flux = clamp(dot(v_WorldNormal.xyz, dirToLight.xyz), 0.0, 1.0 - ambient) + ambient;

    vec4 finalColor;
    finalColor.rgb = colorMultiplier.rgb * flux;
    finalColor.a = 1.0;

    o_Target = finalColor;
}
"#;

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
    mut pipelines: ResMut<Assets<PipelineDescriptor>>,
    mut shaders: ResMut<Assets<Shader>>,
    mut meshes: ResMut<Assets<Mesh>>,
    physics_scene: Res<PhysicsScene>,
) {
    // Create a new shader pipeline
    let pipeline_handle = pipelines.add(PipelineDescriptor::default_config(ShaderStages {
        vertex: shaders.add(Shader::from_glsl(ShaderStage::Vertex, VERTEX_SHADER)),
        fragment: Some(shaders.add(Shader::from_glsl(ShaderStage::Fragment, FRAGMENT_SHADER))),
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
