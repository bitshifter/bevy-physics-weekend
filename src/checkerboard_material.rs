use bevy::prelude::*;
use bevy::reflect::TypePath;
use bevy::render::render_resource::AsBindGroup;
use bevy::shader::ShaderRef;

#[derive(Asset, AsBindGroup, TypePath, Debug, Clone)]
pub struct CheckerboardMaterial {
    #[uniform(0)]
    _dummy: f32,
}

impl CheckerboardMaterial {
    pub fn new() -> Self {
        Self { _dummy: 0.0 }
    }
}

impl Default for CheckerboardMaterial {
    fn default() -> Self {
        Self::new()
    }
}

impl Material for CheckerboardMaterial {
    fn vertex_shader() -> ShaderRef {
        "shaders/checkerboard_vertex.wgsl".into()
    }

    fn fragment_shader() -> ShaderRef {
        "shaders/checkerboard.wgsl".into()
    }
}
