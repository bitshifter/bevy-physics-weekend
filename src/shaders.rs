use bevy::prelude::*;

// #[cfg(not(target_arch = "wasm32"))]
// pub fn load_shaders(
//     asset_server: ResMut<AssetServer>,
//     mut _shaders: ResMut<Assets<Shader>>,
// ) -> (Handle<Shader>, Handle<Shader>) {
//     // watch for changes
//     asset_server.watch_for_changes().unwrap();

//     let vertex = asset_server.load::<Shader, _>("shaders/checkerboard.vert");
//     let fragment = asset_server.load::<Shader, _>("shaders/checkerboard.frag");

//     (vertex, fragment)
// }

// #[cfg(target_arch = "wasm32")]
pub fn load_shaders(
    _asset_server: ResMut<AssetServer>,
    mut shaders: ResMut<Assets<Shader>>,
) -> (Handle<Shader>, Handle<Shader>) {
    use bevy::render::shader::ShaderStage;

    const VERTEX_SRC: &str = include_str!("../assets/shaders/checkerboard.vert");
    const FRAGMENT_SRC: &str = include_str!("../assets/shaders/checkerboard.frag");

    let vertex = shaders.add(Shader::from_glsl(ShaderStage::Vertex, VERTEX_SRC));
    let fragment = shaders.add(Shader::from_glsl(ShaderStage::Fragment, FRAGMENT_SRC));

    (vertex, fragment)
}
