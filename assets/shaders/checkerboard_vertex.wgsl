#define_import_path bevy_pbr::checkerboard_vertex

#import bevy_pbr::{
    mesh_functions,
    forward_io::Vertex,
    view_transformations::position_world_to_clip,
}

struct CheckerboardVertexOutput {
    @builtin(position) position: vec4<f32>,
    @location(1) world_normal: vec3<f32>,
    @location(8) local_position: vec3<f32>,
    @location(9) local_normal: vec3<f32>,
}

@vertex
fn vertex(vertex: Vertex) -> CheckerboardVertexOutput {
    var out: CheckerboardVertexOutput;

    let world_from_local = mesh_functions::get_world_from_local(vertex.instance_index);

    out.world_normal = mesh_functions::mesh_normal_local_to_world(
        vertex.normal,
        vertex.instance_index
    );

    let world_position = mesh_functions::mesh_position_local_to_world(world_from_local, vec4<f32>(vertex.position, 1.0));
    out.position = position_world_to_clip(world_position.xyz);

    out.local_position = vertex.position;
    out.local_normal = vertex.normal;

    return out;
}
