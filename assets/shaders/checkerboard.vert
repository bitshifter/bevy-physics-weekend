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
