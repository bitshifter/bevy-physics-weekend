#import bevy_pbr::forward_io::VertexOutput

fn get_color_from_position_and_normal(world_position: vec3<f32>, normal: vec3<f32>) -> vec3<f32> {
    let pi = 3.141519;
    let scaled_pos = world_position * pi * 2.0;
    let scaled_pos2 = world_position * pi * 2.0 / 10.0 + vec3<f32>(pi / 4.0);
    let s = cos(scaled_pos2.x) * cos(scaled_pos2.y) * cos(scaled_pos2.z);
    let t = cos(scaled_pos.x) * cos(scaled_pos.y) * cos(scaled_pos.z);

    var color_mul = vec3<f32>(0.5, 0.5, 1.0);
    if abs(normal.x) > abs(normal.y) && abs(normal.x) > abs(normal.z) {
        color_mul = vec3<f32>(1.0, 0.5, 0.5);
    } else if abs(normal.y) > abs(normal.x) && abs(normal.y) > abs(normal.z) {
        color_mul = vec3<f32>(0.5, 1.0, 0.5);
    }

    let t_val = ceil(t * 0.9);
    let s_val = (ceil(s * 0.9) + 3.0) * 0.25;
    let color_b = vec3<f32>(0.85, 0.85, 0.85);
    let color_a = vec3<f32>(1.0, 1.0, 1.0);
    let final_color = mix(color_a, color_b, t_val) * s_val;

    return color_mul * final_color;
}

@fragment
fn fragment(in: VertexOutput) -> @location(0) vec4<f32> {
    let dir_to_light = normalize(vec3<f32>(1.0, 1.0, 1.0));

    let dx = 0.25;
    let dy = 0.25;
    var color_mul = vec3<f32>(0.0);
    var y = 0.0;
    loop {
        if y >= 1.0 { break; }
        var x = 0.0;
        loop {
            if x >= 1.0 { break; }
            let sample_pos = in.world_position.xyz
                + dpdx(in.world_position).xyz * x
                + dpdy(in.world_position).xyz * y;
            color_mul += get_color_from_position_and_normal(sample_pos, in.world_normal) * dx * dy;
            x += dx;
        }
        y += dy;
    }

    let ambient = 0.5;
    let flux = clamp(dot(in.world_normal, dir_to_light), 0.0, 1.0 - ambient) + ambient;

    return vec4<f32>(color_mul * flux, 1.0);
}
