use bevy::{
    prelude::*,
    render::{
        mesh::{shape::Icosphere, Indices},
        pipeline::PrimitiveTopology,
    },
};
use physics::shapes::{build_convex_hull, Shape, ShapeConvex};

fn create_mesh_from_convex_shape(convex_shape: &ShapeConvex) -> Mesh {
    // Build the connected convex hull from the points
    let mut hull_pts = Vec::new();
    let mut hull_tris = Vec::new();
    build_convex_hull(convex_shape.points(), &mut hull_pts, &mut hull_tris);

    // calculate smoothed normals
    // TODO: Could use map?
    let mut normals: Vec<[f32; 3]> = Vec::with_capacity(hull_pts.len());
    for i in 0..(hull_pts.len() as u32) {
        // TODO: Could use sum?
        let mut n = Vec3::ZERO;
        for tri in &hull_tris {
            if i != tri.a && i != tri.b && i != tri.c {
                continue;
            }

            let a = hull_pts[tri.a as usize];
            let b = hull_pts[tri.b as usize];
            let c = hull_pts[tri.c as usize];

            let ab = b - a;
            let ac = c - a;
            n += ab.cross(ac);
        }

        normals.push(n.normalize().into());
    }

    // TODO: Could add `From<&Vec3>` to help here or `to_array`
    let positions: Vec<[f32; 3]> = hull_pts.iter().map(|pt| (*pt).into()).collect();

    // TODO: Could use flat_map?
    let mut indices = Vec::with_capacity(hull_tris.len() * 3);
    for tri in &hull_tris {
        indices.push(tri.a);
        indices.push(tri.b);
        indices.push(tri.c);
    }

    let mut mesh = Mesh::new(PrimitiveTopology::TriangleList);
    mesh.set_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh.set_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    mesh.set_indices(Some(Indices::U32(indices)));

    // fake some UVs for the default shader
    let uvs: Vec<[f32; 2]> = std::iter::repeat([0.0; 2]).take(hull_pts.len()).collect();
    mesh.set_attribute(Mesh::ATTRIBUTE_UV_0, uvs);

    mesh
}

pub fn create_mesh_from_shape(shape: &Shape) -> Mesh {
    match shape {
        Shape::Sphere(sphere_shape) => {
            let radius = sphere_shape.radius;
            let subdivisions = (radius as usize).max(10).min(50);
            Mesh::from(Icosphere {
                radius,
                subdivisions,
            })
        }
        Shape::Box(box_shape) => {
            let bounds = box_shape.bounds;
            Mesh::from(shape::Box {
                min_x: bounds.mins.x,
                max_x: bounds.maxs.x,
                min_y: bounds.mins.y,
                max_y: bounds.maxs.y,
                min_z: bounds.mins.z,
                max_z: bounds.maxs.z,
            })
        }
        Shape::Convex(convex_shape) => create_mesh_from_convex_shape(convex_shape),
    }
}
