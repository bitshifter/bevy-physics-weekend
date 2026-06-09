use bevy::{asset::RenderAssetUsages, prelude::*};
use bevy_mesh::{Indices, PrimitiveTopology, VertexAttributeValues};
use physics::shapes::{build_convex_hull, Shape, ShapeConvex};

fn create_mesh_from_convex_shape(convex_shape: &ShapeConvex) -> Mesh {
    let mut hull_pts = Vec::new();
    let mut hull_tris = Vec::new();
    build_convex_hull(convex_shape.points(), &mut hull_pts, &mut hull_tris);

    let mut normals: Vec<[f32; 3]> = Vec::with_capacity(hull_pts.len());
    for i in 0..(hull_pts.len() as u32) {
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

        normals.push(n.normalize().to_array());
    }

    let positions: Vec<[f32; 3]> = hull_pts.iter().map(|pt| pt.to_array()).collect();

    let mut indices = Vec::with_capacity(hull_tris.len() * 3);
    for tri in &hull_tris {
        indices.push(tri.a);
        indices.push(tri.b);
        indices.push(tri.c);
    }

    let mut mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    );
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    mesh.insert_indices(Indices::U32(indices));

    let uvs: Vec<[f32; 2]> = std::iter::repeat_n([0.0; 2], hull_pts.len()).collect();
    mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, uvs);

    mesh
}

pub fn create_mesh_from_shape(shape: &Shape) -> Mesh {
    match shape {
        Shape::Sphere(sphere_shape) => {
            let radius = sphere_shape.radius;
            let subdivisions = (radius as usize).clamp(10, 50);
            Sphere::new(radius).mesh().ico(subdivisions as u32).unwrap()
        }
        Shape::Box(box_shape) => {
            let bounds = box_shape.bounds;
            let min = Vec3::new(bounds.mins.x, bounds.mins.y, bounds.mins.z);
            let max = Vec3::new(bounds.maxs.x, bounds.maxs.y, bounds.maxs.z);
            let size = max - min;
            let center = (min + max) / 2.0;
            let mut mesh = Mesh::from(Cuboid::from_size(size));
            // Offset vertices so the box sits at its absolute world-space position (Cuboid::from_size centers at origin)
            if let Some(VertexAttributeValues::Float32x3(ref mut positions)) =
                mesh.attribute_mut(Mesh::ATTRIBUTE_POSITION)
            {
                for pos in positions.iter_mut() {
                    pos[0] += center.x;
                    pos[1] += center.y;
                    pos[2] += center.z;
                }
            }
            mesh
        }
        Shape::Convex(convex_shape) => create_mesh_from_convex_shape(convex_shape),
    }
}
