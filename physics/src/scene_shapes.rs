use crate::shapes::{Shape, ShapeBox, ShapeConvex};
use glam::{const_vec3, Quat, Vec3};
use std::sync::Arc;

const W: f32 = 50.0;
const H: f32 = 25.0;

const BOX_GROUND: [Vec3; 8] = [
    const_vec3!([-W, 0.0, -H]),
    const_vec3!([W, 0.0, -H]),
    const_vec3!([-W, 0.0, H]),
    const_vec3!([W, 0.0, H]),
    const_vec3!([-W, -1.0, -H]),
    const_vec3!([W, -1.0, -H]),
    const_vec3!([-W, -1.0, H]),
    const_vec3!([W, -1.0, H]),
];

const BOX_WALL0: [Vec3; 8] = [
    const_vec3!([-1.0, 0.0, -H]),
    const_vec3!([1.0, 0.0, -H]),
    const_vec3!([-1.0, 0.0, H]),
    const_vec3!([1.0, 0.0, H]),
    const_vec3!([-1.0, 5.0, -H]),
    const_vec3!([1.0, 5.0, -H]),
    const_vec3!([-1.0, 5.0, H]),
    const_vec3!([1.0, 5.0, H]),
];

const BOX_WALL1: [Vec3; 8] = [
    const_vec3!([-W, 0.0, -1.0]),
    const_vec3!([W, 0.0, -1.0]),
    const_vec3!([-W, 0.0, 1.0]),
    const_vec3!([W, 0.0, 1.0]),
    const_vec3!([-W, 5.0, -1.0]),
    const_vec3!([W, 5.0, -1.0]),
    const_vec3!([-W, 5.0, 1.0]),
    const_vec3!([W, 5.0, 1.0]),
];

pub fn make_sphere(radius: f32) -> Shape {
    Shape::make_sphere(radius)
}

pub fn make_box_ground() -> Shape {
    Shape::make_box(Arc::new(ShapeBox::new(&BOX_GROUND)))
}

pub fn make_box_wall0() -> Shape {
    Shape::make_box(Arc::new(ShapeBox::new(&BOX_WALL0)))
}

pub fn make_box_wall1() -> Shape {
    Shape::make_box(Arc::new(ShapeBox::new(&BOX_WALL1)))
}

pub fn make_diamond() -> Shape {
    let mut diamond = [Vec3::ZERO; 7 * 8];
    let quat_half = Quat::from_rotation_y(2.0 * std::f32::consts::PI * 0.125 * 0.5);
    let mut pts = [Vec3::ZERO; 7];
    pts[0] = Vec3::new(0.1, -1.0, 0.0);
    pts[1] = Vec3::new(1.0, 0.0, 0.0);
    pts[2] = Vec3::new(1.0, 0.1, 0.0);
    pts[3] = Vec3::new(0.4, 0.4, 0.0);
    pts[4] = Vec3::new(0.8, 0.3, 0.0);
    pts[4] = quat_half * pts[4];
    pts[5] = quat_half * pts[1];
    pts[6] = quat_half * pts[2];

    let quat = Quat::from_rotation_y(2.0 * std::f32::consts::PI * 0.125);
    let mut idx = 0;
    for pt in &pts {
        diamond[idx] = *pt;
        idx += 1;
    }

    let mut quat_acc = Quat::IDENTITY;
    for _ in 1..8 {
        quat_acc *= quat;
        for pt in &pts {
            diamond[idx] = quat_acc * *pt;
            idx += 1;
        }
    }

    Shape::make_convex(Arc::new(ShapeConvex::new(&diamond)))
}
