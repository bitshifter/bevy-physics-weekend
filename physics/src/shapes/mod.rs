mod shape_box;
mod shape_convex;
mod shape_sphere;

use crate::bounds::Bounds;
use glam::{Mat3, Quat, Vec3};
use std::{ops::Deref, sync::Arc};

pub use shape_box::ShapeBox;
pub use shape_convex::{build_convex_hull, Edge, ShapeConvex, Tri};
pub use shape_sphere::ShapeSphere;

fn find_support_point(points: &[Vec3], dir: Vec3, pos: Vec3, orient: Quat, bias: f32) -> Vec3 {
    // find the point in the furthest in direction
    let mut max_pt = (orient * points[0]) + pos;
    let mut max_dist = dir.dot(max_pt);
    for &pt in &points[1..] {
        let pt = (orient * pt) + pos;
        let dist = dir.dot(pt);
        if dist > max_dist {
            max_dist = dist;
            max_pt = pt;
        }
    }

    let norm = dir.normalize() * bias;

    max_pt + norm
}

trait ShapeTrait {
    fn centre_of_mass(&self) -> Vec3;
    fn inertia_tensor(&self) -> Mat3;
    fn local_bounds(&self) -> Bounds;
    fn bounds(&self, translation: Vec3, orientation: Quat) -> Bounds;
    fn support(&self, dir: Vec3, pos: Vec3, orient: Quat, bias: f32) -> Vec3;
    fn fastest_linear_speed(&self, angular_velocity: Vec3, dir: Vec3) -> f32;
}

#[derive(Clone, Debug)]
pub enum Shape {
    Sphere(ShapeSphere),
    Box(Arc<ShapeBox>),
    Convex(Arc<ShapeConvex>),
}

impl Default for Shape {
    fn default() -> Shape {
        Shape::Sphere(ShapeSphere { radius: 1.0 })
    }
}

impl Shape {
    #[inline]
    pub fn make_sphere(radius: f32) -> Self {
        Shape::Sphere(ShapeSphere { radius })
    }

    #[inline]
    pub fn make_box(data: Arc<ShapeBox>) -> Self {
        Shape::Box(data)
    }

    #[inline]
    pub fn make_convex(data: Arc<ShapeConvex>) -> Self {
        Shape::Convex(data)
    }

    #[inline(always)]
    fn shape_trait(&self) -> &dyn ShapeTrait {
        // TODO: check the overhead of this
        match self {
            Shape::Sphere(data) => data,
            Shape::Box(data) => data.deref(),
            Shape::Convex(data) => data.deref(),
        }
    }

    #[inline]
    pub fn centre_of_mass(&self) -> Vec3 {
        self.shape_trait().centre_of_mass()
    }

    #[inline]
    pub fn inertia_tensor(&self) -> Mat3 {
        self.shape_trait().inertia_tensor()
    }

    #[inline]
    pub fn local_bounds(&self) -> Bounds {
        self.shape_trait().local_bounds()
    }

    #[inline]
    pub fn bounds(&self, translation: Vec3, orientation: Quat) -> Bounds {
        self.shape_trait().bounds(translation, orientation)
    }

    #[inline]
    pub fn support(&self, dir: Vec3, pos: Vec3, orient: Quat, bias: f32) -> Vec3 {
        self.shape_trait().support(dir, pos, orient, bias)
    }

    #[inline]
    pub fn fastest_linear_speed(&self, angular_velocity: Vec3, dir: Vec3) -> f32 {
        self.shape_trait()
            .fastest_linear_speed(angular_velocity, dir)
    }
}
