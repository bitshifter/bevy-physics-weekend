mod shape_box;
mod shape_convex;
mod shape_sphere;

use crate::bounds::Bounds;
use glam::{Mat3, Quat, Vec3};
use std::{ops::Deref, sync::Arc};

pub use shape_box::ShapeBox;
pub use shape_convex::ShapeConvex;
pub use shape_sphere::ShapeSphere;

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
    pub fn make_sphere(radius: f32) -> Self {
        Shape::Sphere(ShapeSphere { radius })
    }

    pub fn make_box(data: Arc<ShapeBox>) -> Self {
        Shape::Box(data)
    }

    pub fn make_convex(data: Arc<ShapeConvex>) -> Self {
        Shape::Convex(data)
    }

    fn shape_trait(&self) -> &dyn ShapeTrait {
        // TODO: check the overhead of this
        match self {
            Shape::Sphere(data) => data,
            Shape::Box(data) => data.deref(),
            Shape::Convex(data) => data.deref(),
        }
    }

    pub fn centre_of_mass(&self) -> Vec3 {
        self.shape_trait().centre_of_mass()
    }

    pub fn inertia_tensor(&self) -> Mat3 {
        self.shape_trait().inertia_tensor()
    }

    pub fn local_bounds(&self) -> Bounds {
        self.shape_trait().local_bounds()
    }

    pub fn bounds(&self, translation: Vec3, orientation: Quat) -> Bounds {
        self.shape_trait().bounds(translation, orientation)
    }
}
