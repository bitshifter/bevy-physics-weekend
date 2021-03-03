use super::ShapeTrait;
use crate::bounds::Bounds;
use glam::{Mat3, Quat, Vec3};

#[derive(Clone, Debug)]
pub struct ShapeConvex {
    points: Vec<Vec3>,
    bounds: Bounds,
    com: Vec3,
}

impl ShapeConvex {
    pub fn new(_points: &[Vec3]) -> Self {
        unimplemented!();
    }
}

impl ShapeTrait for ShapeConvex {
    fn centre_of_mass(&self) -> Vec3 {
        self.com
    }

    fn inertia_tensor(&self) -> Mat3 {
        unimplemented!();
    }

    fn local_bounds(&self) -> Bounds {
        self.bounds
    }

    fn bounds(&self, _pos: Vec3, _orient: Quat) -> Bounds {
        unimplemented!();
    }

    fn support(&self, _dir: Vec3, _pos: Vec3, _orient: Quat, _bias: f32) -> Vec3 {
        unimplemented!();
    }

    fn fastest_linear_speed(&self, _angular_velocity: Vec3, _dir: Vec3) -> f32 {
        unimplemented!();
    }
}
