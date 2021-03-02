use crate::bounds::Bounds;
use bevy::math::{Mat3, Quat, Vec3};

#[derive(Copy, Clone, Debug)]
pub enum Shape {
    Sphere { radius: f32 },
}

impl Default for Shape {
    fn default() -> Shape {
        Shape::Sphere { radius: 1.0 }
    }
}

impl Shape {
    pub fn centre_of_mass(&self) -> Vec3 {
        match self {
            Shape::Sphere { .. } => Vec3::ZERO,
        }
    }

    pub fn inertia_tensor(&self) -> Mat3 {
        match self {
            Shape::Sphere { radius } => {
                let i = 2.0 * radius * radius / 5.0;
                Mat3::from_diagonal(Vec3::splat(i))
            }
        }
    }

    pub fn bounds(&self, translation: Vec3, _orientation: Quat) -> Bounds {
        match self {
            Shape::Sphere { radius } => Bounds {
                mins: Vec3::splat(-radius) + translation,
                maxs: Vec3::splat(*radius) + translation,
            },
        }
    }
}
