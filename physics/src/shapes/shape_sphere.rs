use super::ShapeTrait;
use crate::bounds::Bounds;
use glam::{Mat3, Quat, Vec3};

#[derive(Copy, Clone, Debug)]
pub struct ShapeSphere {
    pub radius: f32,
}

impl ShapeTrait for ShapeSphere {
    fn centre_of_mass(&self) -> Vec3 {
        Vec3::ZERO
    }

    fn inertia_tensor(&self) -> Mat3 {
        let i = 2.0 * self.radius * self.radius / 5.0;
        Mat3::from_diagonal(Vec3::splat(i))
    }

    fn local_bounds(&self) -> Bounds {
        Bounds {
            mins: Vec3::splat(-self.radius),
            maxs: Vec3::splat(self.radius),
        }
    }

    fn bounds(&self, pos: Vec3, _: Quat) -> Bounds {
        Bounds {
            mins: Vec3::splat(-self.radius) + pos,
            maxs: Vec3::splat(self.radius) + pos,
        }
    }

    fn support(&self, dir: Vec3, pos: Vec3, _: Quat, bias: f32) -> Vec3 {
        pos + dir * (self.radius + bias)
    }

    fn fastest_linear_speed(&self, _angular_velocity: Vec3, _dir: Vec3) -> f32 {
        unimplemented!()
    }
}
