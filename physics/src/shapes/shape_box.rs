use super::{find_support_point, ShapeTrait};
use crate::bounds::Bounds;
use glam::{Mat3, Quat, Vec3};

#[derive(Clone, Debug)]
pub struct ShapeBox {
    pub points: [Vec3; 8],
    pub bounds: Bounds,
    pub center_of_mass: Vec3,
}

impl ShapeBox {
    pub fn new(points: &[Vec3]) -> Self {
        assert!(!points.is_empty());

        let mut bounds = Bounds::new();
        for &p in points {
            bounds.expand_by_point(p);
        }

        let points = [
            Vec3::new(bounds.mins.x, bounds.mins.y, bounds.mins.z),
            Vec3::new(bounds.maxs.x, bounds.mins.y, bounds.mins.z),
            Vec3::new(bounds.mins.x, bounds.maxs.y, bounds.mins.z),
            Vec3::new(bounds.mins.x, bounds.mins.y, bounds.maxs.z),
            Vec3::new(bounds.maxs.x, bounds.maxs.y, bounds.maxs.z),
            Vec3::new(bounds.mins.x, bounds.maxs.y, bounds.maxs.z),
            Vec3::new(bounds.maxs.x, bounds.mins.y, bounds.maxs.z),
            Vec3::new(bounds.maxs.x, bounds.maxs.y, bounds.mins.z),
        ];

        let center_of_mass = (bounds.maxs + bounds.mins) * 0.5;

        ShapeBox {
            points,
            bounds,
            center_of_mass,
        }
    }
}

impl ShapeTrait for ShapeBox {
    fn centre_of_mass(&self) -> Vec3 {
        self.center_of_mass
    }

    fn inertia_tensor(&self) -> Mat3 {
        // inertia tensor for box centered around zero
        let d = self.bounds.maxs - self.bounds.mins;
        let dd = d * d;
        let diagonal = Vec3::new(dd.y + dd.z, dd.x + dd.z, dd.x + dd.y) / 12.0;
        let tensor = Mat3::from_diagonal(diagonal);

        // now we need to use the parallel axis theorem to get the ineria tensor for a box that is
        // not centered around the origin

        let cm = (self.bounds.maxs + self.bounds.mins) * 0.5;

        // the displacement from the center of mass to the origin
        let r = -cm;
        let r2 = r.length_squared();

        let pat_tensor = Mat3::from_cols(
            Vec3::new(r2 - r.x * r.x, r.x * r.y, r.x * r.z),
            Vec3::new(r.y * r.x, r2 - r.y * r.y, r.y * r.z),
            Vec3::new(r.z * r.x, r.z * r.y, r2 - r.z * r.z),
        );

        // now we need to add the centre of mass tensor and the parallel axis theorem tensor
        // together
        tensor + pat_tensor
    }

    fn local_bounds(&self) -> Bounds {
        self.bounds
    }

    fn bounds(&self, pos: Vec3, orient: Quat) -> Bounds {
        let corners = [
            Vec3::new(self.bounds.mins.x, self.bounds.mins.y, self.bounds.mins.z),
            Vec3::new(self.bounds.mins.x, self.bounds.mins.y, self.bounds.maxs.z),
            Vec3::new(self.bounds.mins.x, self.bounds.maxs.y, self.bounds.mins.z),
            Vec3::new(self.bounds.maxs.x, self.bounds.mins.y, self.bounds.mins.z),
            Vec3::new(self.bounds.maxs.x, self.bounds.maxs.y, self.bounds.maxs.z),
            Vec3::new(self.bounds.maxs.x, self.bounds.maxs.y, self.bounds.mins.z),
            Vec3::new(self.bounds.maxs.x, self.bounds.mins.y, self.bounds.maxs.z),
            Vec3::new(self.bounds.mins.x, self.bounds.maxs.y, self.bounds.maxs.z),
        ];

        let mut bounds = Bounds::new();
        for pt in &corners {
            let pt = (orient * *pt) + pos;
            bounds.expand_by_point(pt);
        }

        bounds
    }

    fn support(&self, dir: Vec3, pos: Vec3, orient: Quat, bias: f32) -> Vec3 {
        find_support_point(&self.points, dir, pos, orient, bias)
    }

    fn fastest_linear_speed(&self, angular_velocity: Vec3, dir: Vec3) -> f32 {
        let mut max_speed = 0.0;
        for pt in &self.points {
            let r = *pt - self.center_of_mass;
            let linear_velocity = angular_velocity.cross(r);
            let speed = dir.dot(linear_velocity);
            if speed > max_speed {
                max_speed = speed;
            }
        }
        max_speed
    }
}
