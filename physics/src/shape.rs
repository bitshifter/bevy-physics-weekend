use crate::bounds::Bounds;
use glam::{Mat3, Quat, Vec3};
use std::{ops::Deref, sync::Arc};

trait ShapeTrait {
    fn centre_of_mass(&self) -> Vec3;
    fn inertia_tensor(&self) -> Mat3;
    fn local_bounds(&self) -> Bounds;
    fn bounds(&self, translation: Vec3, orientation: Quat) -> Bounds;
    fn support(&self, dir: Vec3, pos: Vec3, orient: Quat, bias: f32) -> Vec3;
    fn fastest_linear_speed(&self, angular_velocity: Vec3, dir: Vec3) -> f32;
}

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

#[derive(Clone, Debug)]
pub struct ShapeBox {
    points: [Vec3; 8],
    bounds: Bounds,
    com: Vec3,
}

impl ShapeBox {
    pub fn new(points: &[Vec3]) -> Self {
        assert!(!points.is_empty());

        let mut bounds = Bounds::new();
        for p in points {
            bounds.expand_by_point(*p);
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

        let com = (bounds.maxs + bounds.mins) * 0.5;

        ShapeBox {
            points,
            bounds,
            com,
        }
    }
}

impl ShapeTrait for ShapeBox {
    fn centre_of_mass(&self) -> Vec3 {
        self.com
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
        // find the point in the furthest in direction
        let mut max_pt = (orient * self.points[0]) + pos;
        let mut max_dist = dir.dot(max_pt);
        for pt in &self.points[1..] {
            let pt = (orient * *pt) + pos;
            let dist = dir.dot(pt);
            if dist > max_dist {
                max_dist = dist;
                max_pt = pt;
            }
        }

        let norm = dir.normalize() * bias;

        max_pt + norm
    }

    fn fastest_linear_speed(&self, angular_velocity: Vec3, dir: Vec3) -> f32 {
        let mut max_speed = 0.0;
        for pt in &self.points {
            let r = *pt - self.com;
            let linear_velocity = angular_velocity.cross(r);
            let speed = dir.dot(linear_velocity);
            if speed > max_speed {
                max_speed = speed;
            }
        }
        max_speed
    }
}

#[derive(Clone, Debug)]
pub enum Shape {
    Sphere(ShapeSphere),
    Box(Arc<ShapeBox>),
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

    fn shape_trait(&self) -> &dyn ShapeTrait {
        // TODO: check the overhead of this
        match self {
            Shape::Sphere(data) => data,
            Shape::Box(data) => data.deref(),
        }
    }

    pub fn centre_of_mass(&self) -> Vec3 {
        self.shape_trait().centre_of_mass()
    }

    pub fn inertia_tensor(&self) -> Mat3 {
        self.shape_trait().inertia_tensor()
    }

    pub fn bounds(&self, translation: Vec3, orientation: Quat) -> Bounds {
        self.shape_trait().bounds(translation, orientation)
    }
}
