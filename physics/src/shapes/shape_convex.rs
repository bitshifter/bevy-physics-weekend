use super::ShapeTrait;
use crate::bounds::Bounds;
use glam::{Mat3, Quat, Vec3};

fn find_point_furthest_in_dir(pts: &[Vec3], dir: Vec3) -> usize {
    let mut max_idx = 0;
    let mut max_dist = dir.dot(pts[0]);
    for i in 1..pts.len() {
        let dist = dir.dot(pts[i]);
        if dist > max_dist {
            max_dist = dist;
            max_idx = i;
        }
    }
    max_idx
}

fn distance_from_line(a: Vec3, b: Vec3, pt: Vec3) -> f32 {
    let ab = (b - a).normalize();
    let ray = pt - a;
    let projection = ab * ray.dot(ab); // project the ray onto ab
    let perpendicular = ray - projection;
    perpendicular.length()
}

fn find_point_furthest_from_line(pts: &[Vec3], a: Vec3, b: Vec3) -> Vec3 {
    // TODO: don't need the index, could track the point
    // TODO: ab is recalculated every time
    let mut max_idx = 0;
    let mut max_dist = distance_from_line(a, b, pts[0]);
    for i in 1..pts.len() {
        let dist = distance_from_line(a, b, pts[i]);
        if dist > max_dist {
            max_dist = dist;
            max_idx = i;
        }
    }
    pts[max_idx]
}

fn distance_from_triangle(a: Vec3, b: Vec3, c: Vec3, pt: Vec3) -> f32 {
    let ab = b - a;
    let ac = c - a;
    let normal = ab.cross(ac).normalize();

    let ray = pt - a;
    let dist = ray.dot(normal);
    dist
}

fn find_point_furthest_from_triangle(pts: &[Vec3], a: Vec3, b: Vec3, c: Vec3) -> Vec3 {
    let mut max_idx = 0;
    let mut max_dist = distance_from_triangle(a, b, c, pts[0]);
    for i in 1..pts.len() {
        // TODO: triangle normal is recalculated every iteration
        let dist = distance_from_triangle(a, b, c, pts[i]);
        if dist * dist > max_dist * max_dist {
            max_dist = dist;
            max_idx = i;
        }
    }
    pts[max_idx]
}

#[derive(Clone, Copy, Debug)]
struct Tri {
    a: u32,
    b: u32,
    c: u32,
}

fn build_tetrahedron(verts: &[Vec3]) -> (Vec<Vec3>, Vec<Tri>) {
    let mut point0 = verts[find_point_furthest_in_dir(verts, Vec3::X)];
    let mut point1 = verts[find_point_furthest_in_dir(verts, -point0)];
    let point2 = find_point_furthest_from_line(verts, point0, point1);
    let point3 = find_point_furthest_from_triangle(verts, point0, point1, point2);

    // this is important for making sure the ordering is CCW for all faces
    if distance_from_triangle(point0, point1, point2, point3) > 0.0 {
        std::mem::swap(&mut point0, &mut point1);
    }

    // build the tetrahedron
    let hull_pts = vec![point0, point1, point2, point3];

    let hull_tris = vec![
        Tri { a: 0, b: 1, c: 2 },
        Tri { a: 0, b: 2, c: 3 },
        Tri { a: 2, b: 1, c: 3 },
        Tri { a: 1, b: 0, c: 3 },
    ];

    (hull_pts, hull_tris)
}

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

    fn bounds(&self, pos: Vec3, orient: Quat) -> Bounds {
        // TODO: this is the same as the box version
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

    fn support(&self, _dir: Vec3, _pos: Vec3, _orient: Quat, _bias: f32) -> Vec3 {
        unimplemented!();
    }

    fn fastest_linear_speed(&self, angular_velocity: Vec3, dir: Vec3) -> f32 {
        // this is the same as the box version
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
