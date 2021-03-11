// TODO: remove
#![allow(dead_code)]

use super::ShapeTrait;
use crate::bounds::Bounds;
use glam::{Mat3, Quat, Vec3};

// TODO: There are a lot of C style loops that could be made idomatic in here.

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
    // TODO: don't need the index, could track the point
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

fn build_tetrahedron(verts: &[Vec3], hull_points: &mut Vec<Vec3>, hull_tris: &mut Vec<Tri>) {
    hull_points.clear();
    hull_tris.clear();

    let mut point0 = verts[find_point_furthest_in_dir(verts, Vec3::X)];
    let mut point1 = verts[find_point_furthest_in_dir(verts, -point0)];
    let point2 = find_point_furthest_from_line(verts, point0, point1);
    let point3 = find_point_furthest_from_triangle(verts, point0, point1, point2);

    // this is important for making sure the ordering is CCW for all faces
    if distance_from_triangle(point0, point1, point2, point3) > 0.0 {
        std::mem::swap(&mut point0, &mut point1);
    }

    // build the tetrahedron
    hull_points.extend_from_slice(&[point0, point1, point2, point3]);

    hull_tris.extend_from_slice(&[
        Tri { a: 0, b: 1, c: 2 },
        Tri { a: 0, b: 2, c: 3 },
        Tri { a: 2, b: 1, c: 3 },
        Tri { a: 1, b: 0, c: 3 },
    ]);
}

fn remove_internal_points(hull_points: &[Vec3], hull_tris: &[Tri], check_pts: &mut Vec<Vec3>) {
    // for i in 0..check_pts.len() {
    let mut i = 0;
    while i < check_pts.len() {
        let pt = check_pts[i];

        let mut is_external = false;
        // for t in 0..hull_tris.len() {
        //     let tri = hull_tris[t];
        for tri in hull_tris {
            let a = hull_points[tri.a as usize];
            let b = hull_points[tri.b as usize];
            let c = hull_points[tri.c as usize];

            // if the point is in front of any triangle then it's external
            let dist = distance_from_triangle(a, b, c, pt);
            if dist > 0.0 {
                is_external = true;
                break;
            }
        }

        // if it's not external, then it's inside the polyhedron and should be removed
        if !is_external {
            check_pts.remove(i);
            // i -= 1;
        } else {
            i += 1;
        }
    }

    // also remove any points that are just a little too close to the hull points
    // for i in 0..check_pts.len() {
    let mut i = 0;
    while i < check_pts.len() {
        let pt = check_pts[i];

        let mut is_too_close = false;
        // for j in 0..hull_points.len() {
        //     let hull_pt = hull_points[j];
        for hull_pt in hull_points {
            let ray = *hull_pt - pt;
            if ray.length_squared() < 0.01 * 0.01 {
                // 1cm is too close
                is_too_close = true;
                break;
            }
        }

        if is_too_close {
            check_pts.remove(i);
            // i -= 1;
        } else {
            i += 1;
        }
    }
}

#[derive(Copy, Clone, Debug)]
struct Edge {
    a: u32,
    b: u32,
}

impl PartialEq for Edge {
    fn eq(&self, other: &Self) -> bool {
        (self.a == other.a && self.b == other.b) || (self.a == other.b && self.b == other.a)
    }
}

impl Eq for Edge {}

// This will compare the incoming edge with all the edges in the facing tris and then return true
// if it's unique.
fn is_edge_unique(tris: &[Tri], facing_tris: &[u32], ignore_tri: u32, edge: &Edge) -> bool {
    for i in 0..facing_tris.len() {
        let tri_idx = facing_tris[i];
        if ignore_tri == tri_idx {
            continue;
        }

        let tri = tris[tri_idx as usize];

        let edges = [
            Edge { a: tri.a, b: tri.b },
            Edge { a: tri.b, b: tri.c },
            Edge { a: tri.c, b: tri.a },
        ];

        for e in 0..3 {
            if edge == &edges[e] {
                return false;
            }
        }
    }

    true
}

fn add_point(hull_points: &mut Vec<Vec3>, hull_tris: &mut Vec<Tri>, pt: Vec3) {
    // This point is outside
    // Now ew need to remove old triangles and build new ones

    // Find all the triangles that face this point
    let mut facing_tris = Vec::new();
    // TODO: hopefully this works the same as the C loop
    for i in (0..hull_tris.len()).rev() {
        let tri = hull_tris[i];
        let a = hull_points[tri.a as usize];
        let b = hull_points[tri.b as usize];
        let c = hull_points[tri.c as usize];

        let dist = distance_from_triangle(a, b, c, pt);
        if dist > 0.0 {
            facing_tris.push(i as u32);
        }
    }

    // Now find all edges that are unique to the tris, these will be the edges that form the new
    // trianges
    let mut unique_edges = Vec::new();
    for tri_idx in &facing_tris {
        let tri = hull_tris[*tri_idx as usize];

        let edges = [
            Edge { a: tri.a, b: tri.b },
            Edge { a: tri.b, b: tri.c },
            Edge { a: tri.c, b: tri.a },
        ];

        for edge in &edges {
            if is_edge_unique(hull_tris, &facing_tris, *tri_idx, edge) {
                unique_edges.push(*edge);
            }
        }
    }

    // now remove the old facing tris
    for tri_idx in &facing_tris {
        hull_tris.remove(*tri_idx as usize);
    }

    // now add the new point
    hull_points.push(pt);
    let new_pt_idx = hull_points.len() as u32 - 1;

    // now add triangles for each unique edge
    for edge in &unique_edges {
        let tri = Tri {
            a: edge.a,
            b: edge.b,
            c: new_pt_idx,
        };
        hull_tris.push(tri);
    }
}

fn remove_unreferenced_verts(hull_points: &mut Vec<Vec3>, hull_tris: &mut Vec<Tri>) {
    // for i in 0..hull_points.len() as u32 {
    let mut i = 0;
    while i < hull_points.len() as u32 {
        let mut is_used = false;
        for tri in hull_tris.iter() {
            if tri.a == i || tri.b == i || tri.c == i {
                is_used = true;
                break;
            }
        }

        if is_used {
            i += 1;
            continue;
        }

        for tri in hull_tris.iter_mut() {
            if tri.a > i {
                tri.a -= 1;
            }
            if tri.b > i {
                tri.b -= 1;
            }
            if tri.c > i {
                tri.c -= 1;
            }
        }

        hull_points.remove(i as usize);
        // i -= 1;
    }
}

fn expand_convex_hull(hull_points: &mut Vec<Vec3>, hull_tris: &mut Vec<Tri>, verts: &[Vec3]) {
    let mut external_verts = Vec::from(verts);
    remove_internal_points(hull_points, hull_tris, &mut external_verts);

    while !external_verts.is_empty() {
        let pt_idx = find_point_furthest_in_dir(&external_verts, external_verts[0]);

        let pt = external_verts[pt_idx];

        // remove this element
        // TODO: could use swap_remove? Is ordering important?
        external_verts.remove(pt_idx);

        add_point(hull_points, hull_tris, pt);

        remove_internal_points(hull_points, hull_tris, &mut external_verts);
    }

    remove_unreferenced_verts(hull_points, hull_tris);
}

fn build_convex_hull(verts: &[Vec3], hull_points: &mut Vec<Vec3>, hull_tris: &mut Vec<Tri>) {
    if verts.len() < 4 {
        return;
    }

    build_tetrahedron(verts, hull_points, hull_tris);

    expand_convex_hull(hull_points, hull_tris, verts);
}

fn is_external(pts: &[Vec3], tris: &[Tri], pt: Vec3) -> bool {
    for tri in tris {
        let a = pts[tri.a as usize];
        let b = pts[tri.b as usize];
        let c = pts[tri.c as usize];

        // if the point is in front of any triangle then it's external
        let dist = distance_from_triangle(a, b, c, pt);
        if dist > 0.0 {
            return true;
        }
    }

    false
}

fn calculate_center_of_mass(pts: &[Vec3], tris: &[Tri]) -> Vec3 {
    const NUM_SAMPLES: usize = 100;

    let bounds = Bounds::from_points(pts);

    let dv = bounds.width() / NUM_SAMPLES as f32;

    let mut cm = Vec3::ZERO;
    let mut sample_count = 0;

    for i in 0..NUM_SAMPLES {
        let x = bounds.mins.x + dv.x * i as f32;
        for j in 0..NUM_SAMPLES {
            let y = bounds.mins.y + dv.y as f32;
            for k in 0..NUM_SAMPLES {
                let z = bounds.mins.z + dv.z as f32;
                let pt = Vec3::new(x, y, z);
                if is_external(pts, tris, pt) {
                    continue;
                }

                cm += pt;
                sample_count += 1;
            }
        }
    }

    cm / sample_count as f32
}

fn calculate_inertia_tensor(pts: &[Vec3], tris: &[Tri], cm: Vec3) -> Mat3 {
    const NUM_SAMPLES: usize = 100;

    let bounds = Bounds::from_points(pts);

    let mut tensor = Mat3::ZERO;

    let dv = bounds.width() / NUM_SAMPLES as f32;

    let mut sample_count = 0;

    for i in 0..NUM_SAMPLES {
        let x = bounds.mins.x + dv.x * i as f32;
        for j in 0..NUM_SAMPLES {
            let y = bounds.mins.y + dv.y as f32;
            for k in 0..NUM_SAMPLES {
                let z = bounds.mins.z + dv.z as f32;
                let pt = Vec3::new(x, y, z);
                if is_external(pts, tris, pt) {
                    continue;
                }

                // Get the point relative to the center of mass
                pt -= cm;

                // TODO: change glam to support this?
                tensor[0][0] += pt.y * pt.y + pt.z * pt.z;

                sample_count += 1;
            }
        }
    }

    tensor * (sample_count as f32).recip()
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
