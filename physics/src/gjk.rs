#![allow(dead_code)]
#![allow(clippy::many_single_char_names)]

use crate::{
    body::Body,
    math::glam_ext::Mat4Ext,
    shapes::{Edge, Tri},
};
use glam::{Mat4, Vec2, Vec3, Vec4};

#[derive(Debug, Copy, Clone, PartialEq)]
struct Point {
    xyz: Vec3,  // The point on the minkowski sum
    pt_a: Vec3, // the point on body a
    pt_b: Vec3, // the point on body b
}

impl Point {
    fn new() -> Point {
        Self {
            xyz: Vec3::ZERO,
            pt_a: Vec3::ZERO,
            pt_b: Vec3::ZERO,
        }
    }
}

fn signed_volume_1d(s1: Vec3, s2: Vec3) -> Vec2 {
    let ab = s2 - s1; // ray from a to b
    let ap = Vec3::ZERO - s1; // ray from a to origin
    let p0 = s1 + ab * ab.dot(ap) / ab.length_squared(); // projection of the origin onto the line

    // Choose the axis with the greatest difference/length
    let mut idx = 0;
    let mut mu_max = 0.0;
    for i in 0..3 {
        let mu = s2[i] - s1[i];
        if mu * mu > mu_max * mu_max {
            mu_max = mu;
            idx = i;
        }
    }

    // Project the simplex points and projected origin onto the axis with the greatest length
    let a = s1[idx];
    let b = s2[idx];
    let p = p0[idx];

    // Get the signed distance from a to p and from p to b
    let c1 = p - a;
    let c2 = b - p;

    if (p > a && p < b) || (p > b && p < a) {
        // if p is between [a,b]
        Vec2::new(c2 / mu_max, c1 / mu_max) // lambdas
    } else if (a <= b && p <= a) || (a >= b && p >= a) {
        // if p is on the far side of a
        Vec2::X
    } else {
        // p must be on the far side of b
        Vec2::Y
    }
}

fn compare_signs(a: f32, b: f32) -> i32 {
    if (a > 0.0 && b > 0.0) || (a < 0.0 && b < 0.0) {
        1
    } else {
        0
    }
}

fn signed_volume_2d(s1: Vec3, s2: Vec3, s3: Vec3) -> Vec3 {
    let normal = (s2 - s1).cross(s3 - s1);
    let p0 = normal * s1.dot(normal) / normal.length_squared();

    // find the axis with the greatest projected area
    let mut idx = 0;
    let mut area_max = 0.0;
    for i in 0..3 {
        let j = (i + 1) % 3;
        let k = (i + 2) % 3;

        let a = Vec2::new(s1[j], s1[k]);
        let b = Vec2::new(s2[j], s2[k]);
        let c = Vec2::new(s3[j], s3[k]);
        let ab = b - a;
        let ac = c - a;
        let area = ab.x * ac.y - ab.y * ac.x;
        if area * area > area_max * area_max {
            idx = i;
            area_max = area;
        }
    }

    // Project onto the appropriate axis
    let x = (idx + 1) % 3;
    let y = (idx + 2) % 3;
    let s = [
        Vec2::new(s1[x], s1[y]),
        Vec2::new(s2[x], s2[y]),
        Vec2::new(s3[x], s3[y]),
    ];
    let p = Vec2::new(p0[x], p0[y]);

    // Get the sub-areas of the triangles formed from the projected origin and the edges
    let mut areas = Vec3::ZERO;
    for i in 0..3 {
        let j = (i + 1) % 3;
        let k = (i + 2) % 3;

        let a = p;
        let b = s[j];
        let c = s[k];
        let ab = b - a;
        let ac = c - a;

        areas[i] = ab.x * ac.y - ab.y * ac.x;
    }

    if compare_signs(area_max, areas[0]) > 0
        && compare_signs(area_max, areas[1]) > 0
        && compare_signs(area_max, areas[2]) > 0
    {
        // If the projected origin is inside the triangle, then return the barycentric points
        areas / area_max
    } else {
        // If we make it here, then we need to project onto the edges and determine the closest point
        let mut dist = f32::MAX;
        let mut lambdas = Vec3::X;
        for i in 0..3 {
            let k = (i + 1) % 3;
            let l = (i + 2) % 3;

            let edges_pts = [s1, s2, s3];

            let lambda_edge = signed_volume_1d(edges_pts[k], edges_pts[l]);
            let pt = edges_pts[k] * lambda_edge[0] + edges_pts[l] * lambda_edge[1];
            if pt.length_squared() < dist {
                dist = pt.length_squared();
                lambdas[i] = 0.0;
                lambdas[k] = lambda_edge[0];
                lambdas[l] = lambda_edge[1];
            }
        }
        lambdas
    }
}

fn signed_volume_3d(s1: Vec3, s2: Vec3, s3: Vec3, s4: Vec3) -> Vec4 {
    let m = Mat4::from_cols(
        Vec4::new(s1.x, s2.x, s3.x, s4.x),
        Vec4::new(s1.y, s2.y, s3.y, s4.y),
        Vec4::new(s1.z, s2.z, s3.z, s4.z),
        Vec4::ONE,
    );

    let c4 = Vec4::new(
        m.cofactor(3, 0),
        m.cofactor(3, 1),
        m.cofactor(3, 2),
        m.cofactor(3, 3),
    );

    let det_m = c4[0] + c4[1] + c4[2] + c4[3];

    if compare_signs(det_m, c4[0]) > 0
        && compare_signs(det_m, c4[1]) > 0
        && compare_signs(det_m, c4[2]) > 0
        && compare_signs(det_m, c4[3]) > 0
    {
        // If the barycentric coordinates put the origin inside the simplex, then return them
        c4 * det_m.recip() // lambdas
    } else {
        // If we get here, then we need to project the origin onto the faces and determine the
        // closest one
        let mut lambdas = Vec4::ZERO;
        let mut dist = f32::MAX;
        for i in 0..4 {
            let j = (i + 1) % 4;
            let k = (i + 2) % 4;

            let face_pts = [s1, s2, s3, s4];

            let lambdas_face = signed_volume_2d(face_pts[i], face_pts[j], face_pts[k]);
            let pt = face_pts[i] * lambdas_face[0]
                + face_pts[j] * lambdas_face[1]
                + face_pts[k] * lambdas_face[2];
            if pt.length_squared() < dist {
                dist = pt.length_squared();
                let l = (i + 3) % 4;
                lambdas[i] = lambdas_face[0];
                lambdas[j] = lambdas_face[1];
                lambdas[k] = lambdas_face[2];
                lambdas[l] = 0.0;
            }
        }
        lambdas
    }
}

fn support(body_a: &Body, body_b: &Body, dir: Vec3, bias: f32) -> Point {
    let dir = dir.normalize();

    // Find the point in A furthest direction
    let pt_a = body_a
        .shape
        .support(dir, body_a.position, body_a.orientation, bias);

    let dir = -dir;

    // Find the point in B furthest direction
    let pt_b = body_b
        .shape
        .support(dir, body_b.position, body_b.orientation, bias);

    // Return the point in the minkowski sum, furthest in the direction
    Point {
        xyz: pt_a - pt_b,
        pt_a,
        pt_b,
    }
}

/// Projects the origin onto the simplex to acquire the new search direction, also checks if the
/// origin is "inside" the simplex.
fn simple_signed_volumes(
    pts: &[Point; 4],
    num_pts: usize,
    new_dir: &mut Vec3,
    lambdas_out: &mut Vec4,
) -> bool {
    const EPSILON: f32 = 0.0001 * 0.0001;
    match num_pts {
        2 => {
            let lambdas = signed_volume_1d(pts[0].xyz, pts[1].xyz);
            let mut v = Vec3::ZERO;
            for i in 0..2 {
                v += pts[i].xyz * lambdas[i];
            }
            *new_dir = -v;
            *lambdas_out = Vec4::new(lambdas.x, lambdas.y, 0.0, 0.0);
            v.length_squared() < EPSILON
        }
        3 => {
            let lambdas = signed_volume_2d(pts[0].xyz, pts[1].xyz, pts[2].xyz);
            let mut v = Vec3::ZERO;
            for i in 0..3 {
                v += pts[i].xyz * lambdas[i];
            }
            *new_dir = -v;
            *lambdas_out = Vec4::from((lambdas, 0.0));
            v.length_squared() < EPSILON
        }
        4 => {
            let lambdas = signed_volume_3d(pts[0].xyz, pts[1].xyz, pts[2].xyz, pts[3].xyz);
            let mut v = Vec3::ZERO;
            for i in 0..4 {
                v += pts[i].xyz * lambdas[i];
            }
            *new_dir = -v;
            *lambdas_out = lambdas;
            v.length_squared() < EPSILON
        }
        _ => unreachable!(),
    }
}

/// Checks whether the new point already exists in the simplex
fn simplex_has_point(simplex_points: &[Point; 4], new_pt: &Point) -> bool {
    const PRECISION_SQ: f32 = 1e-6 * 1e-6;
    for pt in simplex_points {
        let delta = pt.xyz - new_pt.xyz;
        if delta.length_squared() < PRECISION_SQ {
            return true;
        }
    }
    false
}

/// Sorts the valid support points to the beginning of the array
fn sort_valids(simplex_points: &mut [Point; 4], lambdas: &mut Vec4) {
    let mut valids = [true; 4];
    for i in 0..4 {
        if lambdas[i] == 0.0 {
            valids[i] = false;
        }
    }

    let mut valid_lambdas = Vec4::ZERO;
    let mut valid_count = 0;
    let mut valid_points = [Point::new(); 4];
    for i in 0..4 {
        if valids[i] {
            valid_points[valid_count] = simplex_points[i];
            valid_lambdas[valid_count] = lambdas[i];
            valid_count += 1;
        }
    }

    // Copy the valids back into simplex points
    for i in 0..4 {
        simplex_points[i] = valid_points[i];
        lambdas[i] = valid_lambdas[i];
    }
}

fn num_valids(lambdas: &Vec4) -> usize {
    let mut num = 0;
    for i in 0..4 {
        if lambdas[i] != 0.0 {
            num += 1;
        }
    }
    num
}

pub fn gjk_does_intersect(body_a: &Body, body_b: &Body, bias: f32) -> Option<(Vec3, Vec3)> {
    const ORIGIN: Vec3 = Vec3::ZERO;

    let mut num_pts = 1;
    let mut simplex_points = [Point::new(); 4];
    simplex_points[0] = support(body_a, body_b, Vec3::ONE, 0.0);

    let mut closest_dist = f32::MAX;
    let mut new_dir = -simplex_points[0].xyz;
    loop {
        // Get the new point to check on
        let new_pt = support(body_a, body_b, new_dir, 0.0);

        // If the new point is the same as a previous point then we can't expand any further
        if simplex_has_point(&simplex_points, &new_pt) {
            return None;
        }

        simplex_points[num_pts] = new_pt;
        num_pts += 1;

        // If this new point hasn't moved past the origin then the origin can not be in the set
        // and therefore there is no collision.
        let dotdot = new_dir.dot(new_pt.xyz - ORIGIN);
        if dotdot < 0.0 {
            return None;
        }

        let mut lambdas = Vec4::ZERO;
        if simple_signed_volumes(&simplex_points, num_pts, &mut new_dir, &mut lambdas) {
            break;
        }

        // Check that the new projection of the origin onto the simplex is closer than the previous
        let dist = new_dir.length_squared();
        if dist >= closest_dist {
            return None;
        }
        closest_dist = dist;

        // Use the lambdas that support the new search direction and invalidate any points that
        // don't support it
        sort_valids(&mut simplex_points, &mut lambdas);
        num_pts = num_valids(&lambdas);
        if num_pts == 4 {
            break;
        }
    }

    // Check that we have a 3-simplex (EPA expects a tetrahedron)
    if num_pts == 1 {
        let search_dir = -simplex_points[0].xyz;
        let new_pt = support(body_a, body_b, search_dir, 0.0);
        simplex_points[num_pts] = new_pt;
        num_pts += 1;
    }

    if num_pts == 2 {
        let ab = simplex_points[1].xyz - simplex_points[0].xyz;
        let u = {
            // TODO: replace with glam Vec3::any_orthonormal_vector?
            // let (u, _v) = ab.get_ortho();
            let n = ab.normalize();
            let w = if n.z * n.z > 0.9 * 0.9 {
                Vec3::X
            } else {
                Vec3::Z
            };
            let u = w.cross(n).normalize();
            let v = n.cross(u).normalize();
            v.cross(n).normalize()
        };

        let new_dir = u;
        let new_pt = support(body_a, body_b, new_dir, 0.0);
        simplex_points[num_pts] = new_pt;
        num_pts += 1;
    }

    if num_pts == 3 {
        let ab = simplex_points[1].xyz - simplex_points[0].xyz;
        let ac = simplex_points[2].xyz - simplex_points[0].xyz;
        let norm = ab.cross(ac);

        let new_dir = norm;
        let new_pt = support(body_a, body_b, new_dir, 0.0);
        simplex_points[num_pts] = new_pt;
        num_pts += 1;
    }

    // Expand the simplex by the bias amount

    // Get the center point of the simplex
    let mut avg = Vec3::ZERO;
    for simplex_point in &simplex_points {
        avg += simplex_point.xyz;
    }
    avg *= 0.25;

    // Now expand the simplex by the bias amount
    for pt in &mut simplex_points[0..num_pts] {
        let dir = (pt.xyz - avg).normalize_or_zero();
        pt.pt_a += dir * bias;
        pt.pt_b -= dir * bias;
        pt.xyz = pt.pt_a - pt.pt_b;
    }

    // Perform EPA expansion of the simplex to find the closest face on the CSO
    Some(epa_expand(body_a, body_b, bias, &simplex_points))
}

// This borrows our signed volum code to perform the barycentric coordinates.
fn barycentric_coordinates(s1: Vec3, s2: Vec3, s3: Vec3, pt: Vec3) -> Vec3 {
    let s1 = s1 - pt;
    let s2 = s2 - pt;
    let s3 = s3 - pt;

    let normal = (s2 - s1).cross(s3 - s1);
    let p0 = normal * s1.dot(normal) / normal.length_squared();

    // Find the axis with the greatest projected area
    let mut idx = 0;
    let mut area_max = 0.0;
    for i in 0..3 {
        let j = (i + 1) % 3;
        let k = (i + 2) % 3;

        let a = Vec2::new(s1[j], s1[k]);
        let b = Vec2::new(s2[j], s2[k]);
        let c = Vec2::new(s3[j], s3[k]);
        let ab = b - a;
        let ac = c - a;

        let area = ab.x * ac.y - ab.y * ac.x;
        if area * area > area_max * area_max {
            idx = i;
            area_max = area;
        }
    }

    // Project onto the appropriate axis
    let x = (idx + 1) % 3;
    let y = (idx + 2) % 3;
    let s = [
        Vec2::new(s1[x], s1[y]),
        Vec2::new(s2[x], s2[y]),
        Vec2::new(s3[x], s3[y]),
    ];
    let p = Vec2::new(p0[x], p0[y]);

    // Get the sub-areas of the triangles formed from the projected origin and edges
    let mut areas = Vec3::ZERO;
    for i in 0..3 {
        let j = (i + 1) % 3;
        let k = (i + 2) % 3;

        let a = p;
        let b = s[j];
        let c = s[k];
        let ab = b - a;
        let ac = c - a;

        areas[i] = ab.x * ac.y - ab.y * ac.x;
    }

    let mut lambdas = areas / area_max;
    if !lambdas.is_finite() {
        lambdas = Vec3::X;
    }

    lambdas
}

fn normal_direction(tri: &Tri, points: &[Point]) -> Vec3 {
    let a = points[tri.a as usize].xyz;
    let b = points[tri.b as usize].xyz;
    let c = points[tri.c as usize].xyz;

    let ab = b - a;
    let ac = c - a;
    let normal = ab.cross(ac);
    normal.normalize()
}

fn signed_distance_to_triangle(tri: &Tri, pt: Vec3, points: &[Point]) -> f32 {
    let normal = normal_direction(tri, points);
    let a = points[tri.a as usize].xyz;
    let a2pt = pt - a;
    normal.dot(a2pt)
}

fn closest_triangle(triangles: &[Tri], points: &[Point]) -> Option<Tri> {
    let mut min_dist_sq = f32::MAX;
    let mut idx = None;
    for tri in triangles {
        let dist = signed_distance_to_triangle(tri, Vec3::ZERO, points);
        let dist_sq = dist * dist;
        if dist_sq < min_dist_sq {
            idx = Some(*tri);
            min_dist_sq = dist_sq;
        }
    }
    idx
}

fn triangle_has_point(w: Vec3, triangles: &[Tri], points: &[Point]) -> bool {
    const EPSILONS: f32 = 0.001 * 0.001;

    for tri in triangles {
        let delta = w - points[tri.a as usize].xyz;
        if delta.length_squared() < EPSILONS {
            return true;
        }

        let delta = w - points[tri.b as usize].xyz;
        if delta.length_squared() < EPSILONS {
            return true;
        }

        let delta = w - points[tri.c as usize].xyz;
        if delta.length_squared() < EPSILONS {
            return true;
        }
    }
    false
}

fn remove_triangles_facing_point(pt: Vec3, triangles: &mut Vec<Tri>, points: &[Point]) -> usize {
    let mut num_removed = 0;
    let mut i = 0;
    while i < triangles.len() {
        let tri = triangles[i];
        let dist = signed_distance_to_triangle(&tri, pt, points);
        if dist > 0.0 {
            // This triangle faces the point, remove it.
            triangles.remove(i);
            num_removed += 1;
        } else {
            i += 1;
        }
    }
    num_removed
}

fn find_dangling_edges(dangling_edges: &mut Vec<Edge>, triangles: &[Tri]) {
    dangling_edges.clear();

    for (i, tri) in triangles.iter().enumerate() {
        let edges = [
            Edge { a: tri.a, b: tri.b },
            Edge { a: tri.b, b: tri.c },
            Edge { a: tri.c, b: tri.a },
        ];

        let mut counts = [0; 3];
        for (j, tri2) in triangles.iter().enumerate() {
            if i == j {
                continue;
            }

            let edges2 = [
                Edge {
                    a: tri2.a,
                    b: tri2.b,
                },
                Edge {
                    a: tri2.b,
                    b: tri2.c,
                },
                Edge {
                    a: tri2.c,
                    b: tri2.a,
                },
            ];

            // TODO: could use zip here?
            for k in 0..3 {
                if edges[k] == edges2[0] {
                    counts[k] += 1;
                }
                if edges[k] == edges2[1] {
                    counts[k] += 1;
                }
                if edges[k] == edges2[2] {
                    counts[k] += 1;
                }
            }
        }

        // An edge that isn't shared is dangling
        // TODO: could use zip here?
        for (k, count) in counts.iter().enumerate() {
            if *count == 0 {
                dangling_edges.push(edges[k]);
            }
        }
    }
}

fn epa_expand(
    body_a: &Body,
    body_b: &Body,
    bias: f32,
    simplex_points: &[Point; 4],
) -> (Vec3, Vec3) {
    let mut points = Vec::new();
    let mut triangles = Vec::new();
    let mut dangling_edges = Vec::new();

    let mut center = Vec3::ZERO;
    for &simplex_point in simplex_points {
        points.push(simplex_point);
        center += simplex_point.xyz;
    }
    center *= 0.25;

    // Build the triangles
    for i in 0..4 {
        let j = (i + 1) % 4;
        let k = (i + 2) % 4;
        let mut tri = Tri { a: i, b: j, c: k };

        let unused_pt = (i + 3) % 4;
        let dist = signed_distance_to_triangle(&tri, points[unused_pt as usize].xyz, &points);

        // The unused point is always on the negative/inside of the triangle.. make sure the normal
        // points away
        if dist > 0.0 {
            std::mem::swap(&mut tri.a, &mut tri.b);
        }

        triangles.push(tri);
    }

    // Expand the simplex to find the closest face of the CSO to the origin
    loop {
        let tri = closest_triangle(&triangles, &points).unwrap();
        let normal = normal_direction(&tri, &points);

        let new_pt = support(body_a, body_b, normal, bias);

        // if w already exists then just stop because it means we can't expand any further
        if triangle_has_point(new_pt.xyz, &triangles, &points) {
            break;
        }

        let dist = signed_distance_to_triangle(&tri, new_pt.xyz, &points);
        if dist <= 0.0 {
            // can't append
            break;
        }

        let new_idx = points.len() as u32;
        points.push(new_pt);

        // Remove triangles that face this point
        let num_removed = remove_triangles_facing_point(new_pt.xyz, &mut triangles, &points);
        if num_removed == 0 {
            break;
        }

        // Find dangling edges
        find_dangling_edges(&mut dangling_edges, &triangles);
        if dangling_edges.is_empty() {
            break;
        }

        // In theory the edges should be a proper CCW order so we only need to add the new point as
        // `a` in order to create new triangles that face away from the origin.
        for edge in &dangling_edges {
            let mut triangle = Tri {
                a: new_idx,
                b: edge.b,
                c: edge.a,
            };

            // Make sure it's oriented properly
            let dist = signed_distance_to_triangle(&triangle, center, &points);
            if dist > 0.0 {
                std::mem::swap(&mut triangle.b, &mut triangle.c);
            }

            triangles.push(triangle);
        }
    }

    // Get the projection of the origin on the closest triangle
    let tri = closest_triangle(&triangles, &points).unwrap();
    let pt_a = &points[tri.a as usize];
    let pt_b = &points[tri.b as usize];
    let pt_c = &points[tri.c as usize];
    let lambdas = {
        let pt_a_w = pt_a.xyz;
        let pt_b_w = pt_b.xyz;
        let pt_c_w = pt_c.xyz;
        barycentric_coordinates(pt_a_w, pt_b_w, pt_c_w, Vec3::ZERO)
    };

    // Get the point on shape A
    let pt_on_a = {
        let pt_a_a = pt_a.pt_a;
        let pt_b_a = pt_b.pt_a;
        let pt_c_a = pt_c.pt_a;
        pt_a_a * lambdas[0] + pt_b_a * lambdas[1] + pt_c_a * lambdas[2]
    };

    // Get the point on shape B
    let pt_on_b = {
        let pt_a_b = pt_a.pt_b;
        let pt_b_b = pt_b.pt_b;
        let pt_c_b = pt_c.pt_b;
        pt_a_b * lambdas[0] + pt_b_b * lambdas[1] + pt_c_b * lambdas[2]
    };

    // Return the penetration distance
    // let delta = pt_on_b - pt_on_a;
    // delta.length()
    (pt_on_a, pt_on_b)
}

pub fn gjk_closest_points(body_a: &Body, body_b: &Body) -> (Vec3, Vec3) {
    let mut closest_dist = f32::MAX;
    const BIAS: f32 = 0.0;

    let mut num_pts = 1;

    let mut simplex_points = [Point::new(); 4];
    simplex_points[0] = support(body_a, body_b, Vec3::ONE, BIAS);

    let mut lambdas = Vec4::new(1.0, 0.0, 0.0, 0.0);
    let mut new_dir = -simplex_points[0].xyz;
    loop {
        // get the new point to check on
        let new_pt = support(body_a, body_b, new_dir, BIAS);

        // if the new point is the same as the previous point then we can't expand any further
        if simplex_has_point(&simplex_points, &new_pt) {
            break;
        }

        // add point and get new search direction
        simplex_points[num_pts] = new_pt;
        num_pts += 1;

        simple_signed_volumes(&simplex_points, num_pts, &mut new_dir, &mut lambdas);
        sort_valids(&mut simplex_points, &mut lambdas);
        num_pts = num_valids(&lambdas);

        // check that the new projection of the origin onto the simplex is closer than the previous
        let dist = new_dir.length_squared();
        if dist >= closest_dist {
            break;
        }
        closest_dist = dist;
        if num_pts >= 4 {
            break;
        }
    }

    let mut pt_on_a = Vec3::ZERO;
    let mut pt_on_b = Vec3::ZERO;
    for (i, point) in simplex_points.iter().enumerate() {
        pt_on_a += point.pt_a * lambdas[i];
        pt_on_b += point.pt_b * lambdas[i];
    }

    (pt_on_a, pt_on_b)
}

#[test]
fn test_signed_volume_projection() {
    let org_pts = [Vec3::ZERO, Vec3::X, Vec3::Y, Vec3::Z];

    let test_pts = |pts: [Vec3; 4], expected_lambdas: Vec4, expected_v: Vec3| {
        let lambdas = signed_volume_3d(pts[0], pts[1], pts[2], pts[3]);

        let mut v = Vec3::ZERO;
        for i in 0..4 {
            v += pts[i] * lambdas[i];
        }
        println!("lambdas: {}       v: {}", lambdas, v);
        assert!(expected_lambdas.abs_diff_eq(lambdas, 1e-3));
        assert!(expected_v.abs_diff_eq(v, 1e-3));
    };

    let mut pts = [Vec3::ZERO; 4];
    for i in 0..4 {
        pts[i] = org_pts[i] + Vec3::new(1.0, 1.0, 1.0);
    }
    test_pts(pts, Vec4::X, Vec3::ONE);

    for i in 0..4 {
        pts[i] = org_pts[i] + Vec3::new(-1.0, -1.0, -1.0) * 0.25;
    }
    test_pts(pts, Vec4::splat(0.25), Vec3::ZERO);

    for i in 0..4 {
        pts[i] = org_pts[i] + Vec3::new(-1.0, -1.0, -1.0);
    }
    test_pts(
        pts,
        Vec4::new(0.0, 0.333, 0.333, 0.333),
        Vec3::splat(-0.667),
    );

    for i in 0..4 {
        pts[i] = org_pts[i] + Vec3::new(1.0, 1.0, -0.5);
    }
    test_pts(pts, Vec4::new(0.5, 0.0, 0.0, 0.5), Vec3::new(1.0, 1.0, 0.0));

    pts[0] = Vec3::new(51.1996613, 26.1989613, 1.91339576);
    pts[1] = Vec3::new(-51.0567360, -26.0565681, -0.436143428);
    pts[2] = Vec3::new(50.8978920, -24.1035538, -1.04042661);
    pts[3] = Vec3::new(-49.1021080, 25.8964462, -1.04042661);
    test_pts(
        pts,
        Vec4::new(0.290, 0.302, 0.206, 0.202),
        Vec3::new(0.0, 0.0, 0.0),
    );
}
