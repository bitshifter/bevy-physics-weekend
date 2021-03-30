#![allow(dead_code)]

use crate::{body::Body, math_ext::Mat4Ext};
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
        let lambdas = Vec2::new(c2 / mu_max, c1 / mu_max);
        lambdas
    } else if (a <= b && p <= a) || (a >= b && p >= a) {
        // if p is on the far side of a
        Vec2::X
    } else {
        // p must be on the far side of b
        Vec2::Y
    }
}

fn compare_signs(a: f32, b: f32) -> i32 {
    if a > 0.0 && b > 0.0 {
        1
    } else if a < 0.0 && b < 0.0 {
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
        let lambdas = c4 * det_m.recip();
        lambdas
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
fn simple_signed_volumes(pts: &[Point], new_dir: &mut Vec3, lambdas_out: &mut Vec4) -> bool {
    const EPSILON: f32 = 0.0001 * 0.0001;
    match pts.len() {
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
fn has_point(simplex_points: &[Point; 4], new_pt: &Point) -> bool {
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

fn gjk_does_intersect(body_a: &Body, body_b: &Body) -> bool {
    const ORIGIN: Vec3 = Vec3::ZERO;

    let mut num_pts = 1;
    let mut simplex_points = [Point::new(); 4];
    simplex_points[0] = support(body_a, body_b, Vec3::ONE, 0.0);

    let mut closest_dist = f32::MAX;
    let mut does_contain_origin = false;
    let mut new_dir = -simplex_points[0].xyz;
    loop {
        // Get the new point to check on
        let new_pt = support(body_a, body_b, new_dir, 0.0);
        if has_point(&simplex_points, &new_pt) {
            break;
        }

        simplex_points[num_pts] = new_pt;
        num_pts += 1;

        // If this new point hasn't moved past the origin then the origin can not be in the set.
        // Therefore there is no collision.
        let dotdot = new_dir.dot(new_pt.xyz - ORIGIN);
        if dotdot < 0.0 {
            break;
        }

        let mut lambdas = Vec4::ZERO;
        does_contain_origin =
            simple_signed_volumes(&simplex_points[0..num_pts], &mut new_dir, &mut lambdas);
        if does_contain_origin {
            break;
        }

        // Check that the new projection of the origin onto the simplex is closer than the previous
        let dist = new_dir.length_squared();
        if dist >= closest_dist {
            break;
        }
        closest_dist = dist;

        // Use the lambdas that support the new search direction and invalidate any points that
        // don't support it
        sort_valids(&mut simplex_points, &mut lambdas);
        num_pts = num_valids(&lambdas);
        does_contain_origin = num_pts == 4;
        if does_contain_origin {
            break;
        }
    }
    does_contain_origin
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
