use crate::{
    body::Body,
    gjk::{gjk_closest_points, gjk_does_intersect},
    scene::{BodyHandle, Contact},
    shapes::Shape,
};
use glam::Vec3;
use std::borrow::Borrow;

pub fn sphere_sphere_static(
    radius_a: f32,
    radius_b: f32,
    pos_a: Vec3,
    pos_b: Vec3,
) -> Option<(Vec3, Vec3)> {
    let ab = pos_b - pos_a;
    let radius_ab = radius_a + radius_b;
    let length_squared = ab.length_squared();
    if length_squared < radius_ab * radius_ab {
        let norm = ab.normalize_or_zero();
        let pt_on_a = pos_a + norm * radius_a;
        let pt_on_b = pos_b - norm * radius_b;
        Some((pt_on_a, pt_on_b))
    } else {
        None
    }
}

pub fn ray_sphere_intersect(
    ray_start: Vec3,
    ray_dir: Vec3,
    sphere_centre: Vec3,
    sphere_radius: f32,
) -> Option<(f32, f32)> {
    let m = sphere_centre - ray_start;
    let a = ray_dir.dot(ray_dir);
    let b = m.dot(ray_dir);
    let c = m.dot(m) - sphere_radius * sphere_radius;

    let b2 = b * b;
    let delta = b2 - (a * c);

    if delta < 0.0 {
        None
    } else {
        let inv_a = 1.0 / a;
        let delta_root = delta.sqrt();
        let t1 = inv_a * (b - delta_root);
        let t2 = inv_a * (b + delta_root);
        Some((t1, t2))
    }
}

pub fn sphere_sphere_dynamic(
    radius_a: f32,
    radius_b: f32,
    pos_a: Vec3,
    pos_b: Vec3,
    vel_a: Vec3,
    vel_b: Vec3,
    dt: f32,
) -> Option<(Vec3, Vec3, f32)> {
    let relative_velocity = vel_a - vel_b;

    let start_pt_a = pos_a;
    let end_pt_a = pos_a + relative_velocity * dt;
    let ray_dir = end_pt_a - start_pt_a;

    let mut t0 = 0.0;
    let mut t1 = 0.0;

    const EPSILON: f32 = 0.001;
    const EPSILON_SQ: f32 = EPSILON * EPSILON;
    if ray_dir.length_squared() < EPSILON_SQ {
        // ray is too short, just check if intersecting
        let ab = pos_b - pos_a;
        let radius = radius_a + radius_b + EPSILON;
        if ab.length_squared() > radius * radius {
            return None;
        }
    } else if let Some(toi) = ray_sphere_intersect(pos_a, ray_dir, pos_b, radius_a + radius_b) {
        t0 = toi.0;
        t1 = toi.1;
    } else {
        return None;
    }

    // Change from [0,1] range to [0,dt] range
    t0 *= dt;
    t1 *= dt;

    // If the collision is only in the past, then there's not future collision this frame
    if t1 < 0.0 {
        return None;
    }

    // Get the earliest positive time of impact
    let toi = if t0 < 0.0 { 0.0 } else { t0 };

    // If the earliest collision is too far in the future, then there's no collision this frame
    if toi > dt {
        return None;
    }

    // get the points on the respective points of collision
    let new_pos_a = pos_a + vel_a * toi;
    let new_pos_b = pos_b + vel_b * toi;
    let ab = (new_pos_b - new_pos_a).normalize();

    let pt_on_a = new_pos_a + ab * radius_a;
    let pt_on_b = new_pos_b - ab * radius_b;

    Some((pt_on_a, pt_on_b, toi))
}

fn intersect_static(
    handle_a: BodyHandle,
    body_a: &Body,
    handle_b: BodyHandle,
    body_b: &Body,
) -> (Contact, bool) {
    match (&body_a.shape, &body_b.shape) {
        (Shape::Sphere(sphere_a), Shape::Sphere(sphere_b)) => {
            let pos_a = body_a.position;
            let pos_b = body_b.position;

            if let Some((world_point_a, world_point_b)) =
                sphere_sphere_static(sphere_a.radius, sphere_b.radius, pos_a, pos_b)
            {
                (
                    Contact {
                        world_point_a,
                        world_point_b,
                        local_point_a: body_a.world_to_local(world_point_a),
                        local_point_b: body_b.world_to_local(world_point_b),
                        normal: (pos_a - pos_b).normalize(),
                        separation_dist: (world_point_a - world_point_b).length(),
                        time_of_impact: 0.0,
                        handle_a,
                        handle_b,
                    },
                    true,
                )
            } else {
                (
                    Contact {
                        world_point_a: Vec3::ZERO,
                        world_point_b: Vec3::ZERO,
                        local_point_a: Vec3::ZERO,
                        local_point_b: Vec3::ZERO,
                        normal: Vec3::X,
                        separation_dist: 0.0,
                        time_of_impact: 0.0,
                        handle_a,
                        handle_b,
                    },
                    false,
                )
            }
        }
        (_, _) => {
            const BIAS: f32 = 0.001;
            if let Some((mut world_point_a, mut world_point_b)) =
                gjk_does_intersect(body_a, body_b, BIAS)
            {
                let normal = (world_point_b - world_point_a).normalize();
                world_point_a -= normal * BIAS;
                world_point_b += normal * BIAS;
                (
                    Contact {
                        world_point_a,
                        world_point_b,
                        local_point_a: body_a.world_to_local(world_point_a),
                        local_point_b: body_b.world_to_local(world_point_b),
                        normal,
                        separation_dist: (world_point_a - world_point_b).length(),
                        time_of_impact: 0.0,
                        handle_a,
                        handle_b,
                    },
                    true,
                )
            } else {
                let (world_point_a, world_point_b) = gjk_closest_points(body_a, body_b);
                (
                    Contact {
                        world_point_a,
                        world_point_b,
                        local_point_a: body_a.world_to_local(world_point_a),
                        local_point_b: body_b.world_to_local(world_point_b),
                        normal: Vec3::ZERO,
                        separation_dist: (world_point_a - world_point_b).length(),
                        time_of_impact: 0.0,
                        handle_a,
                        handle_b,
                    },
                    false,
                )
            }
        }
    }
}

pub fn intersect_dynamic(
    handle_a: BodyHandle,
    body_a: &mut Body,
    handle_b: BodyHandle,
    body_b: &mut Body,
    delta_seconds: f32,
) -> Option<Contact> {
    // skip body pairs with infinite mass
    if body_a.has_infinite_mass() && body_b.has_infinite_mass() {
        return None;
    }

    let shape_a = body_a.shape.clone();
    let shape_b = body_b.shape.clone();
    let shapes = (shape_a.borrow(), shape_b.borrow());
    match shapes {
        (Shape::Sphere(sphere_a), Shape::Sphere(sphere_b)) => {
            if let Some((world_point_a, world_point_b, time_of_impact)) = sphere_sphere_dynamic(
                sphere_a.radius,
                sphere_b.radius,
                body_a.position,
                body_b.position,
                body_a.linear_velocity,
                body_b.linear_velocity,
                delta_seconds,
            ) {
                // step bodies forward to get local space collision points
                body_a.update(time_of_impact);
                body_b.update(time_of_impact);

                // convert world space contacts to local space
                let local_point_a = body_a.world_to_local(world_point_a);
                let local_point_b = body_b.world_to_local(world_point_b);

                let normal = (body_a.position - body_b.position).normalize();

                // unwind time step
                body_a.update(-time_of_impact);
                body_b.update(-time_of_impact);

                // calculate the separation distance
                let ab = body_a.position - body_b.position;
                let separation_dist = ab.length() - (sphere_a.radius + sphere_b.radius);

                Some(Contact {
                    world_point_a,
                    world_point_b,
                    local_point_a,
                    local_point_b,
                    normal,
                    separation_dist,
                    time_of_impact,
                    handle_a,
                    handle_b,
                })
            } else {
                None
            }
        }
        _ => {
            // use GJK to perform conservative advancement
            conservative_advance(handle_a, body_a, handle_b, body_b, delta_seconds)
        }
    }
}

fn conservative_advance(
    handle_a: BodyHandle,
    body_a: &mut Body,
    handle_b: BodyHandle,
    body_b: &mut Body,
    mut dt: f32,
) -> Option<Contact> {
    let mut toi = 0.0;
    let mut num_iters = 0;

    // advance the positions of the bodies until they touch or there's not time left
    while dt > 0.0 {
        // check for intersection
        let (mut contact, did_intersect) = intersect_static(handle_a, body_a, handle_b, body_b);
        if did_intersect {
            contact.time_of_impact = toi;
            body_a.update(-toi);
            body_b.update(-toi);
            return Some(contact);
        }

        num_iters += 1;
        if num_iters > 10 {
            break;
        }

        // get the vector from the closest point on A to the closest point on B
        let ab = (contact.world_point_b - contact.world_point_a).normalize();

        // project the relative velocity onto the ray of shortest distance
        let relative_velocity = body_a.linear_velocity - body_b.linear_velocity;
        let mut ortho_speed = relative_velocity.dot(ab);

        // add to the ortho_speed the maximum angular spees of the relative shaps
        let angular_speed_a = body_a
            .shape
            .fastest_linear_speed(body_a.angular_velocity, ab);
        let angular_speed_b = body_b
            .shape
            .fastest_linear_speed(body_b.angular_velocity, -ab);
        ortho_speed += angular_speed_a + angular_speed_b;

        if ortho_speed <= 0.0 {
            break;
        }

        let time_to_go = contact.separation_dist / ortho_speed;
        if time_to_go > dt {
            break;
        }

        dt -= time_to_go;
        toi += time_to_go;
        body_a.update(time_to_go);
        body_b.update(time_to_go);
    }

    // unwind the clock
    body_a.update(-toi);
    body_b.update(-toi);

    None
}
