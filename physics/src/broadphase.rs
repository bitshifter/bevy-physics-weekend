use crate::body::Body;
use crate::scene::BodyHandle;
use glam::Vec3;

#[derive(Copy, Clone, Debug)]
pub struct CollisionPair {
    pub a: BodyHandle,
    pub b: BodyHandle,
}

impl PartialEq for CollisionPair {
    fn eq(&self, other: &Self) -> bool {
        (self.a == other.a && self.b == other.b) || (self.a == other.b && self.b == other.a)
    }
}

impl Eq for CollisionPair {}

#[derive(Copy, Clone, Debug)]
struct PsuedoBody {
    handle: BodyHandle,
    value: f32,
    is_min: bool,
}

fn compare_sat(a: &PsuedoBody, b: &PsuedoBody) -> std::cmp::Ordering {
    if a.value < b.value {
        std::cmp::Ordering::Less
    } else if a.value > b.value {
        std::cmp::Ordering::Greater
    } else {
        std::cmp::Ordering::Equal
    }
}

fn sort_bodies_bounds(bodies: &[Body], dt_sec: f32) -> Vec<PsuedoBody> {
    // TODO: allocation on sort
    let mut sorted_bodies = Vec::with_capacity(bodies.len() * 2);

    let axis = Vec3::ONE.normalize();
    for (i, body) in bodies.iter().enumerate() {
        let mut bounds = body.shape.bounds(body.position, body.orientation);

        // expand the bounds by the linear velocity
        bounds.expand_by_point(bounds.mins + body.linear_velocity * dt_sec);
        bounds.expand_by_point(bounds.maxs + body.linear_velocity * dt_sec);

        const BOUNDS_EPS: f32 = 0.01;
        bounds.expand_by_point(bounds.mins - Vec3::splat(BOUNDS_EPS));
        bounds.expand_by_point(bounds.maxs + Vec3::splat(BOUNDS_EPS));

        sorted_bodies.push(PsuedoBody {
            handle: BodyHandle(i as u32),
            value: axis.dot(bounds.mins),
            is_min: true,
        });
        sorted_bodies.push(PsuedoBody {
            handle: BodyHandle(i as u32),
            value: axis.dot(bounds.maxs),
            is_min: false,
        });
    }

    sorted_bodies.sort_unstable_by(compare_sat);

    sorted_bodies
}

fn build_pairs(sorted_bodies: &[PsuedoBody]) -> Vec<CollisionPair> {
    let mut collision_pairs = Vec::new();

    // Now that the bodies are sorted, build the collision pairs
    for i in 0..sorted_bodies.len() {
        let a = &sorted_bodies[i];
        if !a.is_min {
            continue;
        }

        for j in (i + 1)..sorted_bodies.len() {
            let b = &sorted_bodies[j];
            // if we've hit the end of the a element then we're done creating pairs with a
            if b.handle == a.handle {
                break;
            }

            if !b.is_min {
                continue;
            }

            collision_pairs.push(CollisionPair {
                a: a.handle,
                b: b.handle,
            });
        }
    }

    collision_pairs
}

fn sweep_and_prune_1d(bodies: &[Body], dt_sec: f32) -> Vec<CollisionPair> {
    let sorted_bodies = sort_bodies_bounds(bodies, dt_sec);
    build_pairs(&sorted_bodies)
}

pub fn broadphase(bodies: &[Body], dt_sec: f32) -> Vec<CollisionPair> {
    sweep_and_prune_1d(bodies, dt_sec)
}
