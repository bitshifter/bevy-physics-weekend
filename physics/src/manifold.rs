#![allow(dead_code)]
use crate::{
    body::{BodyArena, BodyHandle},
    constraints::{Constraint, ConstraintConfig, ConstraintPenetration},
    contact::Contact,
};
use glam::Vec3;

const MAX_CONTACTS: usize = 4;

#[derive(Copy, Clone, Debug)]
struct Manifold {
    contacts: [Contact; MAX_CONTACTS],
    num_contacts: u8,
    handle_a: BodyHandle,
    handle_b: BodyHandle,
    constraints: [ConstraintPenetration; MAX_CONTACTS],
}

impl Manifold {
    fn new(handle_a: BodyHandle, handle_b: BodyHandle) -> Self {
        Self {
            contacts: [Contact::default(); MAX_CONTACTS],
            num_contacts: 0,
            handle_a,
            handle_b,
            constraints: [ConstraintPenetration::default(); MAX_CONTACTS],
        }
    }

    fn from_contact(bodies: &BodyArena, contact: Contact) -> Self {
        let mut manifold = Self::new(contact.handle_a, contact.handle_b);
        manifold.add_contact(bodies, contact);
        manifold
    }

    fn add_contact(&mut self, bodies: &BodyArena, mut contact: Contact) {
        // make sure the contact's body_a and body_b are of the correct order
        if contact.handle_a != self.handle_a || contact.handle_b != self.handle_b {
            std::mem::swap(&mut contact.local_point_a, &mut contact.local_point_b);
            std::mem::swap(&mut contact.world_point_a, &mut contact.world_point_b);
            std::mem::swap(&mut contact.handle_a, &mut contact.handle_b);
        }

        // if this contact is close to another contact then keep the old contact
        for manifold_contact in &self.contacts[0..self.num_contacts as usize] {
            let body_a = bodies.get_body(manifold_contact.handle_a);
            let body_b = bodies.get_body(manifold_contact.handle_b);

            let old_a = body_a.local_to_world(manifold_contact.local_point_a);
            let old_b = body_b.local_to_world(manifold_contact.local_point_b);

            let new_a = bodies
                .get_body(contact.handle_a)
                .local_to_world(contact.local_point_a);
            let new_b = bodies
                .get_body(contact.handle_b)
                .local_to_world(contact.local_point_b);

            let aa = new_a - old_a;
            let bb = new_b - old_b;

            const DISTANCE_THRESHOLD: f32 = 0.02;
            const DISTANCE_THRESHOLD_SQ: f32 = DISTANCE_THRESHOLD * DISTANCE_THRESHOLD;
            if aa.length_squared() < DISTANCE_THRESHOLD_SQ {
                return;
            }
            if bb.length_squared() < DISTANCE_THRESHOLD_SQ {
                return;
            }
        }

        // if we're all full on contacts then keep the contacts that are furthest away from each
        // other
        let mut new_slot = self.num_contacts as usize;
        if new_slot >= MAX_CONTACTS {
            let mut avg = Vec3::ZERO;
            avg += self.contacts[0].local_point_a;
            avg += self.contacts[1].local_point_a;
            avg += self.contacts[2].local_point_a;
            avg += self.contacts[3].local_point_a;
            avg += contact.local_point_a;
            avg *= 0.2;

            let mut min_dist = (avg - contact.local_point_a).length_squared();
            let mut new_idx = None;
            for i in 0..MAX_CONTACTS {
                let dist2 = (avg - self.contacts[i].local_point_a).length_squared();

                if dist2 < min_dist {
                    min_dist = dist2;
                    new_idx = Some(i);
                }
            }

            if let Some(new_idx) = new_idx {
                new_slot = new_idx;
            } else {
                return;
            }
        }

        self.contacts[new_slot] = contact;

        // TODO: might be cheaper to reused existing constraint rather than memcpying a new one
        let mut normal = bodies.get_body(self.handle_a).orientation.inverse() * -contact.normal;
        normal = normal.normalize();
        self.constraints[new_slot] = ConstraintPenetration::new(
            ConstraintConfig {
                handle_a: contact.handle_a,
                handle_b: contact.handle_b,
                anchor_a: contact.local_point_a,
                anchor_b: contact.local_point_b,
                ..ConstraintConfig::default()
            },
            normal,
        );

        if new_slot == self.num_contacts as usize {
            self.num_contacts += 1;
        }
    }

    fn remove_expired_contacts(&mut self, bodies: &BodyArena) {
        // remove any contacts that have drifted too far
        let mut i = 0;
        while i < self.num_contacts as usize {
            let contact = &self.contacts[i];
            let body_a = bodies.get_body(contact.handle_a);
            let body_b = bodies.get_body(contact.handle_b);

            // get the tangential distance of the point on a and the point on b
            let a = body_a.local_to_world(contact.local_point_a);
            let b = body_b.local_to_world(contact.local_point_b);

            let mut normal = self.constraints[i].normal();
            normal = body_a.orientation * normal;

            // calculate the tangential separation and penetration depth
            let ab = b - a;
            let penetration_depth = normal.dot(ab);
            let ab_normal = normal * penetration_depth;
            let ab_tangent = ab - ab_normal;

            // if the tangential displacement is less than a specific threshold, it's okay to keep
            // it.
            const DISTANCE_THRESHOLD: f32 = 0.02;
            if ab_tangent.length_squared() < DISTANCE_THRESHOLD * DISTANCE_THRESHOLD
                && penetration_depth <= 0.0
            {
                i += 1;
                continue;
            }

            // this contact has moved beyond its threshold and should be removed
            for j in i..(MAX_CONTACTS - 1) {
                self.constraints[j] = self.constraints[j + 1];
                self.contacts[j] = self.contacts[j + 1];
                if j >= self.num_contacts as usize {
                    self.constraints[j].clear_cached_lambda();
                }
            }
            self.num_contacts -= 1;
        }
    }

    fn constraints_as_mut_slice(&mut self) -> &mut [ConstraintPenetration] {
        &mut self.constraints[0..self.num_contacts as usize]
    }

    fn pre_solve(&mut self, bodies: &mut BodyArena, dt_sec: f32) {
        for constraint in self.constraints_as_mut_slice() {
            constraint.pre_solve(bodies, dt_sec);
        }
    }

    fn solve(&mut self, bodies: &mut BodyArena) {
        for constraint in self.constraints_as_mut_slice() {
            constraint.solve(bodies);
        }
    }

    fn post_solve(&mut self) {
        for constraint in self.constraints_as_mut_slice() {
            constraint.post_solve();
        }
    }

    fn contact(&self, index: usize) -> &Contact {
        assert!(index < MAX_CONTACTS as usize);
        &self.contacts[index]
    }

    fn num_contacts(&self) -> usize {
        self.num_contacts as usize
    }
}

#[derive(Clone, Debug, Default)]
pub struct ManifoldCollector {
    manifolds: Vec<Manifold>,
}

impl ManifoldCollector {
    pub fn add_contact(&mut self, bodies: &BodyArena, contact: Contact) {
        // try to find the previously existing manifold for contacts between two bodies
        let mut found = None;
        for manifold in &mut self.manifolds {
            #[allow(clippy::suspicious_operation_groupings)]
            let has_a =
                manifold.handle_a == contact.handle_a || manifold.handle_b == contact.handle_a;
            #[allow(clippy::suspicious_operation_groupings)]
            let has_b =
                manifold.handle_a == contact.handle_b || manifold.handle_b == contact.handle_b;
            if has_a && has_b {
                found = Some(manifold);
                break;
            }
        }

        if let Some(manifold) = found {
            manifold.add_contact(bodies, contact);
        } else {
            self.manifolds.push(Manifold::from_contact(bodies, contact));
        }
    }

    pub fn remove_expired(&mut self, bodies: &BodyArena) {
        for manifold in &mut self.manifolds {
            manifold.remove_expired_contacts(bodies);
        }
        self.manifolds
            .retain(|&manifold| manifold.num_contacts() > 0);
    }

    pub fn pre_solve(&mut self, bodies: &mut BodyArena, dt_sec: f32) {
        for manifold in &mut self.manifolds {
            manifold.pre_solve(bodies, dt_sec);
        }
    }

    pub fn solve(&mut self, bodies: &mut BodyArena) {
        for manifold in &mut self.manifolds {
            manifold.solve(bodies);
        }
    }

    pub fn post_solve(&mut self) {
        for manifold in &mut self.manifolds {
            manifold.post_solve();
        }
    }

    pub fn clear(&mut self) {
        self.manifolds.clear();
    }
}
