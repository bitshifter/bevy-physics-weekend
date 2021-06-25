use crate::body::BodyHandle;
use glam::Vec3;

#[derive(Copy, Clone, Debug)]
pub struct Contact {
    pub world_point_a: Vec3,
    pub world_point_b: Vec3,
    pub local_point_a: Vec3,
    pub local_point_b: Vec3,
    pub normal: Vec3,

    pub separation_dist: f32,
    pub time_of_impact: f32,

    pub handle_a: BodyHandle,
    pub handle_b: BodyHandle,
}

pub struct ContactArena {
    contacts: Vec<Contact>,
}

impl ContactArena {
    pub fn new() -> Self {
        Self {
            contacts: Vec::new(),
        }
    }

    pub fn clear(&mut self) {
        self.contacts.clear();
    }

    pub fn clear_with_capacity(&mut self, capacity: usize) {
        self.contacts.clear();
        self.contacts.reserve(capacity);
    }

    pub fn push(&mut self, contact: Contact) {
        self.contacts.push(contact);
    }

    pub fn sort(&mut self) {
        self.contacts.sort_unstable_by(|a, b| {
            // TODO: fix lint?
            #[allow(clippy::float_cmp)]
            if a.time_of_impact < b.time_of_impact {
                std::cmp::Ordering::Less
            } else if a.time_of_impact == b.time_of_impact {
                std::cmp::Ordering::Equal
            } else {
                std::cmp::Ordering::Greater
            }
        });
    }

    pub fn iter(&self) -> core::slice::Iter<Contact> {
        self.contacts.iter()
    }
}
