use glam::Vec3;
use std::ops::{Add, AddAssign};

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Bounds {
    pub mins: Vec3,
    pub maxs: Vec3,
}

impl Bounds {
    pub fn new() -> Bounds {
        Bounds {
            mins: Vec3::splat(std::f32::MAX),
            maxs: Vec3::splat(-std::f32::MAX),
        }
    }

    pub fn from_points(pts: &[Vec3]) -> Self {
        pts.iter().fold(Bounds::new(), |acc, pt| acc + *pt)
    }

    // fn clear(&mut self) {
    //     *self = Self::new();
    // }

    // fn does_intersect(&self, rhs: &Self) -> bool {
    //     if self.maxs.cmplt(rhs.mins).any() {
    //         false
    //     } else if rhs.maxs.cmplt(self.mins).any() {
    //         false
    //     } else {
    //         true
    //     }
    // }

    // fn expand_by_points(&mut self, points: &[Vec3]) {
    //     for point in points {
    //         self.expand_by_point(*point);
    //     }
    // }

    pub fn expand_by_point(&mut self, pt: Vec3) {
        self.add_assign(pt);
    }

    // fn expand_by_bounds(&mut self, rhs: &Self) {
    //     self.expand_by_point(rhs.mins);
    //     self.expand_by_point(rhs.maxs);
    // }

    pub fn width(&self) -> Vec3 {
        self.maxs - self.mins
    }

    // fn width_x(&self) -> f32 {
    //     self.maxs.x - self.mins.x
    // }

    // fn width_y(&self) -> f32 {
    //     self.maxs.y - self.mins.y
    // }

    // fn width_z(&self) -> f32 {
    //     self.maxs.z - self.mins.z
    // }
}

impl Default for Bounds {
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

impl Add<Vec3> for Bounds {
    type Output = Self;
    fn add(self, pt: Vec3) -> Self::Output {
        Bounds {
            mins: Vec3::select(pt.cmplt(self.mins), pt, self.mins),
            maxs: Vec3::select(pt.cmpgt(self.maxs), pt, self.maxs),
        }
    }
}

impl AddAssign<Vec3> for Bounds {
    fn add_assign(&mut self, pt: Vec3) {
        self.mins = Vec3::select(pt.cmplt(self.mins), pt, self.mins);
        self.maxs = Vec3::select(pt.cmpgt(self.maxs), pt, self.maxs);
    }
}
