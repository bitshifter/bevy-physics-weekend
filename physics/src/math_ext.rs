use glam::{Mat3, Mat4};

pub trait Mat4Ext {
    fn minor(&self, i: usize, j: usize) -> Mat3;
    fn cofactor(&self, i: usize, j: usize) -> f32;
}

impl Mat4Ext for Mat4 {
    fn minor(&self, i: usize, j: usize) -> Mat3 {
        let mut minor = Mat3::ZERO;
        let mut yy = 0;
        for y in 0..4 {
            if y == j {
                continue;
            }

            let mut xx = 0;
            for x in 0..4 {
                if x == i {
                    continue;
                }

                // TODO: could do with `col_mut()` here
                minor.as_mut()[xx * 3 + yy] = self.as_ref()[x * 4 + y];
                xx += 1;
            }

            yy += 1;
        }
        minor
    }

    fn cofactor(&self, i: usize, j: usize) -> f32 {
        let minor = self.minor(i, j);
        i32::pow(-1, (i + 1 + j + 1) as u32) as f32 * minor.determinant()
    }
}
