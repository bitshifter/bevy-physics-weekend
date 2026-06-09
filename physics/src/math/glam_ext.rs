use glam::{Mat3, Mat4};

pub trait Mat4Ext {
    fn cofactor(&self, i: usize, j: usize) -> f32;
}

impl Mat4Ext for Mat4 {
    fn cofactor(&self, i: usize, j: usize) -> f32 {
        let minor = Mat3::from_mat4_minor(*self, i, j);
        i32::pow(-1, (i + 1 + j + 1) as u32) as f32 * minor.determinant()
    }
}
