#![allow(dead_code)]
pub mod glam_ext;
mod lcp;
mod matrix;
mod vector;

pub use lcp::lcp_gauss_seidel;
pub use matrix::{MatMN, MatN};
pub use vector::VecN;

pub(crate) fn dot<const N: usize>(a: &[f32; N], b: &[f32; N]) -> f32 {
    a.iter()
        .zip(b.iter())
        .fold(0.0, |dot, (&lhs, &rhs)| dot + lhs * rhs)
}
