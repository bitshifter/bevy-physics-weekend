pub mod glam_ext;
mod matn;
mod vecn;

// pub use matn::MatN;
// pub use vecn::VecN;

use core::{
    iter,
    ops::{Index, IndexMut, Mul},
};

fn dot<const N: usize>(a: &[f32; N], b: &[f32; N]) -> f32 {
    a.iter()
        .zip(b.iter())
        .fold(0.0, |dot, (&lhs, &rhs)| dot + lhs * rhs)
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct VecNRef<'a, const N: usize>(&'a [f32; N]);

impl<'a, const N: usize> VecNRef<'a, N> {
    pub fn dot(&self, rhs: &Self) -> f32 {
        dot(&self.0, &rhs.0)
    }
}

impl<'a, const N: usize> From<&'a VecN<N>> for VecNRef<'a, N> {
    fn from(v: &'a VecN<N>) -> Self {
        VecNRef(&v.0)
    }
}

impl<'a, const N: usize> Index<usize> for VecNRef<'a, N> {
    type Output = f32;
    #[inline]
    fn index(&self, index: usize) -> &Self::Output {
        &self.0[index]
    }
}

impl<'a, const N: usize> IntoIterator for VecNRef<'a, N> {
    type Item = <core::slice::Iter<'a, f32> as IntoIterator>::Item;
    type IntoIter = <core::slice::Iter<'a, f32> as IntoIterator>::IntoIter;

    fn into_iter(self) -> Self::IntoIter {
        self.0.into_iter()
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct VecN<const N: usize>(Box<[f32; N]>);

impl<const N: usize> VecN<N> {
    #[inline]
    pub fn zero() -> Self {
        Self(Box::new([0.; N]))
    }

    pub fn dot(&self, rhs: &Self) -> f32 {
        dot(&self.0, &rhs.0)
    }
}

// impl<const N: usize> IntoIterator for VecN<N> {
//     type Item = f32;
//     type IntoIter = std::vec::IntoIter<Self::Item>;

//     fn into_iter(self) -> Self::IntoIter {
//         self.0.into_iter()
//     }
// }

impl<const N: usize> Index<usize> for VecN<N> {
    type Output = f32;
    #[inline]
    fn index(&self, index: usize) -> &Self::Output {
        &self.0[index]
    }
}

impl<const N: usize> IndexMut<usize> for VecN<N> {
    #[inline]
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.0[index]
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct MatN<const N: usize> {
    pub cols: Box<[[f32; N]; N]>,
}

impl<const N: usize> MatN<N> {
    pub fn zero() -> Self {
        MatN {
            cols: Box::new([[0.; N]; N]),
        }
    }

    pub fn identity() -> Self {
        let mut mat = MatN::zero();
        for i in 0..N {
            mat.cols[i][i] = 1.0;
        }
        mat
    }

    pub fn transpose(&self) -> Self {
        let mut mat = MatN::zero();
        for i in 0..N {
            for j in 0..N {
                mat.cols[j][i] = self.cols[i][j];
            }
        }
        mat
    }
}

// impl<const N: usize> Mul<f32> for &MatN<N> {
//     type Output = MatN<N>;
//     fn mul(self, rhs: f32) -> MatN<N> {
//         let mut data = Vec::with_capacity(N);
//         for &v in &self.0 {
//             data.push(v * rhs);
//         }
//         MatN(data)
//     }
// }

impl<const N: usize> From<MatMN<N, N>> for MatN<N> {
    fn from(m: MatMN<N, N>) -> Self {
        Self { cols: m.cols }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct MatMN<const M: usize, const N: usize> {
    pub cols: Box<[[f32; N]; M]>,
}

impl<const M: usize, const N: usize> MatMN<M, N> {
    pub fn zero() -> Self {
        MatMN {
            cols: Box::new([[0.; N]; M]),
        }
    }

    pub fn transpose(&self) -> MatMN<N, M> {
        let mut mat = MatMN::zero();
        for m in 0..M {
            for n in 0..N {
                mat.cols[n][m] = self.cols[m][n];
            }
        }
        mat
    }
}

impl<const M: usize, const N: usize> From<&[[f32; N]; M]> for MatMN<M, N> {
    fn from(a: &[[f32; N]; M]) -> MatMN<M, N> {
        MatMN { cols: Box::new(*a) }
    }
}

pub fn lcp_gauss_seidel<const N: usize>(a: &MatN<N>, b: &VecN<N>) -> VecN<N> {
    let mut x = VecN::zero();
    for _ in 0..N {
        for i in 0..N {
            let dx = (b[i] - dot(&a.cols[i], &x.0)) / a.cols[i][i];
            if dx.is_finite() {
                x[i] += dx;
            }
        }
    }
    x
}

#[test]
fn test_matmn() {
    let m0 = MatMN::from(&[[1., 2.], [3., 4.], [5., 6.]]);
    let m1 = m0.transpose();
    assert_eq!(m1, MatMN::from(&[[1., 3., 5.], [2., 4., 6.]]));
}
