#![allow(dead_code)]
pub mod glam_ext;
mod matn;
mod vecn;

// pub use matn::MatN;
// pub use vecn::VecN;

use core::ops::{Add, AddAssign, Deref, DerefMut, Mul, Sub};

fn dot<const N: usize>(a: &[f32; N], b: &[f32; N]) -> f32 {
    a.iter()
        .zip(b.iter())
        .fold(0.0, |dot, (&lhs, &rhs)| dot + lhs * rhs)
}

// #[derive(Copy, Clone, Debug, PartialEq)]
// pub struct VecNRef<'a, const N: usize>(&'a [f32; N]);

// impl<'a, const N: usize> VecNRef<'a, N> {
//     pub fn dot(&self, rhs: &Self) -> f32 {
//         dot(&self.0, &rhs.0)
//     }
// }

// impl<'a, const N: usize> From<&'a VecN<N>> for VecNRef<'a, N> {
//     fn from(v: &'a VecN<N>) -> Self {
//         VecNRef(&v.0)
//     }
// }

// impl<'a, const N: usize> Index<usize> for VecNRef<'a, N> {
//     type Output = f32;
//     #[inline]
//     fn index(&self, index: usize) -> &Self::Output {
//         &self.0[index]
//     }
// }

// impl<'a, const N: usize> IntoIterator for VecNRef<'a, N> {
//     type Item = <core::slice::Iter<'a, f32> as IntoIterator>::Item;
//     type IntoIter = <core::slice::Iter<'a, f32> as IntoIterator>::IntoIter;

//     fn into_iter(self) -> Self::IntoIter {
//         self.0.into_iter()
//     }
// }

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct VecN<const N: usize>([f32; N]);

impl<const N: usize> VecN<N> {
    #[inline]
    pub const fn zero() -> Self {
        Self([0.; N])
    }

    #[inline]
    pub fn dot(&self, rhs: &Self) -> f32 {
        dot(&self.0, &rhs.0)
    }
}

impl<const N: usize> Default for VecN<N> {
    fn default() -> Self {
        Self::zero()
    }
}

impl<const N: usize> Deref for VecN<N> {
    type Target = [f32; N];
    #[inline]
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<const N: usize> DerefMut for VecN<N> {
    #[inline]
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl<const N: usize> Add<VecN<N>> for VecN<N> {
    type Output = VecN<N>;
    #[inline]
    fn add(self, rhs: VecN<N>) -> Self::Output {
        let mut tmp = self;
        for n in 0..N {
            tmp[n] += rhs[n];
        }
        tmp
    }
}

impl<const N: usize> Mul<f32> for VecN<N> {
    type Output = VecN<N>;
    #[inline]
    fn mul(self, rhs: f32) -> Self::Output {
        let mut tmp = self;
        for n in 0..N {
            tmp[n] *= rhs;
        }
        tmp
    }
}

impl<const N: usize> Sub<VecN<N>> for VecN<N> {
    type Output = VecN<N>;
    #[inline]
    fn sub(self, rhs: VecN<N>) -> Self::Output {
        let mut tmp = self;
        for n in 0..N {
            tmp[n] -= rhs[n];
        }
        tmp
    }
}

impl<const N: usize> AddAssign<VecN<N>> for VecN<N> {
    #[inline]
    fn add_assign(&mut self, rhs: VecN<N>) {
        for n in 0..N {
            self[n] += rhs[n];
        }
    }
}

// impl<const N: usize> IntoIterator for VecN<N> {
//     type Item = f32;
//     type IntoIter = std::vec::IntoIter<Self::Item>;

//     fn into_iter(self) -> Self::IntoIter {
//         self.0.into_iter()
//     }
// }

// impl<const N: usize> Index<usize> for VecN<N> {
//     type Output = f32;
//     #[inline]
//     fn index(&self, index: usize) -> &Self::Output {
//         &self.0[index]
//     }
// }

// impl<const N: usize> IndexMut<usize> for VecN<N> {
//     #[inline]
//     fn index_mut(&mut self, index: usize) -> &mut Self::Output {
//         &mut self.0[index]
//     }
// }

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct MatN<const N: usize> {
    pub rows: [VecN<N>; N],
}

impl<const N: usize> MatN<N> {
    #[inline]
    pub const fn zero() -> Self {
        MatN {
            rows: ([VecN::zero(); N]),
        }
    }

    pub fn identity() -> Self {
        let mut mat = MatN::zero();
        for i in 0..N {
            mat.rows[i][i] = 1.0;
        }
        mat
    }

    pub fn transpose(&self) -> Self {
        let mut mat = MatN::zero();
        for i in 0..N {
            for j in 0..N {
                mat.rows[j][i] = self.rows[i][j];
            }
        }
        mat
    }
}

impl<const N: usize> Default for MatN<N> {
    fn default() -> Self {
        Self::zero()
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
    #[inline]
    fn from(m: MatMN<N, N>) -> Self {
        Self { rows: m.rows }
    }
}

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct MatMN<const M: usize, const N: usize> {
    pub rows: [VecN<N>; M],
}

impl<const M: usize, const N: usize> MatMN<M, N> {
    #[inline]
    pub const fn zero() -> Self {
        MatMN {
            rows: [VecN::zero(); M],
        }
    }

    #[inline]
    pub fn transpose(&self) -> MatMN<N, M> {
        let mut mat = MatMN::zero();
        for m in 0..M {
            for n in 0..N {
                mat.rows[n][m] = self.rows[m][n];
            }
        }
        mat
    }
}

impl<const M: usize, const N: usize> Default for MatMN<M, N> {
    fn default() -> Self {
        Self::zero()
    }
}

// impl<const M: usize, const N: usize> From<[[f32; N]; M]> for MatMN<M, N> {
//     #[inline]
//     fn from(a: [[f32; N]; M]) -> MatMN<M, N> {
//         MatMN { rows: a }
//     }
// }

impl<const M: usize, const N: usize> Mul<VecN<N>> for MatMN<M, N> {
    type Output = VecN<M>;
    #[inline]
    fn mul(self, rhs: VecN<N>) -> Self::Output {
        let mut tmp = VecN::zero();
        for m in 0..M {
            tmp[m] = dot(&rhs, &self.rows[m]);
        }
        tmp
    }
}

impl<const M: usize, const P: usize, const N: usize> Mul<MatMN<P, N>> for MatMN<M, P> {
    type Output = MatMN<M, N>;
    #[inline]
    fn mul(self, rhs: MatMN<P, N>) -> Self::Output {
        let rhs_transpose = rhs.transpose();

        let mut tmp = MatMN::zero();
        for m in 0..M {
            for n in 0..N {
                tmp.rows[m][n] = dot(&self.rows[m], &rhs_transpose.rows[n]);
            }
        }
        tmp
    }
}

pub fn lcp_gauss_seidel<const N: usize>(a: &MatN<N>, b: &VecN<N>) -> VecN<N> {
    let mut x = VecN::zero();
    for _ in 0..N {
        for i in 0..N {
            let dx = (b[i] - dot(&a.rows[i], &x.0)) / a.rows[i][i];
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
