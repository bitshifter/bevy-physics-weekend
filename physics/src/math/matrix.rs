use super::{dot, VecN};
use core::ops::Mul;

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

#[test]
fn test_matmn() {
    let m0 = MatMN::from(&[[1., 2.], [3., 4.], [5., 6.]]);
    let m1 = m0.transpose();
    assert_eq!(m1, MatMN::from(&[[1., 3., 5.], [2., 4., 6.]]));
}
