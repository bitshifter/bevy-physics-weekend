use super::dot;
use core::ops::{Add, AddAssign, Deref, DerefMut, Mul, Sub};

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct VecN<const N: usize>(pub(crate) [f32; N]);

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
