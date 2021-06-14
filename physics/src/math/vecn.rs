/*
use core::{iter, ops::{Index, IndexMut}};

#[derive(Clone, Debug)]
pub struct VecN(Vec<f32>);

impl VecN {
    #[inline]
    pub fn from_size(size: usize) -> VecN {
        Self(iter::repeat(0.0).take(size).collect())
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.len()
    }

    pub fn dot(&self, rhs: &VecN) -> f32 {
        debug_assert!(self.len() == rhs.len());
        self.0.iter().zip(rhs.0.iter()).fold(0.0, |dot, (&lhs, &rhs)| dot + lhs * rhs)
    }
}

impl Index<usize> for VecN {
    type Output = f32;
    #[inline]
    fn index(&self, index: usize) -> &Self::Output {
        &self.0[index]
    }
}

impl IndexMut<usize> for VecN {
    #[inline]
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.0[index]
    }
}
*/
