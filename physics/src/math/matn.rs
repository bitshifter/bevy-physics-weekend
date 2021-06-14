/*
use core::{iter, ops::{Mul, MulAssign}};

#[derive(Clone, Debug)]
pub struct MatN{
    data: Vec<f32>,
    dimension: usize,
}

impl MatN {
    pub fn zero(dimension: usize) -> MatN {
        let dimension_sq = dimension * dimension;
        MatN {
            data: iter::repeat(0.0).take(dimension_sq).collect(),
            dimension
        }
    }

    pub fn identity(dimension: usize) -> MatN {
        let dimension_sq = dimension * dimension;
        let mut data = Vec::with_capacity(dimension_sq);
        for i in 0..dimension {
            for j in 0..dimension {
                data.push(if i == j { 1.0 } else { 0.0 })
            }
        }
        MatN {
            data,
            dimension
        }
    }

    pub fn transpose(&self) -> MatN {
        let mut data = Vec::with_capacity(self.data.len());
        let dimension = self.dimension;
        for i in 0..dimension {
            for j in 0..dimension {
                data.push(self.data[j * dimension + i])
            }
        }
        MatN {
            data,
            dimension,
        }
    }

    fn col(&self, index: usize) -> &[f32] {
        let start = index * self.dimension;
        let end = start + self.dimension;
        &self.data[start..end]
    }
}

impl Mul<f32> for &MatN {
    type Output = MatN;
    fn mul(self, rhs: f32) -> MatN {
        let mut data = Vec::with_capacity(self.data.len());
        for &v in &self.data {
            data.push(v * rhs);
        }
        MatN {
            data,
            dimension: self.dimension,
        }
    }
}
pub fn lcp_gauss_seidel(a: &MatN
*/
