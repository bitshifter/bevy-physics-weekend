use super::{dot, MatN, VecN};

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
