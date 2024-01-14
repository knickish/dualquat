#![cfg(feature = "approx")]

use std::f64::EPSILON;

use approx::AbsDiffEq;

use crate::{DualQuaternion, Quaternion, Vec3};

impl AbsDiffEq for Vec3 {
    type Epsilon = f64;

    fn default_epsilon() -> Self::Epsilon {
        EPSILON
    }

    fn abs_diff_eq(&self, other: &Self, epsilon: Self::Epsilon) -> bool {
        f64::abs_diff_eq(&self.i, &other.i, epsilon)
            && f64::abs_diff_eq(&self.j, &other.j, epsilon)
            && f64::abs_diff_eq(&self.k, &other.k, epsilon)
    }
}

impl AbsDiffEq for Quaternion {
    type Epsilon = f64;

    fn default_epsilon() -> Self::Epsilon {
        EPSILON
    }

    fn abs_diff_eq(&self, other: &Self, epsilon: Self::Epsilon) -> bool {
        f64::abs_diff_eq(&self.scalar, &other.scalar, epsilon)
            && Vec3::abs_diff_eq(&self.vector, &other.vector, epsilon)
    }
}

impl AbsDiffEq for DualQuaternion {
    type Epsilon = f64;

    fn default_epsilon() -> Self::Epsilon {
        EPSILON
    }

    fn abs_diff_eq(&self, other: &Self, epsilon: Self::Epsilon) -> bool {
        Quaternion::abs_diff_eq(&self.real, &other.real, epsilon)
            && Quaternion::abs_diff_eq(&self.dual, &other.dual, epsilon)
    }
}
