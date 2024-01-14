#![cfg(any(feature = "approx", test))]

use std::f64::EPSILON;

use approx::AbsDiffEq;

use crate::{DualQuaternion, Quaternion, TaitBryan, Vec3};

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

impl AbsDiffEq for TaitBryan {
    type Epsilon = f64;

    fn default_epsilon() -> Self::Epsilon {
        EPSILON
    }

    fn abs_diff_eq(&self, other: &Self, epsilon: Self::Epsilon) -> bool {
        f64::abs_diff_eq(&self.roll, &other.roll, epsilon)
            && f64::abs_diff_eq(&self.pitch, &other.pitch, epsilon)
            && f64::abs_diff_eq(&self.yaw, &other.yaw, epsilon)
    }
}
