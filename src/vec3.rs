use core::ops::{Add, AddAssign, Div, Mul, Neg, Sub};
#[cfg(feature = "nanoserde")]
use nanoserde::{DeBin, SerBin};

#[derive(Default, Debug, Copy, Clone, PartialOrd)]
#[cfg_attr(feature = "nanoserde", derive(nanoserde::DeBin, nanoserde::SerBin))]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
#[cfg_attr(feature = "speedy", derive(speedy::Readable, speedy::Writable))]
pub struct Vec3 {
    pub i: f64,
    pub j: f64,
    pub k: f64,
}

impl Neg for Vec3 {
    type Output = Vec3;

    fn neg(self) -> Self::Output {
        Vec3 {
            i: -self.i,
            j: -self.j,
            k: -self.k,
        }
    }
}

impl Add for Vec3 {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        Self {
            i: self.i + other.i,
            j: self.j + other.j,
            k: self.k + other.k,
        }
    }
}

impl AddAssign for Vec3 {
    fn add_assign(&mut self, rhs: Self) {
        self.i += rhs.i;
        self.j += rhs.j;
        self.k += rhs.k;
    }
}

impl Sub for Vec3 {
    type Output = Self;
    fn sub(self, other: Self) -> Self {
        Self {
            i: self.i - other.i,
            j: self.j - other.j,
            k: self.k - other.k,
        }
    }
}

impl Mul<f64> for Vec3 {
    type Output = Self;
    fn mul(self, other: f64) -> Self {
        Self {
            i: self.i * other,
            j: self.j * other,
            k: self.k * other,
        }
    }
}

impl Div<f64> for Vec3 {
    type Output = Self;
    fn div(self, other: f64) -> Self {
        Self {
            i: self.i / other,
            j: self.j / other,
            k: self.k / other,
        }
    }
}

impl IntoIterator for Vec3 {
    type Item = f64;
    type IntoIter = std::array::IntoIter<f64, 3>;

    fn into_iter(self) -> Self::IntoIter {
        IntoIterator::into_iter([self.i, self.j, self.k])
    }
}

impl PartialEq for Vec3 {
    fn eq(&self, other: &Self) -> bool {
        (self.i - other.i).abs() < 10.0 * f64::EPSILON
            && (self.j - other.j).abs() < 10.0 * f64::EPSILON
            && (self.k - other.k).abs() < 10.0 * f64::EPSILON
    }
}

impl Vec3 {
    #[inline(always)]
    pub const fn new(x: f64, y: f64, z: f64) -> Vec3 {
        Vec3 { i: x, j: y, k: z }
    }

    pub fn invert(self) -> Vec3 {
        Vec3 {
            i: -(self.i),
            j: -(self.j),
            k: -(self.k),
        }
    }

    pub fn sum(&self) -> f64 {
        self.i + self.j + self.k
    }

    pub fn distance(&self, second_location: Vec3) -> f64 {
        ((self.i - second_location.i).powi(2)
            + (self.k - second_location.k).powi(2)
            + (self.j - second_location.j).powi(2))
        .sqrt()
    }

    pub fn unit(&self) -> Vec3 {
        let norm = self.norm();
        *self * (1.0 / norm)
    }

    pub fn norm(&self) -> f64 {
        ((self.i).powi(2) + (self.k).powi(2) + (self.j).powi(2)).sqrt()
    }

    pub fn norm_squared(&self) -> f64 {
        (self.i).powi(2) + (self.k).powi(2) + (self.j).powi(2)
    }

    pub fn cross(self, other: Self) -> Self {
        Vec3 {
            i: (self.j * other.k) - (self.k * other.j),
            j: (self.k * other.i) - (self.i * other.k),
            k: (self.i * other.j) - (self.j * other.i),
        }
    }

    pub fn dot(self, other: Self) -> f64 {
        (self.i * other.i) + (self.j * other.j) + (self.k * other.k)
    }

    pub fn between(self, start: Self, end: Self) -> bool {
        (self.distance(start) + self.distance(end) - start.distance(end)).abs() < f64::EPSILON
    }

    pub fn splat(value: f64) -> Self {
        Self {
            i: value,
            j: value,
            k: value,
        }
    }

    #[cfg(test)]
    fn exact_eq(self, other: Self) -> bool {
        (self.i == other.i) && (self.j == other.j) && (self.k == other.k)
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_cross_product_self() {
        let l = Vec3::new(2.0, 3.0, 4.0);
        assert!(l.cross(l).exact_eq(Vec3::default()));
    }

    #[test]
    fn test_cross_product_other() {
        let l = Vec3::new(1.0, 2.0, 3.0);
        let r = Vec3::new(4.0, 5.0, 6.0);
        assert!(l.cross(r).exact_eq(Vec3::new(-3.0, 6.0, -3.0)));
    }

    #[test]
    fn test_dot_product() {
        let l = Vec3::new(3.0, 5.0, 8.0);
        let r = Vec3::new(2.0, 7.0, 1.0);
        assert!(l.dot(r) == 49.0)
    }

    #[test]
    fn test_invert() {
        let l = Vec3::new(3.0, 5.0, 8.0);
        let r = l.invert();
        assert_ne!(l, r);
        let r = r.invert();
        assert_eq!(l, r);
    }
}
