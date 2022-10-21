use crate::Vec3;
use core::f64::consts::PI;
use core::ops::{Add, AddAssign, Mul, Neg, Sub};

#[cfg(feature = "nanoserde")]
use nanoserde::{DeBin, SerBin};

#[derive(Default, Debug, Copy, Clone, PartialOrd)]
#[cfg_attr(feature = "nanoserde", derive(DeBin, SerBin))]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
#[cfg_attr(feature = "speedy", derive(speedy::Readable, speedy::Writable))]
pub struct Quaternion {
    pub scalar: f64,
    pub vector: Vec3,
}

impl Add for Quaternion {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        Self {
            scalar: self.scalar + other.scalar,
            vector: Vec3::new(
                self.vector.i + other.vector.i,
                self.vector.j + other.vector.j,
                self.vector.k + other.vector.k,
            ),
        }
    }
}

impl AddAssign for Quaternion {
    fn add_assign(&mut self, rhs: Self) {
        self.scalar += rhs.scalar;
        self.vector += rhs.vector;
    }
}

impl Sub for Quaternion {
    type Output = Self;
    fn sub(self, other: Self) -> Self {
        Self {
            scalar: self.scalar - other.scalar,
            vector: Vec3::new(
                self.vector.i - other.vector.i,
                self.vector.j - other.vector.j,
                self.vector.k - other.vector.k,
            ),
        }
    }
}

impl Mul for Quaternion {
    type Output = Self;
    fn mul(self, other: Self::Output) -> Self::Output {
        Self::Output {
            scalar: (self.scalar * other.scalar) - self.vector.dot(other.vector),
            vector: {
                (other.vector * self.scalar)
                    + (self.vector * other.scalar)
                    + self.vector.cross(other.vector)
            },
        }
    }
}

impl Mul<f64> for Quaternion {
    type Output = Self;
    fn mul(self, other: f64) -> Self {
        Self {
            scalar: self.scalar * other,
            vector: Vec3::new(
                self.vector.i * other,
                self.vector.j * other,
                self.vector.k * other,
            ),
        }
    }
}

impl Mul<&Vec3> for Quaternion {
    type Output = Vec3;
    fn mul(self, other: &Vec3) -> Vec3 {
        let t = self.vector.cross(*other) * 2.0;
        let cross = self.vector.cross(t);
        t * self.scalar + cross + *other
    }
}

impl Mul<Vec3> for Quaternion {
    type Output = Vec3;
    fn mul(self, other: Vec3) -> Vec3 {
        self * &other
    }
}

#[cfg(test)]
impl Mul<[f64; 16]> for Quaternion {
    type Output = Self;

    fn mul(self, rhs: [f64; 16]) -> Self::Output {
        let row = 0;
        let column = 0;
        //M11
        let m11q = rhs[row * 4 + column] * self.scalar;

        //M12
        let mut quat_vec_iter = self.vector.into_iter();
        let mut m12q = 0.0;
        for i in 0..3 {
            m12q += rhs[row * 4 + column + 1 + i] * quat_vec_iter.next().unwrap();
        }

        //M21
        let mut m21q: [f64; 3] = [0.0; 3];
        for i in 0..3 {
            m21q[i] += rhs[4 * (row + 1 + i) + column] * self.scalar;
        }

        //M22
        let mut m22q: [f64; 3] = [0.0; 3];
        for i in 0..3 {
            let mut quat_vec_iter = self.vector.into_iter();
            for j in 0..3 {
                let val = quat_vec_iter.next().unwrap();
                if row == 4 && column == 4 {}
                m22q[i] += rhs[4 * (row + 1 + i) + column + 1 + j] * val;
            }
        }
        Quaternion {
            scalar: m11q + m12q,
            vector: Vec3::new(m21q[0] + m22q[0], m21q[1] + m22q[1], m21q[2] + m22q[2]),
        }
    }
}

impl Mul<Quaternion> for f64 {
    type Output = Quaternion;
    fn mul(self, other: Quaternion) -> Quaternion {
        other * self
    }
}

impl Neg for Quaternion {
    type Output = Quaternion;

    fn neg(self) -> Self::Output {
        Self {
            scalar: -self.scalar,
            vector: -self.vector,
        }
    }
}

impl IntoIterator for Quaternion {
    type Item = f64;
    type IntoIter = std::array::IntoIter<f64, 4>;

    fn into_iter(self) -> Self::IntoIter {
        IntoIterator::into_iter([self.scalar, self.vector.i, self.vector.j, self.vector.k])
    }
}

impl PartialEq for Quaternion {
    fn eq(&self, other: &Self) -> bool {
        (self.scalar - other.scalar).abs() < 10.0 * f64::EPSILON && self.vector == other.vector
    }
}

impl Quaternion {
    pub fn unit() -> Quaternion {
        Quaternion {
            scalar: 1.0,
            vector: Vec3::default(),
        }
    }

    pub fn rotation_between(u: Vec3, v: Vec3) -> Quaternion {
        debug_assert!((u.norm() - 1.0).abs() < f64::EPSILON);
        debug_assert!((v.norm() - 1.0).abs() < f64::EPSILON);

        Quaternion {
            scalar: u.dot(v) + (u.norm_squared() * v.norm_squared()).sqrt(),
            vector: u.cross(v),
        }
        .normalized()
    }

    pub fn from_euler_angles(roll: f64, pitch: f64, yaw: f64) -> Quaternion {
        let (sr, cr) = (roll * 0.5f64).sin_cos();
        let (sp, cp) = (pitch * 0.5f64).sin_cos();
        let (sy, cy) = (yaw * 0.5f64).sin_cos();

        Quaternion {
            scalar: cr * cp * cy + sr * sp * sy,
            vector: Vec3 {
                i: sr * cp * cy - cr * sp * sy,
                j: cr * sp * cy + sr * cp * sy,
                k: cr * cp * sy - sr * sp * cy,
            },
        }
    }

    pub fn from_axis_angle(axis: Vec3, angle: f64) -> Quaternion {
        let s = (angle / 2.0).sin();
        let u = axis.unit();
        Quaternion {
            scalar: (angle / 2.0).cos(),
            vector: u * s,
        }
    }

    pub fn get_angle(&self) -> f64 {
        let w = self.scalar.abs();
        self.vector.norm().atan2(w) * 2.0
    }

    pub fn get_axis(&self) -> Vec3 {
        let v = if self.scalar >= f64::EPSILON {
            self.vector
        } else {
            -self.vector
        };

        v.unit()
    }

    pub fn get_axis_angle(&self) -> (Vec3, f64) {
        (self.get_axis(), self.get_angle())
    }

    pub fn get_heading(&self, forward: Vec3) -> Vec3 {
        //https://answers.unity.com/questions/525952/how-i-can-converting-a-quaternion-to-a-direction-v.html
        *self * forward
    }

    /// source is the forward vector for the space by default
    pub fn look_along(dir: Vec3, forward: Vec3, up: Vec3) -> Quaternion {
        let dest = dir.unit();
        let source = forward.unit();
        let dot = source.dot(dest);
        match dot {
            same if (same - 1.0).abs() < f32::EPSILON as f64 => {
                return Quaternion::unit();
            }
            opposite if (opposite + 1.0).abs() < f32::EPSILON as f64 => {
                return Quaternion::from_axis_angle(up, PI);
            }
            _ => (),
        }

        let rot_angle = dot.acos();
        let rot_axis = source.cross(dest).unit();

        Quaternion::from_axis_angle(rot_axis, rot_angle)
    }

    pub fn conjugate(self) -> Self {
        Self {
            scalar: self.scalar,
            vector: self.vector.invert(),
        }
    }

    pub fn dot(self, other: Self) -> Self {
        if self.scalar != 0.0 || other.scalar != 0.0 {
            0.5f64 * ((self.conjugate() * other) + (other.conjugate() * self))
        } else {
            Self {
                scalar: self.vector.dot(other.vector),
                vector: Vec3::default(),
            }
        }
    }

    pub fn cross(self, other: Self) -> Self {
        Self {
            scalar: 0.0,
            vector: other.vector * self.scalar
                + self.vector * other.scalar
                + self.vector.cross(other.vector),
        }
    }

    pub fn norm(self) -> f64 {
        (self.scalar.powi(2)
            + self.vector.i.powi(2)
            + self.vector.j.powi(2)
            + self.vector.k.powi(2))
        .sqrt()
    }

    // #[cfg(unused)]
    pub fn norm_squared(self) -> f64 {
        self.scalar.powi(2)
            + self.vector.i.powi(2)
            + self.vector.j.powi(2)
            + self.vector.k.powi(2)
    }

    pub fn normalized(self) -> Self {
        let norm = self.norm();
        debug_assert!(norm > 0.0);

        let ret = (1.0 / norm) * self;
        debug_assert!(ret.norm() - 1.0f64 < 10.0 * f64::EPSILON);
        ret
    }

    pub fn error(self, other: Self) -> Quaternion {
        other.conjugate() * self
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::test_shared::{FORWARD, UP};

    mod basic_ops_tests {
        use crate::test_shared::{gen_rand, gen_rand_mat, gen_rand_q, trans_mat};

        #[test]
        fn test_1() {
            let a = gen_rand_q();
            let b = gen_rand_q();
            let lhs = a.dot(b);
            let rhs = b.dot(a);
            pretty_assertions::assert_eq!(lhs, rhs);
        }

        #[test]
        fn test_2() {
            let a = gen_rand_q();
            let b = gen_rand_q();
            let c = gen_rand_q();
            let lhs = (a + b).dot(c);
            let rhs = a.dot(c) + b.dot(c);
            pretty_assertions::assert_eq!(lhs, rhs);

            let lhs = a.dot(b + c);
            let rhs = a.dot(b) + a.dot(c);
            pretty_assertions::assert_eq!(lhs, rhs);
        }

        #[test]
        fn test_3() {
            let a = gen_rand_q();
            let b = gen_rand_q();
            let c = gen_rand_q();
            let lhs = (a + b).cross(c);
            let rhs = a.cross(c) + b.cross(c);
            pretty_assertions::assert_eq!(lhs, rhs);

            let lhs = a.cross(b + c);
            let rhs = a.cross(b) + a.cross(c);
            pretty_assertions::assert_eq!(lhs, rhs);
        }

        #[test]
        fn test_4() {
            let a = gen_rand_q();
            let b = gen_rand_q();
            let c = gen_rand(None);
            let lhs = (c * a).dot(b);
            let rhs = a.dot(c * b);
            pretty_assertions::assert_eq!(lhs, rhs);

            let rhs = c * (a.dot(b));
            pretty_assertions::assert_eq!(lhs, rhs);
        }

        #[test]
        fn test_5() {
            let a = gen_rand_q();
            let b = gen_rand_q();
            let c = gen_rand(None);
            let lhs = (c * a).cross(b);
            let rhs = a.cross(c * b);
            pretty_assertions::assert_eq!(lhs, rhs);

            let rhs = c * (a.cross(b));
            pretty_assertions::assert_eq!(lhs, rhs);
        }

        #[test]
        fn test_6() {
            let a = gen_rand_q();
            let b = gen_rand_q();
            let lhs = (a * b).conjugate();
            let rhs = (b.conjugate()) * (a.conjugate());
            pretty_assertions::assert_eq!(lhs, rhs);
        }

        #[test]
        fn test_7() {
            let a = gen_rand_q();
            let b = gen_rand_q();
            let c = gen_rand_q();
            let lhs = a.dot(b * c);
            let rhs = b.dot(a * (c.conjugate()));
            pretty_assertions::assert_eq!(lhs, rhs);

            let rhs = c.dot(b.conjugate() * a);
            pretty_assertions::assert_eq!(lhs, rhs);
        }

        #[test]
        fn test_8() {
            let a = gen_rand_q();
            let lhs = a;
            let conj = a.conjugate();
            assert_ne!(a, conj);
            let rhs = conj.conjugate();
            pretty_assertions::assert_eq!(lhs, rhs);
        }

        #[test]
        fn test_9() {
            let a = gen_rand_q();
            let b = gen_rand_q();
            let lhs = a.conjugate().dot(b.conjugate());
            let rhs = a.dot(b);
            pretty_assertions::assert_eq!(lhs, rhs);
        }

        #[test]
        fn test_10() {
            let a = gen_rand_q();
            let b = gen_rand_q();
            let lhs = (a * b).norm();
            let rhs = a.norm() * (b.norm());
            assert!((lhs - rhs).abs() < 10.0 * f64::EPSILON);
        }

        #[test]
        fn test_11() {
            let a = gen_rand_q();
            let b = gen_rand_q();
            let mat = gen_rand_mat();
            let trans = trans_mat(&mat);
            let lhs = (a * mat).dot(b);
            let rhs = a.dot(b * trans);
            pretty_assertions::assert_eq!(lhs, rhs);
        }
    }

    mod orientation_tests {
        use crate::test_shared::gen_rand_vec;

        use super::*;

        #[test]
        fn test_lookat_from_forward() {
            for _ in 0..100 {
                let dir = gen_rand_vec().unit();
                let lookat = Quaternion::look_along(dir, FORWARD, UP);
                pretty_assertions::assert_eq!(lookat * FORWARD, dir.unit());
            }
        }

        #[test]
        fn test_lookat_from_arbitrary() {
            for _ in 0..100 {
                let start_dir = gen_rand_vec();
                let start_orient = Quaternion::look_along(start_dir, FORWARD, UP).normalized();
                let end_dir = gen_rand_vec().unit();
                let end_orient = Quaternion::look_along(end_dir, FORWARD, UP);
                let conversion = start_orient.error(end_orient);
                if (conversion * start_orient).get_heading(FORWARD) != end_dir {
                    pretty_assertions::assert_eq!(end_orient.get_heading(FORWARD), end_dir);
                }
            }
        }
    }
}