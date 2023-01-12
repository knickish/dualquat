
use crate::{Quaternion, Vec3};
use core::{
    fmt::Display,
    ops::{Add, AddAssign, Mul, Neg, Sub},
};

#[cfg(feature = "nanoserde")]
use nanoserde::{DeBin, SerBin};

#[derive(Default, Debug, Copy, Clone, PartialEq, PartialOrd)]
#[cfg_attr(feature = "nanoserde", derive(DeBin, SerBin))]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
#[cfg_attr(feature = "speedy", derive(speedy::Readable, speedy::Writable))]
pub struct DualNumber {
    pub real: f64,
    pub dual: f64,
}

impl DualNumber {
    pub fn new(real: f64, dual: f64) -> Self {
        Self { real, dual }
    }
}

#[derive(Default, Debug, Copy, Clone, PartialEq, PartialOrd)]
#[cfg_attr(feature = "nanoserde", derive(DeBin, SerBin))]
#[cfg_attr(feature = "serde", derive(serde::Deserialize, serde::Serialize))]
#[cfg_attr(feature = "speedy", derive(speedy::Readable, speedy::Writable))]
pub struct DualQuaternion {
    pub real: Quaternion,
    pub dual: Quaternion,
}

impl Display for DualQuaternion {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f,
            "real: {{scalar: {}, i: {}, j: {}, k: {}}} dual: {{scalar: {}, i: {}, j: {}, k: {}}}",
            self.real.scalar,
            self.real.vector.i,
            self.real.vector.j,
            self.real.vector.k,
            self.dual.scalar,
            self.dual.vector.i,
            self.dual.vector.j,
            self.dual.vector.k
        )
    }
}

impl Add for DualQuaternion {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        Self {
            real: self.real + other.real,
            dual: self.dual + other.dual,
        }
    }
}

impl Neg for DualQuaternion {
    type Output = DualQuaternion;

    fn neg(self) -> Self::Output {
        Self {
            real: -self.real,
            dual: -self.dual,
        }
    }
}

impl AddAssign for DualQuaternion {
    fn add_assign(&mut self, rhs: Self) {
        self.real += rhs.real;
        self.dual += rhs.dual;
    }
}

impl Sub for DualQuaternion {
    type Output = Self;
    fn sub(self, other: Self) -> Self {
        Self {
            real: self.real - other.real,
            dual: self.dual - other.dual,
        }
    }
}

impl Mul for DualQuaternion {
    type Output = Self;
    fn mul(self, other: Self) -> Self {
        Self {
            real: self.real * other.real,
            dual: { (self.real * other.dual) + (self.dual * other.real) },
        }
    }
}

impl Mul<f64> for DualQuaternion {
    type Output = Self;
    fn mul(self, other: f64) -> Self {
        Self {
            real: self.real * other,
            dual: self.dual * other,
        }
    }
}

impl Mul<DualNumber> for DualQuaternion {
    type Output = Self;
    fn mul(self, other: DualNumber) -> Self {
        Self {
            real: self.real * other.real,
            dual: self.dual * other.dual,
        }
    }
}

impl Mul<f64> for &DualQuaternion {
    type Output = DualQuaternion;
    fn mul(self, other: f64) -> DualQuaternion {
        DualQuaternion {
            real: self.real * other,
            dual: self.dual * other,
        }
    }
}

impl Mul<DualQuaternion> for f64 {
    type Output = DualQuaternion;
    fn mul(self, other: DualQuaternion) -> DualQuaternion {
        other * self
    }
}

impl Mul<&DualQuaternion> for f64 {
    type Output = DualQuaternion;
    fn mul(self, other: &DualQuaternion) -> DualQuaternion {
        other * self
    }
}

impl IntoIterator for DualQuaternion {
    type Item = f64;
    type IntoIter = std::array::IntoIter<f64, 8>;

    fn into_iter(self) -> Self::IntoIter {
        IntoIterator::into_iter([
            self.real.scalar,
            self.real.vector.i,
            self.real.vector.j,
            self.real.vector.k,
            self.dual.scalar,
            self.dual.vector.i,
            self.dual.vector.j,
            self.dual.vector.k,
        ])
    }
}

impl DualQuaternion {
    pub fn unit() -> DualQuaternion {
        DualQuaternion {
            real: Quaternion::unit(),
            dual: Quaternion::default(),
        }
    }

    pub fn pose_from_location_tait_bryan(
        x: f64,
        y: f64,
        z: f64,
        roll: f64,
        pitch: f64,
        yaw: f64,
    ) -> DualQuaternion {
        // https://www.sedris.org/wg8home/Documents/WG80485.pdf
        let mut ret = DualQuaternion { real: Quaternion::from_euler_angles(roll, pitch, yaw), ..Default::default() };

        ret.encode_translation(Vec3::new(x, y, z));
        ret
    }

    pub fn pose_from_location_heading(
        x: f64,
        y: f64,
        z: f64,
        heading: Vec3,
        // forward: Vec3,
        // up: Vec3
    ) -> DualQuaternion {
        let heading = heading.unit();
        let yaw = (heading.j / heading.i).atan();
        let pitch = (heading.k).asin();
        DualQuaternion::pose_from_location_tait_bryan(x, y, z, 0.0, pitch, yaw)
    }

    pub fn location_from_pose(self) -> Vec3 {
        let tmp: Quaternion = 2.0 * (self.dual * (self.real.conjugate()));
        tmp.vector
    }

    pub fn from_attitude_location(attitude: Quaternion, location: Vec3) -> DualQuaternion {
        let mut ret = DualQuaternion {
            real: attitude,
            dual: Quaternion::default(),
        };
        ret.encode_translation(location);
        debug_assert_eq!(ret.location_from_pose(), location);
        ret
    }

    pub fn euler_angles_from_pose(self) -> Vec3 {
        //https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/index.htm
        let quat = self.real;
        let sqx: f64 = quat.vector.i * quat.vector.i;
        let sqy: f64 = quat.vector.j * quat.vector.j;
        let sqz: f64 = quat.vector.k * quat.vector.k;
        Vec3::new(
            (quat.vector.j * quat.vector.k + quat.vector.i * quat.scalar)
                .atan2(0.5 - (sqx + sqy)), //roll
            (-2.0 * (quat.vector.i * quat.vector.k - quat.vector.j * quat.scalar)).asin(), //pitch
            (quat.vector.i * quat.vector.j + quat.scalar * quat.vector.k)
                .atan2(0.5 - (sqy + sqz)), //yaw
        )
    }

    /// from the current location, find the necessary transformation to look towards a point
    pub fn lookat(&self, location: Vec3, forward: Vec3, up: Vec3) -> DualQuaternion {
        let current_location = self.location_from_pose();
        let desired_heading = location - current_location;
        debug_assert!(desired_heading.norm_squared() > f32::EPSILON as f64);
        let desired_heading = desired_heading.unit();
        let desired_orient = Quaternion::look_along(desired_heading, forward, up);
        debug_assert_eq!(desired_orient * forward, desired_heading);
        let mut target = DualQuaternion {
            real: desired_orient,
            ..Default::default()
        };
        target.encode_translation(current_location);
        debug_assert_eq!(target.real * forward, desired_heading);
        debug_assert_eq!(target.get_heading(forward), desired_heading);
        debug_assert_eq!(target.location_from_pose(), current_location);
        self.error(target)
    }

    pub fn get_heading(self, forward: Vec3) -> Vec3 {
        self.real.get_heading(forward)
    }

    pub fn conjugate(self) -> Self {
        Self {
            real: self.real.conjugate(),
            dual: self.dual.conjugate(),
        }
    }

    pub fn swap(self) -> Self {
        let tmp = self;
        Self {
            real: tmp.dual,
            dual: tmp.real,
        }
    }

    pub fn dot(self, other: Self) -> Self {
        0.5f64 * ((self.conjugate() * other) + (other.conjugate() * self))
    }

    pub fn cross(self, other: Self) -> Self {
        DualQuaternion {
            real: self.real.cross(other.real),
            dual: (self.dual.cross(other.real) + self.real.cross(other.dual)),
        }
    }

    fn circle(self, other: Self) -> Self {
        Self {
            real: self.real.dot(other.real) + self.dual.dot(other.dual),
            dual: Quaternion::default(),
        }
    }

    pub fn norm(self) -> f64 {
        self.circle(self).real.scalar
    }

    pub fn is_normalized(self) -> bool {
        let res = self.real.dot(self.real);
        let mut qr1 = true;
        res.into_iter().zip(Quaternion::unit()).for_each(|(x, y)| {
            if (x - y).abs() > f64::EPSILON {
                qr1 = false
            }
        });

        let res = self.real.dot(self.dual);
        let mut qrqd0 = true;
        res.into_iter()
            .zip(Quaternion::default())
            .for_each(|(x, y)| {
                if (x - y).abs() > f64::EPSILON {
                    qrqd0 = false
                }
            });

        qr1 && qrqd0
    }

    fn encode_translation(&mut self, trans: Vec3) {
        self.dual =
            0.5 * Quaternion {
                scalar: 0.0,
                vector: trans,
            } * self.real;
    }

    pub fn normalized(self) -> Self {
        let trans = self.location_from_pose();
        let mut ret = DualQuaternion {
            real: self.real.normalized(),
            dual: Quaternion::default(),
        };
        ret.encode_translation(trans);
        debug_assert!(ret.real.dot(ret.real) == Quaternion::unit());
        debug_assert!(ret.real.dot(ret.dual) == Quaternion::default());
        ret
    }

    pub fn interpolate(&self, other: &Self, balance: f64) -> Self {
        let real = self.real.slerp( other.real, balance);
        let self_loc =  self.location_from_pose();
        let diff = other.location_from_pose() - self_loc;
        let change = diff * balance.recip();
        DualQuaternion::from_attitude_location(real, self_loc + change)
    }   

    pub fn error(self, other: Self) -> Self {
        (self.conjugate() * other).normalized()
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::test_shared::{gen_rand, gen_rand_dq, Class};

    mod test_basic_operations {
        use std::f64::consts::{FRAC_PI_2, FRAC_PI_3};

        use super::*;
        use pretty_assertions::assert_eq;

        #[test]
        fn test_cross_product() {
            let lhs = DualQuaternion {
                real: Quaternion {
                    scalar: 0.0,
                    vector: Vec3::new(0.0, 0.0, 0.0),
                },
                dual: Quaternion {
                    scalar: 0.0,
                    vector: Vec3::new(-0.8624439953418593, 7657.01018772304, 0.0),
                },
            };
            let rhs = DualQuaternion {
                real: Quaternion {
                    scalar: 0.0,
                    vector: Vec3::new(-383455.53698892076, 3404421584.6144795, 0.0),
                },
                dual: Quaternion {
                    scalar: 0.0,
                    vector: Vec3::new(0.0, 0.0, 0.0),
                },
            };
            let desired_result = DualQuaternion {
                real: Quaternion {
                    scalar: 0.0,
                    vector: Vec3::new(0.0, 0.0, 0.0),
                },
                dual: Quaternion {
                    scalar: 0.0,
                    vector: Vec3::new(0.0, 0.0, -4.76837158203125e-7),
                },
            };
            let actual_result = lhs.cross(rhs);
            assert_eq!(actual_result, desired_result);
        }

        #[test]
        fn test_interpolate() {
            let first = DualQuaternion::pose_from_location_tait_bryan(0.0, 0.0, 0.0, 0.0, 0.0, FRAC_PI_2);
            let second = DualQuaternion::pose_from_location_tait_bryan(0.0, 0.0, 0.0, 0.0, 0.0, FRAC_PI_2 + FRAC_PI_3);
            let interpolated_mid = DualQuaternion::pose_from_location_tait_bryan(0.0, 0.0, 0.0, 0.0, 0.0, FRAC_PI_2 + FRAC_PI_3/2.0).normalized();
            let actual_mid = first.interpolate(&second, 0.5).normalized();
            assert_eq!(interpolated_mid, actual_mid);
        }

        #[test]
        fn test_swap() {
            let a = gen_rand_dq(None);
            let a2 = a.swap();
            assert_eq!(a.real, a2.dual);
            assert_eq!(a.dual, a2.real);
            let a3 = a2.swap();
            assert_eq!(a, a3);
        }

        #[test]
        fn test_23() {
            let a = gen_rand_dq(None);
            let b = gen_rand_dq(None);
            let ab = a.dot(b);
            let ba = b.dot(a);
            assert_eq!(ab, ba);
        }

        #[test]
        fn test_24() {
            let a = gen_rand_dq(None);
            let b = gen_rand_dq(None);
            let ab = a.circle(b);
            let ba = b.circle(a);
            assert_eq!(ab, ba);
        }

        #[test]
        fn test_25() {
            let a = gen_rand_dq(None);
            let b = gen_rand_dq(None);
            let c = gen_rand_dq(None);
            let lhs = (a + b).dot(c);
            let rhs = a.dot(c) + b.dot(c);
            assert_eq!(lhs, rhs);

            let lhs = a.dot(b + c);
            let rhs = a.dot(b) + a.dot(c);
            assert_eq!(lhs, rhs);
        }

        #[test]
        fn test_26() {
            let a = gen_rand_dq(None);
            let b = gen_rand_dq(None);
            let c = gen_rand_dq(None);
            let lhs = (a + b).circle(c);
            let rhs = a.circle(c) + b.circle(c);
            assert_eq!(lhs, rhs);

            let lhs = a.circle(b + c);
            let rhs = a.circle(b) + a.circle(c);
            assert_eq!(lhs, rhs);
        }

        #[test]
        fn test_27() {
            let a = gen_rand_dq(None);
            let b = gen_rand_dq(None);
            let c = gen_rand_dq(None);
            let lhs = (a + b).cross(c);
            let rhs = a.cross(c) + b.cross(c);
            assert_eq!(lhs, rhs);

            let lhs = a.cross(b + c);
            let rhs = a.cross(b) + a.cross(c);
            assert_eq!(lhs, rhs);
        }

        #[test]
        fn test_28() {
            let a = gen_rand_dq(None);
            let b = gen_rand_dq(None);
            let c = gen_rand(None);
            let lhs = (c * a).dot(b);
            let rhs = c * (a.dot(b));
            assert_eq!(lhs, rhs);
        }

        #[test]
        fn test_29() {
            let a = gen_rand_dq(None);
            let b = gen_rand_dq(None);
            let c = gen_rand(None);
            let lhs = (c * a).cross(b);
            let rhs = c * (a.cross(b));
            assert_eq!(lhs, rhs);
        }

        #[test]
        fn test_30() {
            let a = gen_rand_dq(None);
            let b = gen_rand_dq(None);
            let c = gen_rand(None);
            let lhs = (c * a).circle(b);
            let rhs = c * (a.circle(b));
            assert_eq!(lhs, rhs);
        }

        #[test]
        fn test_31() {
            let a = gen_rand_dq(None);
            let b = gen_rand_dq(None);
            let lhs = (a * b).conjugate();
            let rhs = b.conjugate() * (a.conjugate());
            assert_eq!(lhs, rhs);
        }

        #[test]
        fn test_32() {
            let a = gen_rand_dq(None);
            let b = gen_rand_dq(None);
            let c = gen_rand_dq(None);
            let lhs = a.dot(b * c);
            let rhs = c.dot(b.conjugate() * a);
            assert_eq!(lhs, rhs);
        }

        #[test]
        fn test_33() {
            let a = gen_rand_dq(None);
            let b = gen_rand_dq(None);
            let c = gen_rand_dq(None);
            let lhs = a.circle(b * c);
            let mid = b.swap().circle((a.swap()) * (c.conjugate()));
            let rhs = c.swap().circle(b.conjugate() * (a.swap()));
            assert_eq!(lhs, mid);
            assert_eq!(mid, rhs);
        }

        #[test]
        fn test_34() {
            let a = gen_rand_dq(None);
            let lhs = a;
            let rhs = a.conjugate().conjugate();
            assert_eq!(lhs, rhs);
        }

        #[test]
        fn test_35() {
            let a = gen_rand_dq(None);
            let b = gen_rand_dq(None);
            let lhs = a.conjugate().dot(b.conjugate());
            let rhs = a.dot(b);
            assert_eq!(lhs, rhs);
        }

        #[test]
        fn test_36() {
            let a = gen_rand_dq(None);
            let b = gen_rand_dq(None);
            let lhs = a.conjugate().circle(b.conjugate());
            let rhs = a.circle(b);
            assert_eq!(lhs, rhs);
        }

        #[test]
        fn test_37() {
            let a = gen_rand_dq(None);
            let b = gen_rand_dq(None);
            let lhs = (a.swap()).circle(b.swap());
            let rhs = a.circle(b);
            assert_eq!(lhs, rhs);
        }

        #[test]
        fn test_61() {
            let a = gen_rand_dq(None);
            let res = a.error(a);
            assert!(res == DualQuaternion::unit());
        }

        #[test]
        fn test_error() {
            let a = gen_rand_dq(Some(Class::Unit));
            let b = gen_rand_dq(Some(Class::Unit));
            let err = a.error(b);
            let res = a * err;
            assert_eq!(res, b);
        }
    }

    mod orientation_tests {
        use super::*;
        use crate::test_shared::{eps_equal, gen_rand_vec, FORWARD, UP};

        #[test]
        fn test_lookat_from_arbitrary() {
            let eps = f32::EPSILON as f64;

            for _ in 0..10000 {
                let start_dir = gen_rand_vec();
                let start_loc = gen_rand_vec();
                let start_orient = Quaternion::look_along(start_dir, FORWARD, UP);
                let start = DualQuaternion::from_attitude_location(start_orient, start_loc);
                let lookat_location = {
                    let mut lookat_location = gen_rand_vec();
                    while eps_equal(lookat_location.into_iter(), start_loc.into_iter(), eps) {
                        lookat_location = gen_rand_vec();
                    }
                    lookat_location
                };
                let end_heading = (lookat_location - start_loc).unit();
                let conversion = start.lookat(lookat_location, FORWARD, UP);
                let final_dual_pose = start * conversion;

                assert!(eps_equal(
                    start_loc.into_iter(),
                    final_dual_pose.location_from_pose().into_iter(),
                    eps
                ));
                assert!(eps_equal(
                    end_heading.into_iter(),
                    final_dual_pose.get_heading(FORWARD).into_iter(),
                    eps
                ));
            }
        }
    }
}