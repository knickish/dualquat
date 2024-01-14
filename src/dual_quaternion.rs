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
        write!(f, "real: {{{}}} dual: {{{}}}", self.real, self.dual,)
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

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TaitBryan {
    pub roll: f64,
    pub pitch: f64,
    pub yaw: f64,
}

impl DualQuaternion {
    pub fn unit() -> DualQuaternion {
        DualQuaternion {
            real: Quaternion::unit(),
            dual: Quaternion::default(),
        }
    }

    pub fn from_location_tait_bryan(location: Vec3, tb: TaitBryan) -> DualQuaternion {
        // https://www.sedris.org/wg8home/Documents/WG80485.pdf
        let mut ret = DualQuaternion {
            real: Quaternion::from_tait_bryan(tb),
            ..Default::default()
        };

        ret.encode_translation(location);
        ret
    }

    pub fn from_location_heading(
        location: Vec3,
        heading: Vec3,
        // forward: Vec3,
        // up: Vec3
    ) -> DualQuaternion {
        let heading = heading.unit();
        let yaw = (heading.j / heading.i).atan();
        let pitch = (heading.k).asin();
        DualQuaternion::from_location_tait_bryan(
            location,
            TaitBryan {
                roll: 0.0,
                pitch,
                yaw,
            },
        )
    }

    pub const fn from_rotation(rotation: Quaternion) -> Self {
        DualQuaternion {
            real: rotation,
            dual: Quaternion::zero(),
        }
    }

    pub fn from_rotation_translation(rotation: Quaternion, translation: Vec3) -> DualQuaternion {
        let mut ret = DualQuaternion {
            real: rotation,
            dual: Quaternion::default(),
        };
        ret.encode_translation(translation);
        debug_assert_eq!(ret.to_translation(), translation);
        ret
    }

    pub fn to_translation(self) -> Vec3 {
        let tmp: Quaternion = 2.0 * (self.dual * (self.real.conjugate()));
        tmp.vector
    }

    pub fn to_rotation_translation(self) -> (Quaternion, Vec3) {
        (self.real, self.to_translation())
    }

    /// Apply another dual quaternion as a transformation
    pub fn transform_by(self, transform: Self) -> Self {
        transform * self * transform.conjugate()
    }

    /// from the current location, find the necessary transformation to look towards a point
    pub fn lookat(&self, location: Vec3, forward: Vec3, up: Vec3) -> DualQuaternion {
        let current_location = self.to_translation();
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
        debug_assert_eq!(target.to_translation(), current_location);
        self.error(target)
    }

    pub fn get_heading(self, forward: Vec3) -> Vec3 {
        self.real.get_heading(forward)
    }

    pub fn relative_to(self, other: Self) -> Self {
        other.conjugate() * self
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

    /// translate self using using frame coordinates
    pub fn translate_absolute(self, trans: Vec3) -> Self {
        Self {
            real: Quaternion::unit(),
            dual: Quaternion {
                scalar: 0.0,
                vector: trans * 0.5,
            } * Quaternion::unit(),
        } * self
    }

    /// translate self using body coordinates
    pub fn translate_relative(self, trans: Vec3) -> Self {
        Self {
            real: self.real,
            dual: Quaternion {
                scalar: 0.0,
                vector: trans * 0.5,
            } * self.real,
        } * self
    }

    #[cfg_attr(debug_assertions, track_caller)]
    pub fn normalized(self) -> Self {
        let trans = self.to_translation();
        let mut ret = DualQuaternion {
            real: self.real.normalized(),
            dual: Quaternion::default(),
        };
        ret.encode_translation(trans);
        ret
    }

    #[cfg(test)]
    #[cfg_attr(debug_assertions, track_caller)]
    pub fn normalized_match_sign(self) -> Self {
        match self.normalized() {
            neg if neg.real.scalar.is_sign_negative() => neg.neg(),
            leave => leave,
        }
    }

    pub fn interpolate(&self, other: &Self, balance: f64) -> Self {
        let real = self.real.slerp(other.real, balance);
        let self_loc = self.to_translation();
        let diff = other.to_translation() - self_loc;
        let change = diff * balance.recip();
        DualQuaternion::from_rotation_translation(real, self_loc + change)
    }

    pub fn error(self, other: Self) -> Self {
        (self.conjugate() * other).normalized()
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::test_shared::{gen_rand, gen_rand_dq, Class, FORWARD, UP};

    fn backwards() -> Quaternion {
        Quaternion::look_along(
            Vec3 {
                i: -1.0,
                j: 0.0,
                k: 0.0,
            },
            FORWARD,
            UP,
        )
    }

    mod test_basic_operations {
        use std::f64::consts::{FRAC_PI_2, FRAC_PI_3};

        use crate::test_shared::FORWARD;

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
            let first = DualQuaternion::from_location_tait_bryan(
                Vec3::new(0.0, 0.0, 0.0),
                TaitBryan {
                    roll: 0.0,
                    pitch: 0.0,
                    yaw: FRAC_PI_2,
                },
            );
            let second = DualQuaternion::from_location_tait_bryan(
                Vec3::new(0.0, 0.0, 0.0),
                TaitBryan {
                    roll: 0.0,
                    pitch: 0.0,
                    yaw: FRAC_PI_2 + FRAC_PI_3,
                },
            );
            let interpolated_mid = DualQuaternion::from_location_tait_bryan(
                Vec3::new(0.0, 0.0, 0.0),
                TaitBryan {
                    roll: 0.0,
                    pitch: 0.0,
                    yaw: FRAC_PI_2 + (FRAC_PI_3 / 2.0),
                },
            )
            .normalized();
            let actual_mid = first.interpolate(&second, 0.5).normalized();
            assert_eq!(
                interpolated_mid.get_heading(FORWARD),
                actual_mid.get_heading(FORWARD)
            );
        }

        #[test]
        fn test_relative_position() {
            {
                let first = DualQuaternion::from_rotation_translation(
                    Quaternion::unit(),
                    Vec3::new(10.0, 20.0, 30.0),
                );
                let second = DualQuaternion::from_rotation_translation(
                    Quaternion::unit(),
                    Vec3::new(5.0, 10.0, 15.0),
                );
                let second_relative_to_first = second.relative_to(first).normalized();
                let actual_relative = DualQuaternion::from_rotation_translation(
                    Quaternion::unit(),
                    Vec3::new(-5.0, -10.0, -15.0),
                )
                .normalized();
                dbg!(second_relative_to_first.to_translation());
                assert_eq!(second_relative_to_first, actual_relative);
            }

            {
                let second = DualQuaternion::from_rotation_translation(
                    Quaternion::unit(),
                    Vec3::new(10.0, 20.0, 30.0),
                );
                let first = DualQuaternion::from_rotation_translation(
                    Quaternion::unit(),
                    Vec3::new(5.0, 10.0, 15.0),
                );
                let second_relative_to_first = second.relative_to(first).normalized();
                let actual_relative = DualQuaternion::from_rotation_translation(
                    Quaternion::unit(),
                    Vec3::new(5.0, 10.0, 15.0),
                )
                .normalized();
                dbg!(second_relative_to_first.to_translation());
                assert_eq!(second_relative_to_first, actual_relative);
            }
        }

        #[test]
        fn test_relative_change() {
            {
                let first = DualQuaternion::from_rotation_translation(
                    Quaternion::unit(),
                    Vec3::new(10.0, 20.0, 30.0),
                );
                let second = DualQuaternion::from_rotation_translation(
                    Quaternion::unit(),
                    Vec3::new(5.0, 10.0, 15.0),
                );
                let second_relative_to_first = second.relative_to(first).normalized();
                let actual_relative = DualQuaternion::from_rotation_translation(
                    Quaternion::unit(),
                    Vec3::new(-5.0, -10.0, -15.0),
                )
                .normalized();
                dbg!(second_relative_to_first.to_translation());
                assert_eq!(second_relative_to_first, actual_relative);
            }

            {
                let first = DualQuaternion::from_rotation_translation(
                    backwards(),
                    Vec3::new(10.0, 20.0, 30.0),
                );
                let second = DualQuaternion::from_rotation_translation(
                    Quaternion::unit(),
                    Vec3::new(5.0, 10.0, 15.0),
                );

                assert_eq!(
                    first.get_heading(FORWARD),
                    Vec3 {
                        i: -1.0,
                        j: 0.0,
                        k: 0.0
                    }
                );

                let second_relative_to_first = second.relative_to(first).normalized();
                let actual_relative = DualQuaternion::from_rotation_translation(
                    backwards(),
                    Vec3::new(5.0, 10.0, 15.0).invert(),
                )
                .normalized();
                dbg!(second_relative_to_first.to_translation());
                assert_eq!(second_relative_to_first, actual_relative);
            }
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
        use std::f64::consts::{FRAC_2_PI, PI};

        use super::*;
        use crate::test_shared::{eps_equal, gen_rand_vec, FORWARD, RIGHT, UP};

        #[test]
        fn test_lookat_from_arbitrary() {
            let eps = f32::EPSILON as f64;

            for _ in 0..10000 {
                let start_dir = gen_rand_vec();
                let start_loc = gen_rand_vec();
                let start_orient = Quaternion::look_along(start_dir, FORWARD, UP);
                let start = DualQuaternion::from_rotation_translation(start_orient, start_loc);
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
                    final_dual_pose.to_translation().into_iter(),
                    eps
                ));
                assert!(eps_equal(
                    end_heading.into_iter(),
                    final_dual_pose.get_heading(FORWARD).into_iter(),
                    eps
                ));
            }
        }

        #[test]
        fn test_full_rotate() {
            let eps = f32::EPSILON as f64;

            let start_loc = Vec3::default();
            let start_orient = Quaternion::look_along(FORWARD, FORWARD, UP);
            let start = DualQuaternion::from_rotation_translation(start_orient, start_loc);

            let first_rotate =
                DualQuaternion::from_rotation(Quaternion::from_axis_angle(UP, FRAC_2_PI));

            assert!(
                eps_equal(start.transform_by(first_rotate), start, eps),
                "\nrotate: {},\nstart: {},\nrotated: {}",
                first_rotate,
                start,
                start.transform_by(first_rotate)
            );

            let second_rotate =
                DualQuaternion::from_rotation(Quaternion::from_axis_angle(RIGHT, FRAC_2_PI));

            assert!(
                eps_equal(start.transform_by(second_rotate), start, eps),
                "\nrotate: {},\nstart: {},\nrotated: {}",
                second_rotate,
                start,
                start.transform_by(second_rotate)
            );

            let third_rotate =
                DualQuaternion::from_rotation(Quaternion::from_axis_angle(RIGHT, FRAC_2_PI));

            assert!(
                eps_equal(start.transform_by(third_rotate), start, eps),
                "\nrotate: {},\nstart: {},\nrotated: {}",
                third_rotate,
                start,
                third_rotate
            );
        }

        #[test]
        fn test_full_rotate_by_parts() {
            let eps = f32::EPSILON as f64 * 10.0;

            let start_loc = gen_rand_vec();
            let start_orient = Quaternion::look_along(FORWARD, FORWARD, UP);
            let reverse_orient = Quaternion::look_along(-FORWARD, FORWARD, UP);
            let start = DualQuaternion::from_rotation_translation(start_orient, start_loc);
            let reverse = DualQuaternion::from_rotation_translation(reverse_orient, start_loc);

            {
                // Yaw
                let first_rotate =
                    DualQuaternion::from_rotation(dbg!(Quaternion::from_axis_angle(UP, PI)));

                assert!(
                    eps_equal(start * first_rotate, reverse, eps),
                    "\nstart: {}\nrotate: {},\nreverse: {},\nrotated: {}",
                    start,
                    first_rotate,
                    reverse,
                    start * first_rotate
                );

                let second_rotate =
                    DualQuaternion::from_rotation(Quaternion::from_axis_angle(UP, PI));

                assert!(
                    eps_equal(
                        (start * first_rotate * second_rotate)
                            .normalized_match_sign()
                            .relative_to(start),
                        DualQuaternion::unit(),
                        eps
                    ),
                    "\nrotate: {},\nstart: {},\nrotated: {}",
                    first_rotate * second_rotate,
                    start,
                    start * first_rotate * second_rotate
                );
            }

            let second_rotate =
                DualQuaternion::from_rotation(Quaternion::from_axis_angle(RIGHT, FRAC_2_PI));

            let third_rotate =
                DualQuaternion::from_rotation(Quaternion::from_axis_angle(RIGHT, FRAC_2_PI));

            assert!(
                eps_equal(
                    start.transform_by(second_rotate).transform_by(third_rotate),
                    start,
                    eps
                ),
                "\nrotate: {},\nstart: {},\nrotated: {}",
                second_rotate * third_rotate,
                start,
                start.transform_by(second_rotate).transform_by(third_rotate)
            );
        }
    }

    mod displacement_tests {
        use super::*;
        use crate::test_shared::{eps_equal, gen_rand_vec, FORWARD, UP};

        #[test]
        fn test_displace_absolute() {
            let eps = f32::EPSILON as f64;

            for _ in 0..10000 {
                let start_loc = gen_rand_vec();
                let start_dir = gen_rand_vec();
                let start_orient = Quaternion::look_along(start_dir, FORWARD, UP);
                let start = DualQuaternion::from_rotation_translation(start_orient, start_loc);
                let dest_location = {
                    let mut lookat_location = gen_rand_vec();
                    while eps_equal(lookat_location.into_iter(), start_loc.into_iter(), eps) {
                        lookat_location = gen_rand_vec();
                    }
                    lookat_location
                };
                let final_dual_pose = start.translate_absolute(dest_location - start_loc);

                assert!(
                    eps_equal(
                        dest_location.into_iter(),
                        final_dual_pose.to_translation().into_iter(),
                        eps
                    ),
                    "{:?}, {:?}, {:?}",
                    start_loc,
                    dest_location,
                    final_dual_pose.to_translation()
                );
                assert!(
                    eps_equal(
                        start_orient.into_iter(),
                        final_dual_pose.to_rotation_translation().0.into_iter(),
                        eps
                    ),
                    "{:?}, {:?}",
                    dest_location,
                    final_dual_pose.to_translation()
                );
            }
        }

        #[test]
        fn test_displace_relative_single() {
            // let eps = f32::EPSILON as f64;

            let start_loc = Vec3::default();
            let start_dir = Vec3::new(0.0, 0.0, 1.0);
            let start_orient = Quaternion::look_along(start_dir, FORWARD, UP);
            let start = DualQuaternion::from_rotation_translation(start_orient, start_loc);
            let body_frame_translation = Vec3::new(1.0, 0.0, 0.0);
            let world_frame_translation = Vec3::new(0.0, 0.0, 1.0);

            let body_frame_transform =
                DualQuaternion::from_rotation_translation(start_orient, world_frame_translation);
            let world_frame_transform = DualQuaternion::from_rotation_translation(
                Quaternion::unit(),
                world_frame_translation,
            );

            let final_pos_body = start * body_frame_transform;
            let final_pos_world = world_frame_transform * start;

            dbg!(
                final_pos_body.to_translation(),
                final_pos_world.to_translation(),
                world_frame_transform
            );

            let tmp = DualQuaternion::from_rotation_translation(
                Quaternion::unit(),
                body_frame_translation,
            );

            dbg!((start * tmp * start.conjugate()).to_translation());
            dbg!(start * tmp * start.conjugate());
        }
    }
}
