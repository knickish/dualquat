#[cfg(feature = "glam")]
mod glam {
    impl From<glam::DQuat> for crate::Quaternion {
        fn from(quat: glam::DQuat) -> Self {
            Self {
                scalar: quat.w,
                vector: crate::Vec3 {
                    i: quat.x,
                    j: quat.y,
                    k: quat.z,
                },
            }
        }
    }

    impl From<glam::Quat> for crate::Quaternion {
        fn from(quat: glam::Quat) -> Self {
            Self {
                scalar: quat.w as f64,
                vector: crate::Vec3 {
                    i: quat.x as f64,
                    j: quat.y as f64,
                    k: quat.z as f64,
                },
            }
        }
    }

    impl From<glam::Vec3> for crate::Vec3 {
        fn from(vec: glam::Vec3) -> Self {
            Self {
                i: vec.x as f64,
                j: vec.y as f64,
                k: vec.z as f64,
            }
        }
    }

    impl From<glam::DVec3> for crate::Vec3 {
        fn from(vec: glam::DVec3) -> Self {
            Self {
                i: vec.x,
                j: vec.y,
                k: vec.z,
            }
        }
    }

    impl From<crate::Quaternion> for glam::DQuat {
        fn from(quat: crate::Quaternion) -> Self {
            Self {
                x: quat.vector.i,
                y: quat.vector.j,
                z: quat.vector.k,
                w: quat.scalar,
            }
        }
    }

    impl From<crate::Quaternion> for glam::Quat {
        fn from(quat: crate::Quaternion) -> Self {
            Self::from_xyzw(
                quat.vector.i as f32,
                quat.vector.j as f32,
                quat.vector.k as f32,
                quat.scalar as f32,
            )
        }
    }

    impl From<crate::Vec3> for glam::Vec3 {
        fn from(vec: crate::Vec3) -> Self {
            Self {
                x: vec.i as f32,
                y: vec.j as f32,
                z: vec.k as f32,
            }
        }
    }

    impl From<crate::Vec3> for glam::DVec3 {
        fn from(vec: crate::Vec3) -> Self {
            Self {
                x: vec.i,
                y: vec.j,
                z: vec.k,
            }
        }
    }
}

#[cfg(feature = "nalgebra")]
mod nalgebra {
    impl From<nalgebra::DualQuaternion<f64>> for crate::DualQuaternion {
        fn from(dq: nalgebra::DualQuaternion<f64>) -> Self {
            Self {
                real: dq.real.into(),
                dual: dq.dual.into(),
            }
        }
    }

    impl From<nalgebra::DualQuaternion<f32>> for crate::DualQuaternion {
        fn from(dq: nalgebra::DualQuaternion<f32>) -> Self {
            Self {
                real: dq.real.into(),
                dual: dq.dual.into(),
            }
        }
    }

    impl From<crate::DualQuaternion> for nalgebra::DualQuaternion<f64> {
        fn from(dq: crate::DualQuaternion) -> Self {
            Self {
                real: dq.real.into(),
                dual: dq.dual.into(),
            }
        }
    }

    impl From<crate::DualQuaternion> for nalgebra::DualQuaternion<f32> {
        fn from(dq: crate::DualQuaternion) -> Self {
            Self {
                real: dq.real.into(),
                dual: dq.dual.into(),
            }
        }
    }

    impl From<nalgebra::Quaternion<f64>> for crate::Quaternion {
        fn from(quat: nalgebra::Quaternion<f64>) -> Self {
            Self {
                scalar: quat.w,
                vector: crate::Vec3 {
                    i: quat.i,
                    j: quat.j,
                    k: quat.k,
                },
            }
        }
    }

    impl From<nalgebra::Quaternion<f32>> for crate::Quaternion {
        fn from(quat: nalgebra::Quaternion<f32>) -> Self {
            Self {
                scalar: quat.w as f64,
                vector: crate::Vec3 {
                    i: quat.i as f64,
                    j: quat.j as f64,
                    k: quat.k as f64,
                },
            }
        }
    }

    impl From<nalgebra::Vector3<f32>> for crate::Vec3 {
        fn from(vec: nalgebra::Vector3<f32>) -> Self {
            Self {
                i: vec.x as f64,
                j: vec.y as f64,
                k: vec.z as f64,
            }
        }
    }

    impl From<nalgebra::Vector3<f64>> for crate::Vec3 {
        fn from(vec: nalgebra::Vector3<f64>) -> Self {
            Self {
                i: vec.x,
                j: vec.y,
                k: vec.z,
            }
        }
    }

    impl From<crate::Quaternion> for nalgebra::Quaternion<f64> {
        fn from(quat: crate::Quaternion) -> Self {
            nalgebra::Quaternion::<f64>::new(
                quat.scalar,
                quat.vector.i,
                quat.vector.j,
                quat.vector.k,
            )
        }
    }

    impl From<crate::Quaternion> for nalgebra::Quaternion<f32> {
        fn from(quat: crate::Quaternion) -> Self {
            nalgebra::Quaternion::<f32>::new(
                quat.scalar as f32,
                quat.vector.i as f32,
                quat.vector.j as f32,
                quat.vector.k as f32,
            )
        }
    }

    impl From<crate::Vec3> for nalgebra::Vector3<f64> {
        fn from(vec: crate::Vec3) -> Self {
            nalgebra::Vector3::from_iterator(vec.into_iter())
        }
    }

    impl From<crate::Vec3> for nalgebra::Vector3<f32> {
        fn from(vec: crate::Vec3) -> Self {
            nalgebra::Vector3::from_iterator(vec.into_iter().map(|x| x as f32))
        }
    }
}

#[cfg(feature = "mint")]
mod mint {
    impl From<mint::Quaternion<f64>> for crate::Quaternion {
        fn from(quat: mint::Quaternion<f64>) -> Self {
            Self {
                scalar: quat.s,
                vector: quat.v.into(),
            }
        }
    }

    impl From<mint::Quaternion<f32>> for crate::Quaternion {
        fn from(quat: mint::Quaternion<f32>) -> Self {
            Self {
                scalar: quat.s as f64,
                vector: quat.v.into(),
            }
        }
    }

    impl From<mint::Vector3<f64>> for crate::Vec3 {
        fn from(vec: mint::Vector3<f64>) -> Self {
            Self {
                i: vec.x as f64,
                j: vec.y as f64,
                k: vec.z as f64,
            }
        }
    }

    impl From<mint::Vector3<f32>> for crate::Vec3 {
        fn from(vec: mint::Vector3<f32>) -> Self {
            Self {
                i: vec.x as f64,
                j: vec.y as f64,
                k: vec.z as f64,
            }
        }
    }

    impl From<crate::Quaternion> for mint::Quaternion<f64> {
        fn from(quat: crate::Quaternion) -> Self {
            Self {
                v: quat.vector.into(),
                s: quat.scalar,
            }
        }
    }

    impl From<crate::Quaternion> for mint::Quaternion<f32> {
        fn from(quat: crate::Quaternion) -> Self {
            Self {
                v: quat.vector.into(),
                s: quat.scalar as f32,
            }
        }
    }
    impl From<crate::Vec3> for mint::Vector3<f64> {
        fn from(vec: crate::Vec3) -> Self {
            Self {
                x: vec.i,
                y: vec.j,
                z: vec.k,
            }
        }
    }

    impl From<crate::Vec3> for mint::Vector3<f32> {
        fn from(vec: crate::Vec3) -> Self {
            Self {
                x: vec.i as f32,
                y: vec.j as f32,
                z: vec.k as f32,
            }
        }
    }
}
