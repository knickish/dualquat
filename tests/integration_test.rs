#[cfg(feature = "glam")]
#[test]
fn test_interop_glam() {
    use dualquat::{Quaternion, Vec3};

    let vec3 = Vec3 { i: 2.0, j: 3.0, k: 4.0 };
    let quat = Quaternion { scalar: 1.0, vector: vec3 };

    let vec3_external_f64: glam::DVec3 = vec3.into();
    let vec3_external_f32: glam::Vec3 = vec3.into();

    let quat_external_f64: glam::DQuat = quat.into();
    let quat_external_f32: glam::Quat = quat.into();

    assert_eq!(vec3, vec3_external_f64.into());
    assert_eq!(vec3, vec3_external_f32.into());

    assert_eq!(quat, quat_external_f64.into());
    assert_eq!(quat, quat_external_f32.into());
}

#[cfg(feature = "nalgebra")]
#[test]
fn test_interop_nalgebra() {
    use dualquat::{Quaternion, Vec3, DualQuaternion};

    let vec3 = Vec3 { i: 2.0, j: 3.0, k: 4.0 };
    let quat = Quaternion { scalar: 1.0, vector: vec3 };
    let dquat = DualQuaternion { real: quat, dual: quat * 2.0 };

    let vec3_external_f64: nalgebra::Vector3<f64> = vec3.into();
    let vec3_external_f32: nalgebra::Vector3<f32> = vec3.into();

    let quat_external_f64: nalgebra::Quaternion<f64> = quat.into();
    let quat_external_f32: nalgebra::Quaternion<f32> = quat.into();

    let dquat_external_f64: nalgebra::DualQuaternion<f64> = dquat.into();
    let dquat_external_f32: nalgebra::DualQuaternion<f32> = dquat.into();

    assert_eq!(vec3, vec3_external_f64.into());
    assert_eq!(vec3, vec3_external_f32.into());

    assert_eq!(quat, quat_external_f64.into());
    assert_eq!(quat, quat_external_f32.into());

    assert_eq!(dquat, dquat_external_f64.into());
    assert_eq!(dquat, dquat_external_f32.into());
}

#[cfg(feature = "mint")]
#[test]
fn test_interop_mint() {
    use dualquat::{Quaternion, Vec3};

    let vec3 = Vec3 { i: 2.0, j: 3.0, k: 4.0 };
    let quat = Quaternion { scalar: 1.0, vector: vec3 };

    let vec3_external_f64: mint::Vector3<f64> = vec3.into();
    let vec3_external_f32: mint::Vector3<f32> = vec3.into();

    let quat_external_f64: mint::Quaternion<f64> = quat.into();
    let quat_external_f32: mint::Quaternion<f32> = quat.into();

    assert_eq!(vec3, vec3_external_f64.into());
    assert_eq!(vec3, vec3_external_f32.into());

    assert_eq!(quat, quat_external_f64.into());
    assert_eq!(quat, quat_external_f32.into());
}