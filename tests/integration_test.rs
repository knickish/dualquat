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

#[cfg(feature = "nanoserde")]
#[test]
fn test_serialization_nanoserde() {
    use dualquat::{Quaternion, Vec3, DualQuaternion};

    let vec3 = Vec3 { i: 2.0, j: 3.0, k: 4.0 };
    let quat = Quaternion { scalar: 1.0, vector: vec3 };
    let dquat = DualQuaternion { real: quat, dual: quat * 2.0 };

    let serialized_vec3 = nanoserde::SerBin::serialize_bin(&vec3);
    let serialized_quat = nanoserde::SerBin::serialize_bin(&quat);
    let serialized_dquat = nanoserde::SerBin::serialize_bin(&dquat);

    assert_eq!(vec3, nanoserde::DeBin::deserialize_bin(&serialized_vec3).unwrap());
    assert_eq!(quat, nanoserde::DeBin::deserialize_bin(&serialized_quat).unwrap());
    assert_eq!(dquat, nanoserde::DeBin::deserialize_bin(&serialized_dquat).unwrap());
}

#[cfg(feature = "speedy")]
#[test]
fn test_serialization_speedy() {
    use dualquat::{Quaternion, Vec3, DualQuaternion};
    use speedy::{Writable, Readable};

    let vec3 = Vec3 { i: 2.0, j: 3.0, k: 4.0 };
    let quat = Quaternion { scalar: 1.0, vector: vec3 };
    let dquat = DualQuaternion { real: quat, dual: quat * 2.0 };

    let serialized_vec3 = vec3.write_to_vec().unwrap();
    let serialized_quat = quat.write_to_vec().unwrap();
    let serialized_dquat = dquat.write_to_vec().unwrap();

    assert_eq!(vec3,  Vec3::read_from_buffer(&serialized_vec3).unwrap());
    assert_eq!(quat,  Quaternion::read_from_buffer(&serialized_quat).unwrap());
    assert_eq!(dquat, DualQuaternion::read_from_buffer(&serialized_dquat).unwrap());
}

#[cfg(feature = "serde")]
#[test]
fn test_serialization_serde() {
    use dualquat::{Quaternion, Vec3, DualQuaternion};

    let vec3 = Vec3 { i: 2.0, j: 3.0, k: 4.0 };
    let quat = Quaternion { scalar: 1.0, vector: vec3 };
    let dquat = DualQuaternion { real: quat, dual: quat * 2.0 };

    let serialized_vec3 = serde_json::to_string(&vec3).unwrap();
    let serialized_quat = serde_json::to_string(&quat).unwrap();
    let serialized_dquat = serde_json::to_string(&dquat).unwrap();

    assert_eq!(vec3, serde_json::from_str(&serialized_vec3).unwrap());
    assert_eq!(quat, serde_json::from_str(&serialized_quat).unwrap());
    assert_eq!(dquat,serde_json::from_str(&serialized_dquat).unwrap());
}