use dualquat::{DualQuaternion as DQ, Quaternion as Q, Vec3};
const X: Vec3 = Vec3::new(1.0, 0.0, 0.0);
const Y: Vec3 = Vec3::new(0.0, 1.0, 0.0);
const Z: Vec3 = Vec3::new(0.0, 0.0, 1.0);

#[test]
fn relative_translation_preserves_rotation_and_moves_in_body_axes() {
    let q = Q::from_axis_angle(Z, std::f64::consts::FRAC_PI_2);
    let start = Vec3::new(3.0, 4.0, 5.0);
    let pose = DQ::from_rotation_translation(q, start);
    for distance in [0.0, 2.0, -2.0] {
        let moved = pose.translate_relative(X * distance);
        assert!(moved.to_translation().distance(start + Y * distance) < 1e-12);
        assert!((moved.real * X).distance(Y) < 1e-12);
    }
}

#[test]
fn normalizing_scaled_pose_preserves_the_rigid_transform() {
    let q = Q::from_axis_angle(Z, 0.7);
    let t = Vec3::new(3.0, 4.0, 5.0);
    let pose = DQ::from_rotation_translation(q, t);
    for scale in [0.1, 2.0, -3.0] {
        let normalized = (pose * scale).normalized();
        assert!((normalized.real.norm() - 1.0).abs() < 1e-12);
        assert!(normalized.to_translation().distance(t) < 1e-12);
        assert!((normalized.real * X).distance(q * X) < 1e-12);
        let twice = normalized.normalized();
        assert!(twice.to_translation().distance(t) < 1e-12);
    }
    // Normalization also removes a component parallel to the real part.
    let contaminated = DQ {
        real: pose.real,
        dual: pose.dual + pose.real * 3.0,
    };
    let normalized = contaminated.normalized();
    assert!(normalized.real.dot(normalized.dual).scalar.abs() < 1e-12);
    assert!(normalized.to_translation().distance(t) < 1e-12);
}

#[test]
fn interpolation_has_correct_endpoints_and_translation_midpoints() {
    let start = DQ::from_rotation_translation(Q::unit(), X * 2.0);
    for rotation in [Q::unit(), Q::from_axis_angle(Z, 0.5)] {
        let end = DQ::from_rotation_translation(rotation, X * 10.0);
        for balance in [0.0, 0.25, 0.5, 1.0] {
            let pose = start.interpolate(&end, balance);
            assert!(pose.to_translation().distance(X * (2.0 + balance * 8.0)) < 1e-12);
            assert!((pose.real.norm() - 1.0).abs() < 1e-12);
        }
    }
}

#[test]
fn slerp_is_finite_for_equal_opposite_and_nearly_equal_orientations() {
    let q = Q::from_axis_angle(Z, 0.7).normalized();
    for end in [q, -q, Q::from_axis_angle(Z, 0.70000001)] {
        for balance in [0.0, 0.5, 1.0] {
            let interpolated = q.slerp(end, balance);
            assert!(interpolated.into_iter().all(f64::is_finite));
            assert!((interpolated.norm() - 1.0).abs() < 1e-12);
            assert!((interpolated * X).distance(q * X) < 1e-7);
        }
    }
    assert_eq!(Q::unit().slerp(Q::unit(), 0.5), Q::unit());
}
