// #![no_std] need to wait for error in core for this

mod dual_quaternion;
mod from;
mod quaternion;
mod vec3;

#[cfg(test)]
mod test_shared;

pub use dual_quaternion::{DualNumber, DualQuaternion};
pub use quaternion::Quaternion;
pub use vec3::Vec3;
