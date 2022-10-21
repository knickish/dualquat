use crate::{Quaternion, Vec3, DualQuaternion};

pub const FORWARD: Vec3 = Vec3::new(1.0, 0.0, 0.0);
pub const UP: Vec3 = Vec3::new(0.0, 0.0, 1.0);

use nanorand::{Rng, WyRand};

pub(crate) fn gen_rand(generator: Option<&mut WyRand>) -> f64 {
    let rng: &mut WyRand;
    let mut _holder: Option<WyRand> = None;
    match generator {
        Some(gen) => {
            rng = gen;
        }
        None => {
            _holder = Some(WyRand::new());
            rng = _holder.as_mut().unwrap();
        }
    }
    let sign = match rng.generate_range(1_u64..=2) {
        1 => -1.0,
        _ => 1.0,
    };
    let dividend: f64 = rng.generate_range(1_u64..=2) as f64;
    let divisor: f64 = rng.generate_range(1_u64..=2) as f64;
    return sign * (dividend / divisor);
}

pub(crate) fn gen_rand_mat() -> [f64; 16] {
    let mut rng = WyRand::new();
    let mut ret: [f64; 16] = [0.0; 16];
    for i in 0..16 {
        ret[i] = gen_rand(Some(&mut rng));
    }
    return ret;
}

pub(crate) fn gen_rand_vec() -> Vec3 {
    let mut rng = WyRand::new();
    Vec3::new(
        gen_rand(Some(&mut rng)),
        gen_rand(Some(&mut rng)),
        gen_rand(Some(&mut rng)),
    )
}

pub(crate) fn trans_mat(mat: &[f64; 16]) -> [f64; 16] {
    let mut ret: [f64; 16] = [0.0; 16];
    let mut k = 0;
    for i in 0..4 {
        for j in 0..4 {
            ret[k] = mat[j * 4 + i];
            k += 1
        }
    }
    return ret;
}

pub(crate) fn gen_rand_q() -> Quaternion {
    fn gen_rand(rng: &mut WyRand) -> f64 {
        let sign = match rng.generate_range(1_u64..=2) {
            1 => -1.0,
            _ => 1.0,
        };
        let dividend: f64 = rng.generate_range(1_u64..=2) as f64;
        let divisor: f64 = rng.generate_range(1_u64..=2) as f64;
        return sign * dividend / divisor;
    }
    let mut rng = WyRand::new();
    let ret = Quaternion {
        scalar: gen_rand(&mut rng),
        vector: Vec3::new(gen_rand(&mut rng), gen_rand(&mut rng), gen_rand(&mut rng)),
    };

    #[cfg(unused)]
    match class {
        Some(Class::Vector) => {
            ret.scalar = 0.0;
        }
        Some(Class::Unit) => ret = ret.normalized(),
        _ => (),
    };
    return ret;
}

pub(crate) enum Class {
    Unit,
}

pub(crate) fn gen_rand_dq(class: Option<Class>) -> DualQuaternion {
    fn gen_rand(rng: &mut WyRand) -> f64 {
        let sign = match rng.generate_range(1_u64..=2) {
            1 => -1.0,
            _ => 1.0,
        };
        let dividend: f64 = rng.generate_range(1_u64..=2) as f64;
        let divisor: f64 = rng.generate_range(1_u64..=2) as f64;
        return sign * dividend / divisor;
    }
    let mut rng = WyRand::new();
    let ret = DualQuaternion {
        real: Quaternion {
            scalar: gen_rand(&mut rng),
            vector: Vec3::new(gen_rand(&mut rng), gen_rand(&mut rng), gen_rand(&mut rng)),
        },
        dual: Quaternion {
            scalar: gen_rand(&mut rng),
            vector: Vec3::new(gen_rand(&mut rng), gen_rand(&mut rng), gen_rand(&mut rng)),
        },
    };
    match class {
        #[cfg(unused)]
        Some(Class::Vector) => {
            ret.real.scalar = 0.0;
            ret.dual.scalar = 0.0;
        }
        Some(Class::Unit) => ret.normalized(),
        _ => ret,
    }
}

pub(crate) fn eps_equal(
    first: impl Iterator<Item = f64>,
    second: impl Iterator<Item = f64>,
    eps: f64,
) -> bool {
    first
        .zip(second)
        .filter(|(val1, val2)| (val1 - val2).abs() > eps)
        .count()
        == 0
}
