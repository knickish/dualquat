# dualquat

A lightweight, zero-dependency 3d math library for use in Dual Quaternion based physics simulation. Capable of representing and transforming location and attitude simultaneously, in fewer operations than would be required for a transformation matrix.

## Features
* `DualQuaternion` type
* `DualNumber` type
* `Quaternion` type
* `Vec3` type

## Optional features
- [`nanoserde`, `serde`, `speedy`] - Serialization of all types
- [`glam`, `nalgebra`, `mint`] - Conversion to/from equivalent types for interop

### Development status 
This is being actively worked on. PRs will be accepted for either more tests or functionality.

### TODO
- Serialization tests for supported libs
- Use approx for tests
- Benchmarks