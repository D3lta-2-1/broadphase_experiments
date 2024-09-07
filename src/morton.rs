use lindel::hilbert_encode;
use nalgebra::Vector3;

#[inline]
fn to_positive(val: i32) -> u32 {
    //if val is negative, we need to flip the first bit to make it positive
    //if it is positive, we need to offset it by 2^31
    //in both cases, we need to flip the first bit...
    (val ^ 1 << 31) as u32
}

pub fn to_hilbert(pos: Vector3<i32>) -> u128 {

    let arr = [to_positive(pos.x), to_positive(pos.y), to_positive(pos.z)];
    hilbert_encode(arr)
}



pub fn to_morton(pos: Vector3<i32>) -> u128 {
    let x = morton_partition_3(pos.x);
    let y = morton_partition_3(pos.y);
    let z = morton_partition_3(pos.z);
    x | (y << 1) | (z << 2)
}

fn morton_partition_3(x: i32) -> u128 {
    let mut x = to_positive(x) as u128;
    x = (x | x << 32) & 0x1f00000000ffff;
    x = (x | x << 16) & 0x1f0000ff0000ff;
    x = (x | x << 8) & 0x100f00f00f00f00f;
    x = (x | x << 4) & 0x10c30c30c30c30c3;
    x = (x | x << 2) & 0x1249249249249249;
    x
}
