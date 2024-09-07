use fixed::traits::{Fixed, ToFixed};
use fixed::types::I32F32;
use nalgebra::Vector3;

pub type EntityPos = Vector3<I32F32>;
pub type BlockPos = Vector3<i32>;

pub trait EntityPosExt: Sized {
    fn from_primitives(x: impl ToFixed, y: impl ToFixed, z: impl ToFixed) -> Self;
    fn block_pos(&self) -> BlockPos;
}

impl EntityPosExt for EntityPos {
    fn from_primitives(x: impl ToFixed, y: impl ToFixed, z: impl ToFixed) -> Self {
        new_fixed_vec(x, y, z)
    }
    fn block_pos(&self) -> BlockPos {
        BlockPos::new(self.x.to_num(), self.y.to_num(), self.z.to_num())
    }
}

pub fn new_fixed_vec<T: Fixed>(x: impl ToFixed, y: impl ToFixed, z: impl ToFixed) -> Vector3<T> {
    Vector3::new(x.to_fixed(), y.to_fixed(), z.to_fixed())
}

#[derive(Debug, PartialEq, Clone, Copy)]
pub struct AABB {
    min: EntityPos,
    max: EntityPos,
}

impl AABB {

    pub fn new(min: EntityPos, max: EntityPos) -> Self {
        assert!(min.x <= max.x && min.y <= max.y && min.z <= max.z);
        Self {
            min,
            max,
        }
    }
    pub fn from_center(center: EntityPos, half_size: EntityPos) -> Self {
        let min = center - half_size;
        let max = center + half_size;
        Self::new(min, max)
    }

    pub fn empty() -> Self {
        let min = EntityPos::from_primitives(I32F32::ZERO, I32F32::ZERO, I32F32::ZERO);
        let max = EntityPos::from_primitives(I32F32::ZERO, I32F32::ZERO, I32F32::ZERO);
        Self::new(min, max)
    }

    pub fn min(&self) -> &EntityPos {
        &self.min
    }
    pub fn max(&self) -> &EntityPos {
        &self.max
    }

    pub fn center(&self) -> EntityPos {
        (self.min + self.max) / I32F32::from_num(2)
    }

    pub fn intersects(&self, other: &Self) -> bool {
        self.min().x <= other.max().x && self.max().x >= other.min().x &&
            self.min().y <= other.max().y && self.max().y >= other.min().y &&
            self.min().z <= other.max().z && self.max().z >= other.min().z
    }

    pub fn union(&self, other: &Self) -> Self {
        let min = self.min.zip_map(&other.min, |a, b| a.min(b));
        let max = self.max.zip_map(&other.max, |a, b| a.max(b));
        Self::new(min, max)
    }

}

