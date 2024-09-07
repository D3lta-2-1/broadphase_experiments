use std::collections::HashMap;
use nalgebra::Vector3;
use crate::position::{AABB, EntityPosExt};

pub const CELL_SIZE: i32 = 128;

pub struct GridTracker {
    grid: HashMap<Vector3<i32>, Vec<AABB>>,
}

impl GridTracker {
    pub fn new() -> Self {
        Self {
            grid: HashMap::new(),
        }
    }

    pub fn insert(&mut self, aabb: AABB) {
        let min = aabb.min().block_pos();
        let max = aabb.max().block_pos();

        let min = min.map(|x| x / CELL_SIZE);
        let max = max.map(|x| x / CELL_SIZE);

        for x in min.x..=max.x {
            for y in min.y..=max.y {
                for z in min.z..=max.z {
                    let key = Vector3::new(x, y, z);
                    self.grid.entry(key).or_insert_with(Vec::new).push(aabb);
                }
            }
        }
    }

    pub fn get_collisions(&self) -> usize {
        let mut collisions = 0;
        for (_, aabbs) in self.grid.iter() {
            for (i, aabb1) in aabbs.iter().enumerate() {
                for aabb2 in aabbs[i + 1..].iter() {
                    if aabb1.intersects(aabb2) {
                        collisions += 1;
                    }
                }
            }
        }
        collisions
    }
}