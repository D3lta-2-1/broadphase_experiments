use fixed::types::I32F32;
use nalgebra::Vector3;
use rayon::prelude::*;
use crate::morton::to_morton;
use crate::position::{AABB, EntityPosExt};


//This code must be wrong because of my morton implementation
struct Node {
    aabb: AABB,
    min: u128,
    max: u128,
}

impl Node {
    fn new(aabb: AABB) -> Self {
        let min = to_morton(aabb.min().block_pos());

        let max = aabb.max() - Vector3::new(I32F32::DELTA, I32F32::DELTA, I32F32::DELTA);
        let max = to_morton(max.block_pos());

        Self {
            aabb,
            min,
            max,
        }
    }
}

pub struct MortonList {
    nodes: Vec<Node>,
}

impl MortonList {
    pub fn new() -> Self {
        Self {
            nodes: Vec::new(),
        }
    }

    pub fn build(&mut self, aabbs: Vec<AABB>) {
        self.nodes = aabbs.into_par_iter().map(|aabb| Node::new(aabb)).collect();
        self.nodes.par_sort_unstable_by_key(|node| node.min);
    }

    pub fn get_collisions(&self) -> usize {
        self.nodes.par_iter().enumerate().map(|(i, node)| {
            let mut collisions = 0;
            for other in self.nodes.iter().skip(i + 1) {
                if node.max < other.min { //this can be really long with the wrong interval
                    break;
                }
                if node.aabb.intersects(&other.aabb) {
                    collisions += 1;
                }
            }
            collisions
        }).sum()
    }
}
