use std::mem::MaybeUninit;
use rayon::iter::{ParallelIterator, IndexedParallelIterator, IntoParallelRefMutIterator, IntoParallelRefIterator, IntoParallelIterator};
use rayon::slice::ParallelSliceMut;
use crate::morton::to_hilbert;
use crate::position::{AABB, EntityPosExt};

pub struct Node {
    aabb: AABB,
    level: u32,
}

impl Node {
    fn child_offset(&self, index: usize) -> usize {
        (index << self.level) - 1
    }
}

pub struct BVH {
    nodes: Vec<Node>,
    leaf_count: usize,
    //using complete binary tree representation
    // since the root is the last node
    // the left child of a node at index i is at index 2*i + 1
}

impl BVH {

    #[inline]
    fn get_parent(index: usize) -> usize {
        (index - 1) >> 1
    }

    #[inline]
    fn get_childs(index: usize) -> (usize, usize) {
        ((index << 1) + 1, (index << 1) + 2)
    }

    #[inline]
    fn is_leaf(index: usize, len: usize) -> bool {
        index >= (len >> 1)
    }


    pub fn new() -> Self {
        Self {
            nodes: Vec::new(),
            leaf_count: 0,
        }
    }

    fn post_fixe_node_build(array: &mut[MaybeUninit<AABB>], index: usize) -> AABB {
        if Self::is_leaf(index, array.len()) {
            let node = &array[index];
            let aabb = unsafe { node.assume_init() };
            return aabb;
        }
        let (left, right) = Self::get_childs(index);
        let left_aabb = Self::post_fixe_node_build(array, left);
        let right_aabb = Self::post_fixe_node_build(array, right);
        let new_aabb = AABB::union(&left_aabb, &right_aabb);
        array[index].write(new_aabb);
        new_aabb
    }

    pub fn build(&mut self, leaves: Vec<AABB>) {

        let mut hilbert_indices = leaves.par_iter().enumerate().map(|(i, aabb)| (to_hilbert(aabb.center().block_pos()), i)).collect::<Vec<_>>();
        hilbert_indices.par_sort_unstable_by_key(|(morton, _)| *morton);


        self.leaf_count = leaves.len();
        let leaf_start = self.leaf_count - 1; //because the is n - 1 branches for n leaves
        let len = 2 * self.leaf_count - 1;
        self.nodes.clear();
        self.nodes.reserve(len);

        //example for 11 leaves
        // 0
        // 1 2
        // 3 4 5 6
        // 7 8 9 10 11 12
        // 15 16 17 18 19 20 21 22 23 24 25

        let array = self.nodes.spare_capacity_mut();
        { //little scope where we likely put our mess
            let leafs = &mut array[leaf_start..len];
            leafs.par_iter_mut().enumerate().for_each(|(i,uninit)|{
                uninit.write(leaves[hilbert_indices[i].1]);
            });
        }

        //Self::post_fixe_node_build(array, 0);

        for i in (0..leaf_start).rev() {
            let (left, right) = Self::get_childs(i);
            let left_aabb = &array[left];
            let right_aabb = &array[right];
            unsafe {
                let new_aabb = AABB::union(left_aabb.assume_init_ref(), right_aabb.assume_init_ref());
                array[i].write(new_aabb);
            }
        }

        unsafe { self.nodes.set_len(len); } //BAM
    }

    pub fn get_collision_par(&self) -> usize {
        let slice = 0..self.leaf_count - 1;
        slice.into_par_iter().map(|i| { //the simple presence of rev() cut the time by half, cache coherence is my guess
            let mut output = 0;
            let (left, right) = Self::get_childs(i);
            self.recursive_collision_between_nodes(left, right, &mut output);
            output
        }).sum()
    }
    
    pub fn get_collision_rev_par(&self) -> usize {
        let slice = 0..self.leaf_count - 1;
        slice.into_par_iter().rev().map(|i| { //the simple presence of rev() cut the time by half, cache coherence is my guess
            let mut output = 0;
            let (left, right) = Self::get_childs(i);
            self.recursive_collision_between_nodes(left, right, &mut output);
            output
        }).sum() 
    }

    pub fn recursive_collision_between_nodes(&self, left: usize, right: usize, output: &mut usize) {
        let len = self.nodes.len();
        let left_node = &self.nodes[left];
        let right_node = &self.nodes[right];
        if !AABB::intersects(left_node, right_node) { return; }
        match (Self::is_leaf(left, len), Self::is_leaf(right, len)) {
            (true, true) => {
                *output += 1;
            },
            (false, true) => {
                let (left_left, left_right) = Self::get_childs(left);
                self.recursive_collision_between_nodes(left_left, right, output);
                self.recursive_collision_between_nodes(left_right, right, output);
            },
            (true, false) => {
                let (right_left, right_right) = Self::get_childs(right);
                self.recursive_collision_between_nodes(left, right_left, output);
                self.recursive_collision_between_nodes(left, right_right, output);
            },
            (false, false) => {
                let (left_left, left_right) = Self::get_childs(left);
                let (right_left, right_right) = Self::get_childs(right);
                self.recursive_collision_between_nodes(left_left, right_left, output);
                self.recursive_collision_between_nodes(left_left, right_right, output);
                self.recursive_collision_between_nodes(left_right, right_left, output);
                self.recursive_collision_between_nodes(left_right, right_right, output);
            }
        }
    }
}