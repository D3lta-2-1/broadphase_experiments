use std::cmp::Ordering;
use rayon::iter::{IntoParallelRefIterator, ParallelIterator};
use crate::morton::to_hilbert;
use crate::position::{AABB, EntityPosExt};

///use the fist bit to determine if it's a leaf or a node, 1 for leaf, 0 for node, theoretically are limited to 2^31 elements which is more like 2^30 entities
#[derive(Clone, Copy)]
struct NodeIndex(u32);

impl NodeIndex {
    fn is_leaf(&self) -> bool {
        self.0 & (1 << 31) == 0
    }

    fn new_leaf(index: usize) -> Self {
        assert!(index < (1 << 31));
        Self(index as u32)
    }

    fn new_node(index: usize) -> Self {
        assert!(index < (1 << 31));
        Self((index as u32) | (1 << 31))
    }

    fn index(&self) -> usize {
        (self.0 & !(1 << 31)) as usize
    }
}

struct Leaf {
    aabb: AABB,
}

struct Node {
    aabb: AABB,
    left: NodeIndex,
    right: NodeIndex,
}

///small temporary struct to be easily sorted by morton code, I believe the smallest it is, the better it is for the cache
struct SortableNodeData {
    node: NodeIndex,
    morton: u128,
}

impl Eq for SortableNodeData {}

impl PartialEq<Self> for SortableNodeData {
    fn eq(&self, other: &Self) -> bool {
        self.morton == other.morton
    }
}

impl PartialOrd<Self> for SortableNodeData {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.morton.partial_cmp(&other.morton)
    }
}

impl Ord for SortableNodeData {
    fn cmp(&self, other: &Self) -> Ordering {
        self.morton.cmp(&other.morton)
    }
}

pub struct BVH {
    leaves: Vec<Leaf>,
    nodes: Vec<Node>,
}


impl BVH {
    pub fn new() -> Self {
        Self {
            leaves: Vec::new(),
            nodes: Vec::new(),
        }
    }

    fn get_aabb(&self, node: NodeIndex) -> &AABB {
        if node.is_leaf() {
            &self.leaves[node.index()].aabb
        } else {
            &self.nodes[node.index()].aabb
        }
    }

    pub fn build(&mut self, leaves: Vec<AABB>) {
        //clear the current state, and reserve the necessary space
        self.leaves.clear();
        self.leaves.reserve(leaves.len());
        self.leaves.extend(leaves.into_iter().map(|aabb| Leaf { aabb }));
        self.nodes.clear();
        self.nodes.reserve(self.leaves.len() - 1);

        //for this implementation, we also need additional space for building the tree
        let mut node_to_process: Vec<SortableNodeData> = self.leaves.iter().enumerate().map(|(i, leaf)| {
            let center = leaf.aabb.center().block_pos();
            let node = NodeIndex::new_leaf(i);
            let morton = to_hilbert(center);
            SortableNodeData { node, morton }
        }).collect();

        loop {
            if node_to_process.len() == 1 {
                break;
            }

            node_to_process.sort_unstable();

            let node_to_add = node_to_process.len() / 2;
            for _ in 0..node_to_add {
                //take the first two nodes
                let left = node_to_process.pop().unwrap().node;
                let right = node_to_process.pop().unwrap().node;
                let left_aabb = self.get_aabb(left);
                let right_aabb = self.get_aabb(right);
                let aabb = AABB::union(left_aabb, right_aabb);

                self.nodes.push(Node {
                    aabb,
                    left,
                    right,
                });
            }

            //add theses new nodes to the list of nodes to process
            let len = self.nodes.len();
            let range = len - node_to_add..len;
            for i in range {
                let center = self.nodes[i].aabb.center().block_pos();
                let node = NodeIndex::new_node(i);
                let morton = to_hilbert(center);
                node_to_process.push(SortableNodeData {
                    node,
                    morton,
                });
            }
        }
    }

    pub fn get_collision_par(&self) -> usize {
        self.nodes.par_iter().map(|node| { //this could be batched more efficiently, but I'm too lazy to do it
            let mut output = 0;
            self.recursive_collision_between_nodes(node.left, node.right, &mut output);
            output
        }).sum()
    }

    pub fn get_collision(&self) -> usize {
        let mut output = 0;
        if let Some(root) = self.nodes.last() {
            self.recursive_visit(root.left, root.right, &mut output);
        }
        output
    }

    fn recursive_visit(&self, left: NodeIndex, right: NodeIndex, output: &mut usize) {
        self.recursive_collision_between_nodes(left, right, output);
        if !left.is_leaf() {
            let node = &self.nodes[left.index()];
            self.recursive_visit(node.left, node.right, output);
        }
        if !right.is_leaf() {
            let node = &self.nodes[right.index()];
            self.recursive_visit(node.left, node.right, output);
        }
    }

    fn recursive_collision_between_nodes(&self, left: NodeIndex, right: NodeIndex, output: &mut usize) {
        match (left.is_leaf(), right.is_leaf()) {
            (true, true) => {
                let left_aabb = &self.leaves[left.index()].aabb;
                let right_aabb = &self.leaves[right.index()].aabb;
                if left_aabb.intersects(right_aabb) {
                    *output += 1;
                }
            },
            (true, false) => {
                let right_node = &self.nodes[right.index()];
                let left_aabb = &self.leaves[left.index()].aabb;
                let right_aabb = &right_node.aabb;
                if left_aabb.intersects(right_aabb) {
                    self.recursive_collision_between_nodes(left, right_node.left, output);
                    self.recursive_collision_between_nodes(left, right_node.right, output);
                }
            },
            (false, true) => {
                let left_node = &self.nodes[left.index()];
                let left_aabb = &left_node.aabb;
                let right_aabb = &self.leaves[right.index()].aabb;
                if left_aabb.intersects(right_aabb) {
                    self.recursive_collision_between_nodes(left_node.left, right, output);
                    self.recursive_collision_between_nodes(left_node.right, right, output);
                }
            },
            (false, false) => {
                let left_node = &self.nodes[left.index()];
                let right_node = &self.nodes[right.index()];
                if left_node.aabb.intersects(&right_node.aabb) {
                    self.recursive_collision_between_nodes(left_node.left, right_node.left, output);
                    self.recursive_collision_between_nodes(left_node.left, right_node.right, output);
                    self.recursive_collision_between_nodes(left_node.right, right_node.left, output);
                    self.recursive_collision_between_nodes(left_node.right, right_node.right, output);
                }
            }
        }
    }


}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_node_index() {
        let leaf = NodeIndex::new_leaf(1234);
        assert!(leaf.is_leaf());
        assert!(!leaf.is_node());
        assert_eq!(leaf.index(), 1234);

        let node = NodeIndex::new_node(5498);
        assert!(!node.is_leaf());
        assert!(node.is_node());
        assert_eq!(node.index(), 5498);
    }
}