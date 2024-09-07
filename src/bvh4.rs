use rayon::iter::{IntoParallelIterator, IntoParallelRefIterator, ParallelIterator, IndexedParallelIterator};
use rayon::slice::ParallelSliceMut;
use crate::morton::to_hilbert;
use crate::position::{AABB, EntityPosExt};

enum NodeKind {
    Leaf,
    Branch(usize, usize),
}

struct Node {
    aabb: AABB,
    kind: NodeKind,
}

pub struct BVH {
    nodes: Vec<Node>, //from my observation, storing branches and leaves in the same vec is faster than storing them in separate vecs, I believe it's because of the cache
    start_of_branches: usize, // the slice [0..start_of_branches] contains the leaves, the slice [start_of_branches..] contains the branches
}

impl BVH {
    pub fn new() -> Self {
        Self {
            nodes: Vec::new(),
            start_of_branches: 0,
        }
    }

    pub fn build(&mut self, leaves: Vec<AABB>) {
        let len = leaves.len();
        self.nodes.clear();
        self.nodes.reserve(2 * len - 1);

        let mut hilbert_indices = leaves.iter().enumerate().map(|(i, aabb)| (to_hilbert(aabb.center().block_pos()), i)).collect::<Vec<_>>();
        hilbert_indices.sort_unstable_by_key(|(morton, _)| *morton);

        let iter = hilbert_indices.into_par_iter().map(|(_, i)| Node { aabb: leaves[i], kind: NodeKind::Leaf });

        iter.collect_into_vec(&mut self.nodes);

        //self.nodes.sort_unstable_by_key(|node| to_morton_index(node.aabb.center().block_pos()));
        self.start_of_branches = len;

        let mut node_to_add = self.nodes.len() / 2;
        let mut begin = 0;

        while node_to_add > 0 {

            //we don't need to sort the added nodes, union of nodes keep local coherence too
            let range = 0..node_to_add;

            for i in range.rev() { //rev is important, it avoids to leave an odd node in his own unique branch
                let left = i * 2 + begin;
                let right = left + 1;
                let aabb = AABB::union(&self.nodes[left].aabb, &self.nodes[right].aabb);
                self.nodes.push(Node { aabb, kind: NodeKind::Branch(left, right) });
            }

            begin += node_to_add * 2;
            node_to_add = (self.nodes.len() - begin) / 2; // we can't simply divide node_to_add by 2, because the last level of nodes can be odd

        }
    }

    pub fn build_par(&mut self, leaves: Vec<AABB>) {
        let len = leaves.len();
        self.nodes.clear();
        self.nodes.reserve(2 * len - 1);

        //sort unstable is a bunch of garbage, maybe there is another better way to do it, for BVH 5 I'll lik
        let mut morton_indices = leaves.par_iter().enumerate().map(|(i, aabb)| (to_hilbert(aabb.center().block_pos()), i)).collect::<Vec<_>>();
        morton_indices.par_sort_unstable_by_key(|(morton, _)| *morton);

        let iter = morton_indices.into_par_iter().map(|(_, i)| Node { aabb: leaves[i], kind: NodeKind::Leaf });

        iter.collect_into_vec(&mut self.nodes);

        //self.nodes.sort_unstable_by_key(|node| to_morton_index(node.aabb.center().block_pos()));
        self.start_of_branches = len;

        let mut node_to_add = self.nodes.len() / 2;
        let mut begin = 0;

        loop {
            if node_to_add == 0 {
                break;
            }

            //we don't need to sort the added nodes, union of nodes keep local coherence too
            let range = 0..node_to_add;

            for i in range.rev() { //rev is important, it avoids to leave an odd node in his own unique branch
                let left = i * 2 + begin;
                let right = left + 1;
                let aabb = AABB::union(&self.nodes[left].aabb, &self.nodes[right].aabb);
                self.nodes.push(Node { aabb, kind: NodeKind::Branch(left, right) });
            }

            begin += node_to_add * 2;
            node_to_add = (self.nodes.len() - begin) / 2; // we can't simply divide node_to_add by 2, because the last level of nodes can be odd

        }
    }

    pub fn get_collision_recursive(&self) -> usize {
        let mut output = 0;
        if let Some(Node{kind: NodeKind::Branch(left, right), ..}) = &self.nodes.last() { // if None, there is no collision
            self.recursive_visit(*left, *right, &mut output);                           // if Some but not a branch, there is only one leaf, so no collision
        }
        self.recursive_visit(0, 1, &mut output);
        output
    }

    pub fn get_collision_par(&self) -> usize {
        let slice = &self.nodes[self.start_of_branches..];
        slice.par_iter().map(|node| { //this could be batched more efficiently, but I'm too lazy to do it
            let mut output = 0;
            if let NodeKind::Branch(left, right) = &node.kind {
                self.recursive_collision_between_nodes(*left, *right, &mut output);
            }
            output
        }).sum()
    }

    pub fn recursive_visit(&self, left: usize, right: usize, output: &mut usize) {
        if let NodeKind::Branch(left, right) = &self.nodes[left].kind {
            self.recursive_visit(*left, *right, output);
        }
        if let NodeKind::Branch(left, right) = &self.nodes[right].kind {
            self.recursive_visit(*left, *right, output);
        }
        self.recursive_collision_between_nodes(left, right, output);
    }
    
    pub fn recursive_collision_between_nodes(&self, left: usize, right: usize, output: &mut usize) {
        let left_node = &self.nodes[left];
        let right_node = &self.nodes[right];
        if !AABB::intersects(&left_node.aabb, &right_node.aabb) { return; }
        match (&left_node.kind, &right_node.kind) {
            (NodeKind::Leaf, NodeKind::Leaf) => {
                *output += 1;
            },
            (NodeKind::Branch(left_left, left_right), NodeKind::Leaf) => {
                self.recursive_collision_between_nodes(*left_left, right, output);
                self.recursive_collision_between_nodes(*left_right, right, output);
            },
            (NodeKind::Leaf, NodeKind::Branch(right_left, right_right)) => {
                self.recursive_collision_between_nodes(left, *right_left, output);
                self.recursive_collision_between_nodes(left, *right_right, output);
            },
            (NodeKind::Branch(left_left, left_right), NodeKind::Branch(right_left, right_right)) => {
                self.recursive_collision_between_nodes(*left_left, *right_left, output);
                self.recursive_collision_between_nodes(*left_left, *right_right, output);
                self.recursive_collision_between_nodes(*left_right, *right_left, output);
                self.recursive_collision_between_nodes(*left_right, *right_right, output);
            }
        }
    } 
}