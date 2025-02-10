use std::cmp::PartialEq;
use std::iter::repeat_with;
use crate::morton::to_hilbert;
use crate::position::{AABB, EntityPosExt};

#[derive(Debug, PartialEq)]
enum Node {
    Uninit,
    Leaf{
        morton: u128,
        aabb: AABB,
        // ... entity ID stuff here
    },
    Node {
        morton: u128,
        aabb: AABB,
        left: usize, // usize might be overkill
        right: usize,
    },
}

impl Node {
    fn get_aabb(&self) -> &AABB {
        match self {
            Node::Uninit => panic!("Uninitialized node"), //this should never happen
            Node::Leaf { aabb, .. } => aabb,
            Node::Node { aabb, .. } => aabb,
        }
    }

    fn get_morton(&self) -> u128 {
        match self {
            Node::Uninit => panic!("Uninitialized node"), //this should never happen
            Node::Leaf { morton, .. } => *morton,
            Node::Node { morton, .. } => *morton,
        }
    }
}

pub struct BVH {
    nodes: Vec<Node>,
}

impl BVH {
    pub fn new() -> Self {
        Self {
            nodes: Vec::new(),
        }
    }

    pub fn build(&mut self, leaves: Vec<AABB>) {
        let len = leaves.len();

        //setup the nodes vec
        self.nodes.clear();
        self.nodes.reserve(2 * len - 1); //this formula doesn't come from out of nowhere, if you want to store n leaves, you need n-1 branches, so 2n-1 nodes in total
        let iter = leaves.into_iter().map(|aabb| Node::Leaf { morton: to_hilbert(aabb.center().block_pos()) , aabb });
        let fill_with_uninit = repeat_with(|| Node::Uninit).take(len -1 ); //n-1 branches...
        self.nodes.extend(iter);
        self.nodes.extend(fill_with_uninit);

        //setup loop elements
        let mut batch_start = 0; //of many node do we process to make the "upper" level
        let mut writing_start = len; //this is the index is where we start to write the next level of nodes
        let mut node_to_add = len / 2; //how many nodes will be added in the next level



        loop {
            if node_to_add == 0 {
                break;
            }

            let (sorting_slice, writing_slice) = self.nodes.split_at_mut(writing_start);

            let sorting_slice = &mut sorting_slice[batch_start..];
            let writing_slice = &mut writing_slice[..node_to_add];

            //sort the nodes with morton codes
            sorting_slice.sort_unstable_by(|a, b| {
                a.get_morton().cmp(&b.get_morton())
            });

            for (i, new_node) in writing_slice.iter_mut().enumerate() {
                let left = 2 * i;
                let right = left + 1;
                let left_aabb = sorting_slice[left].get_aabb();
                let right_aabb = sorting_slice[right].get_aabb();

                let union = AABB::union(left_aabb, right_aabb);

                let morton = to_hilbert(union.center().block_pos());

                *new_node = Node::Node {
                    morton,
                    aabb: union,
                    left: left + batch_start,
                    right: right + batch_start,
                };
            }

            //update loop elements
            batch_start += node_to_add * 2; //number of nodes processed, we don't use len because it could be odd
            writing_start += node_to_add;
            node_to_add = (writing_start - batch_start) / 2;
        }
    }

    pub fn get_collision(&self) -> usize {
        let mut collisions = 0;
        if let Some(Node::Node { left, right, .. }) = self.nodes.last() {
            self.recursive_visit(*left, *right, &mut collisions);
        }
        collisions
    }

    //this function doesn't check for collisions directly, it simply visits the tree (so maybe a linear iterator would be faster)
    fn recursive_visit(&self, left: usize, right: usize, output: &mut usize) {
        //this iterates over the tree but doesn't check for collisions
        if let Node::Node { left, right, .. } = &self.nodes[left] {
            self.recursive_visit(*left, *right, output);
        }
        if let Node::Node { left, right, .. } = &self.nodes[right] {
            self.recursive_visit(*left, *right, output);
        }
        self.recursive_collision_between_nodes(left, right, output);
    }

    //this function check for intersections between nodes, this is the most expensive part of the algorithm, it's step node by node checking for intersections
    fn recursive_collision_between_nodes(&self, left: usize, right: usize, output: &mut usize) {
        let left_node = &self.nodes[left];
        let right_node = &self.nodes[right];
        let left_aabb = left_node.get_aabb();
        let right_aabb = right_node.get_aabb();
        if !AABB::intersects(left_aabb, right_aabb) { return; } //no collision, nothing to do

        match (left_node, right_node) {
            (Node::Leaf { .. }, Node::Leaf { .. }) => { //case 1: both are leaves
                *output += 1;
            }
            (Node::Node { left: left_left, right: left_right, .. }, Node::Leaf { .. }) => { //case 2: left is a node, right is a leaf
                self.recursive_collision_between_nodes(*left_left, right, output);
                self.recursive_collision_between_nodes(*left_right, right, output);
            }
            (Node::Leaf { .. }, Node::Node { left: right_left, right: right_right, .. }) => { //case 3: left is a leaf, right is a node
                self.recursive_collision_between_nodes(left, *right_left, output);
                self.recursive_collision_between_nodes(left, *right_right, output);
            }
            (Node::Node { left: left_left, right: left_right, .. }, Node::Node { left: right_left, right: right_right, .. }) => { //case 4: both are nodes
                self.recursive_collision_between_nodes(*left_left, *right_left, output);
                self.recursive_collision_between_nodes(*left_left, *right_right, output);
                self.recursive_collision_between_nodes(*left_right, *right_left, output);
                self.recursive_collision_between_nodes(*left_right, *right_right, output);
            }
            _ => unreachable!(),
        }
    }
}