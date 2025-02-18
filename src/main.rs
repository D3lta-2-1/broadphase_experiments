#![feature(iter_array_chunks)]

use crate::position::{new_fixed_vec, EntityPos, EntityPosExt, AABB};
use rand::Rng;
use std::ops::Range;
use std::time::{Duration, Instant};

mod bvh2;
mod bvh3;
mod bvh4;
mod bvh5;
mod homemade;
mod morton;
mod position;
mod static_grid;
//mod bvh6;

const RANGE: Range<i32> = -10_000..10_000;
const ENTITY_COUNT: usize = 10_000;
const BOX_HALF_SIZE: i32 = 50;

fn random_pos() -> EntityPos {
    let mut rng = rand::thread_rng();
    EntityPos::from_primitives(
        rng.gen_range(RANGE.clone()),
        rng.gen_range(RANGE.clone()),
        rng.gen_range(RANGE.clone()),
    )
}

fn main() {
    let leaves: Vec<AABB> = (0..ENTITY_COUNT)
        .map(|_| {
            let pos = random_pos();
            let aabb = AABB::from_center(
                pos,
                new_fixed_vec(BOX_HALF_SIZE, BOX_HALF_SIZE, BOX_HALF_SIZE),
            );
            aabb
        })
        .collect();

    rayon::ThreadPoolBuilder::new().build_global().unwrap();

    println!("world size: {:?}", RANGE);
    println!("entity count: {}", leaves.len());
    println!("box half size: {}", BOX_HALF_SIZE);
    println!("num_thread: {}", rayon::current_num_threads());

    /*
    //-- dummy way to test all collisions
    if ENTITY_COUNT <= 10_000 {
        println!("------------------------------------");
        let time = Instant::now();
        let mut collisions = 0;
        for (i, aabb1) in leaves.iter().enumerate() {
            for aabb2 in leaves[i + 1..].iter() {
                if aabb1.intersects(aabb2) {
                    collisions += 1;
                }
            }
        }
        let elapsed = time.elapsed();
        println!("collisions: {collisions} in {:?} with dummy way", elapsed);
    }


    //-- BVH2 way
    println!("------------------------------------");
    let mut bvh2 = bvh2::BVH::new();
    let clone = leaves.clone();

    let time = Instant::now();
    bvh2.build(clone);
    let elapsed = time.elapsed();
    println!("bvh2 build in {:?}", elapsed);

    let time = Instant::now();
    let bvh_collisions = bvh2.get_collision();
    let elapsed2 = time.elapsed();
    println!("collisions: {bvh_collisions} in {:?} with BVH", elapsed2);
    println!("total time: {:?}", elapsed + elapsed2);

    //-- BVH3 way
    println!("------------------------------------");
    let mut bvh3 = bvh3::BVH::new();
    let clone = leaves.clone();

    let time = Instant::now();
    bvh3.build(clone);
    let elapsed = time.elapsed();
    println!("bvh3 build in {:?}", elapsed);

    let time = Instant::now();
    let bvh3_collisions = bvh3.get_collision_par();
    let elapsed2 = time.elapsed();
    println!("collisions: {bvh3_collisions} in {:?} with BVH3", elapsed2);
    println!("total time: {:?}", elapsed + elapsed2);

    let time = Instant::now();
    let collisions = bvh3.get_collision();
    let elapsed = time.elapsed();
    println!("collisions: {collisions} in {:?} with BVH3 recur", elapsed);

    //-- BVH4 way
    println!("------------------------------------");
    let mut bvh4 = bvh4::BVH::new();
    let clone = leaves.clone();

    let time = Instant::now();
    bvh4.build(clone);
    let elapsed = time.elapsed();
    println!("bvh4 build in {:?}", elapsed);

    let time = Instant::now();
    let bvh4_collisions = bvh4.get_collision_recursive();
    let elapsed2 = time.elapsed();
    println!("collisions: {bvh4_collisions} in {:?} with BVH4", elapsed2);
    println!("total time: {:?}", elapsed + elapsed2); */

    //-- BVH4 par way
    /*println!("------------------------------------");
    let mut bvh4 = bvh4::BVH::new();
    let clone = leaves.clone();

    let time = Instant::now();
    bvh4.build_par(clone);
    let elapsed = time.elapsed();
    println!("bvh4 build par in {:?}", elapsed);

    let time = Instant::now();
    let bvh4_collisions = bvh4.get_collision_par();
    let elapsed2 = time.elapsed();
    println!("collisions: {bvh4_collisions} in {:?} with BVH4", elapsed2);
    let total = elapsed + elapsed2;
    println!("total time: {:?}", total); */

    //-- BVH5 way
    println!("------------------------------------");
    let mut bvh5 = bvh5::BVH::new();
    let clone = leaves.clone();

    let time = Instant::now();
    bvh5.build(clone);
    let elapsed = time.elapsed();
    println!("bvh5 build in {:?}", elapsed);

    /*

    let time = Instant::now();
    let bvh5_collisions = bvh5.get_collision_par();
    let elapsed2 = time.elapsed();
    println!("collisions: {bvh5_collisions} in {:?} with BVH5", elapsed2);
    println!("total time: {:?}", elapsed + elapsed2);

    let time = Instant::now();
    let bvh5_collisions = bvh5.get_collision_rev_par();
    let elapsed2 = time.elapsed();
    println!(
        "collisions: {bvh5_collisions} in {:?} with BVH5 rev",
        elapsed2
    );
    println!("total time: {:?}", elapsed + elapsed2);*/


    let mut forward = Duration::ZERO;
    let mut backward = Duration::ZERO;

    for _ in 0..50 {
        let time = Instant::now();
        let bvh5_collisions = bvh5.get_collision_par();
        let elapsed2 = time.elapsed();
        forward += elapsed2;
    }
    for _ in 0..50 {
        let time = Instant::now();
        let bvh5_collisions = bvh5.get_collision_rev_par();
        let elapsed2 = time.elapsed();
        backward += elapsed2;
    }

    println!("forward: {:?}", forward/50);
    println!("backward: {:?}", backward/50);


    /* both are broken and give wrong results
    //-- homemade way
    println!("------------------------------------");
    let mut morton_list = homemade::MortonList::new();
    let clone = leaves.clone();

    let time = Instant::now();
    morton_list.build(clone);
    let elapsed = time.elapsed();
    println!("morton-based enclosing build in {:?}", elapsed);

    let time = Instant::now();
    let homemade_collisions = morton_list.get_collisions();
    let elapsed2 = time.elapsed();
    println!("collisions: {homemade_collisions} in {:?} with morton-based enclosing", elapsed2);
    println!("total time: {:?}", elapsed + elapsed2);

    //-- Static grid way
    println!("------------------------------------");
    println!("grid size: {:?}", static_grid::CELL_SIZE);
    let mut grid = static_grid::GridTracker::new();
    let clone = leaves.clone();

    let time = Instant::now();
    for aabb in clone {
        grid.insert(aabb);
    }
    let elapsed = time.elapsed();
    println!("grid build in {:?}", elapsed);

    let time = Instant::now();
    let grid_collisions = grid.get_collisions();
    let elapsed2 = time.elapsed();
    println!("collisions: {grid_collisions} in {:?} with grid", elapsed2);
    println!("total time: {:?}", elapsed + elapsed2);*/
}
