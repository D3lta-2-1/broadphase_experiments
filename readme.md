# Broadphase_experiments

This repository contains my personal experiments with broad phase algorithms.
My goal is to find which AABB are colliding fast as possible.

possible outputs on my machine (ryzen 7 5800x);
```
world size: -1000..1000
entity count: 500000
box half size: 50
------------------------------------
collisions: 117523857 in 306.0963746s with dummy way
------------------------------------
bvh2 build in 374.2577ms
collisions: 117523857 in 1.2557179s with BVH
total time: 1.6299756s
------------------------------------
bvh3 build in 346.5087ms
collisions: 117523857 in 254.2261ms with BVH3
total time: 600.7348ms
collisions: 117523857 in 1.314552s with BVH3 recur
------------------------------------
bvh4 build in 187.9494ms
collisions: 117523858 in 1.1840283s with BVH4
total time: 1.3719777s
------------------------------------
bvh4 build par in 42.1384ms
collisions: 117523857 in 179.5408ms with BVH4
total time: 221.6792ms
```