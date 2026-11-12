<p align="center">
  <img src="assets/logo.svg" width="88" alt="voxbroad logo"/>
</p>

# voxbroad

Pure Dart spatial hash grids, sweep-and-prune broadphase, and continuous swept-AABB time-of-impact solvers for `gl_volition`.

## Features
- **`SpatialHashGrid3D`**: Dynamic 3D integer cell bucket spatial partitioning with O(1) cell lookup, AABB region queries, and deduplicated candidate overlap pairs.
- **`SweepAndPrune1D`**: Sorted endpoint broadphase along a primary axis with insertion-sort frame coherence and full 3D AABB verification.
- **`SweptAabbToi`**: Continuous Minkowski-difference slab sweep for dynamic AABBs against static or relative obstacles, returning exact normalized Time-of-Impact `[0, 1]`, hit normal, and contact center.
