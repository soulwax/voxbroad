import 'package:test/test.dart';
import 'package:voxbroad/voxbroad.dart';

void main() {
  test('SpatialHashGrid3D queries AABBs and deduplicates overlap pairs accurately', () {
    final grid = SpatialHashGrid3D(cellSize: 2.0);
    grid.insertOrUpdate(
      BroadEntry(
        id: 1,
        aabb: BroadAabb.fromCenterHalfExtents(
          const BroadVec3(0.0, 0.0, 0.0),
          const BroadVec3(1.0, 1.0, 1.0),
        ),
      ),
    );
    grid.insertOrUpdate(
      BroadEntry(
        id: 2,
        aabb: BroadAabb.fromCenterHalfExtents(
          const BroadVec3(1.5, 0.0, 0.0),
          const BroadVec3(1.0, 1.0, 1.0),
        ),
      ),
    );
    grid.insertOrUpdate(
      BroadEntry(
        id: 3,
        aabb: BroadAabb.fromCenterHalfExtents(
          const BroadVec3(8.0, 0.0, 0.0),
          const BroadVec3(0.5, 0.5, 0.5),
        ),
      ),
    );

    final pairs = grid.computeOverlapPairs();
    expect(pairs, [BroadPair(1, 2)]);

    final queryHits = grid.queryAabb(
      BroadAabb.fromCenterHalfExtents(
        const BroadVec3(8.2, 0.0, 0.0),
        const BroadVec3(0.4, 0.4, 0.4),
      ),
    );
    expect(queryHits.map((e) => e.id).toList(), [3]);
  });

  test('SweepAndPrune1D matches SpatialHashGrid3D overlap pairs and handles removals', () {
    final sap = SweepAndPrune1D(axis: SapAxis.x);
    final e1 = BroadEntry(
      id: 10,
      aabb: BroadAabb.fromCenterHalfExtents(
        const BroadVec3(0.0, 0.0, 0.0),
        const BroadVec3(1.0, 1.0, 1.0),
      ),
    );
    final e2 = BroadEntry(
      id: 20,
      aabb: BroadAabb.fromCenterHalfExtents(
        const BroadVec3(0.8, 5.0, 0.0),
        const BroadVec3(1.0, 1.0, 1.0),
      ),
    );
    final e3 = BroadEntry(
      id: 30,
      aabb: BroadAabb.fromCenterHalfExtents(
        const BroadVec3(0.8, 0.2, 0.0),
        const BroadVec3(1.0, 1.0, 1.0),
      ),
    );
    sap
      ..insertOrUpdate(e1)
      ..insertOrUpdate(e2)
      ..insertOrUpdate(e3);

    // e1 and e2 overlap on X but are separated on Y -> only (10, 30) overlaps in 3D!
    expect(sap.computeOverlapPairs(), [BroadPair(10, 30)]);
    expect(sap.remove(30), isTrue);
    expect(sap.computeOverlapPairs(), isEmpty);
  });

  test('SweptAabbToi prevents fast bullet tunneling and returns exact earliest TOI & normal', () {
    final projectile = BroadAabb.fromCenterHalfExtents(
      const BroadVec3(-5.0, 0.0, 0.0),
      const BroadVec3(0.5, 0.5, 0.5),
    );
    final wallNear = BroadEntry(
      id: 1,
      aabb: BroadAabb.fromCenterHalfExtents(
        const BroadVec3(0.0, 0.0, 0.0),
        const BroadVec3(0.5, 2.0, 2.0),
      ),
      isStatic: true,
    );
    final wallFar = BroadEntry(
      id: 2,
      aabb: BroadAabb.fromCenterHalfExtents(
        const BroadVec3(4.0, 0.0, 0.0),
        const BroadVec3(0.5, 2.0, 2.0),
      ),
      isStatic: true,
    );

    // Moves from x=-5 to x=+15 (delta = 20): hits wallNear (-0.5 face - 0.5 half = -1.0 -> dist 4.0 -> toi = 0.2).
    final hit = SweptAabbToi.sweepFirstHit(
      mover: projectile,
      delta: const BroadVec3(20.0, 0.0, 0.0),
      obstacles: [wallFar, wallNear],
    );

    expect(hit, isNotNull);
    expect(hit!.targetId, 1);
    expect(hit.toi, closeTo(0.2, 1e-6));
    expect(hit.normal.x, closeTo(-1.0, 1e-6));
    expect(hit.contactCenter.x, closeTo(-1.0, 1e-6));
  });
}
