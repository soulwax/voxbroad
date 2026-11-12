import 'dart:math' as math;

import 'math.dart';

/// Dynamic 3D uniform grid spatial hash partitioning for broadphase AABB queries & overlap pairs.
final class SpatialHashGrid3D {
  final double cellSize;
  final Map<int, BroadEntry> _entries = {};
  final Map<(int, int, int), Set<int>> _buckets = {};
  final Map<int, List<(int, int, int)>> _entryCells = {};

  SpatialHashGrid3D({this.cellSize = 2.0}) {
    if (cellSize <= 0.0) {
      throw ArgumentError.value(cellSize, 'cellSize', 'Must be > 0');
    }
  }

  int get entryCount => _entries.length;
  int get occupiedCellCount => _buckets.length;

  double get averageBucketLoad {
    if (_buckets.isEmpty) return 0.0;
    var totalRefs = 0;
    for (final bucket in _buckets.values) {
      totalRefs += bucket.length;
    }
    return totalRefs / _buckets.length;
  }

  BroadEntry? entryById(int id) => _entries[id];

  int _cellCoord(double v) => (v / cellSize).floor();

  void clear() {
    _entries.clear();
    _buckets.clear();
    _entryCells.clear();
  }

  void insertOrUpdate(BroadEntry entry) {
    remove(entry.id);
    _entries[entry.id] = entry;

    final minX = _cellCoord(entry.aabb.min.x);
    final maxX = _cellCoord(entry.aabb.max.x);
    final minY = _cellCoord(entry.aabb.min.y);
    final maxY = _cellCoord(entry.aabb.max.y);
    final minZ = _cellCoord(entry.aabb.min.z);
    final maxZ = _cellCoord(entry.aabb.max.z);

    final cells = <(int, int, int)>[];
    for (var cx = minX; cx <= maxX; cx++) {
      for (var cy = minY; cy <= maxY; cy++) {
        for (var cz = minZ; cz <= maxZ; cz++) {
          final key = (cx, cy, cz);
          cells.add(key);
          (_buckets[key] ??= <int>{}).add(entry.id);
        }
      }
    }
    _entryCells[entry.id] = cells;
  }

  bool remove(int id) {
    final removed = _entries.remove(id);
    if (removed == null) return false;
    final cells = _entryCells.remove(id);
    if (cells != null) {
      for (final key in cells) {
        final bucket = _buckets[key];
        if (bucket == null) continue;
        bucket.remove(id);
        if (bucket.isEmpty) {
          _buckets.remove(key);
        }
      }
    }
    return true;
  }

  /// Queries all entries overlapping [bounds], filtered optionally by [mask] & [excludeId].
  List<BroadEntry> queryAabb(
    BroadAabb bounds, {
    int mask = -1,
    int? excludeId,
  }) {
    final minX = _cellCoord(bounds.min.x);
    final maxX = _cellCoord(bounds.max.x);
    final minY = _cellCoord(bounds.min.y);
    final maxY = _cellCoord(bounds.max.y);
    final minZ = _cellCoord(bounds.min.z);
    final maxZ = _cellCoord(bounds.max.z);

    final visited = <int>{};
    final out = <BroadEntry>[];

    for (var cx = minX; cx <= maxX; cx++) {
      for (var cy = minY; cy <= maxY; cy++) {
        for (var cz = minZ; cz <= maxZ; cz++) {
          final bucket = _buckets[(cx, cy, cz)];
          if (bucket == null) continue;
          for (final id in bucket) {
            if (id == excludeId || !visited.add(id)) continue;
            final entry = _entries[id];
            if (entry == null) continue;
            if ((entry.layerMask & mask) == 0) continue;
            if (entry.aabb.intersects(bounds)) {
              out.add(entry);
            }
          }
        }
      }
    }
    return out;
  }

  /// Returns all unique overlapping entry pairs `(idA < idB)`.
  List<BroadPair> computeOverlapPairs({bool ignoreStaticStatic = true}) {
    final seen = <BroadPair>{};
    final pairs = <BroadPair>[];

    final sortedKeys = _entries.keys.toList()..sort();
    for (final id in sortedKeys) {
      final a = _entries[id]!;
      final hits = queryAabb(a.aabb, mask: a.layerMask, excludeId: id);
      for (final b in hits) {
        if ((a.layerMask & b.layerMask) == 0) continue;
        if (ignoreStaticStatic && a.isStatic && b.isStatic) continue;
        final pair = BroadPair(math.min(a.id, b.id), math.max(a.id, b.id));
        if (seen.add(pair)) {
          pairs.add(pair);
        }
      }
    }

    pairs.sort((p1, p2) {
      final c1 = p1.idA.compareTo(p2.idA);
      return c1 != 0 ? c1 : p1.idB.compareTo(p2.idB);
    });
    return pairs;
  }
}
