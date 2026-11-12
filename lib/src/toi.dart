import 'dart:math' as math;

import 'math.dart';

/// Continuous swept-AABB collision Time-of-Impact contact result.
final class ToiHitResult {
  final int? targetId;
  final double toi;
  final BroadVec3 normal;
  final BroadVec3 contactCenter;
  final bool startedOverlapping;

  const ToiHitResult({
    this.targetId,
    required this.toi,
    required this.normal,
    required this.contactCenter,
    this.startedOverlapping = false,
  });
}

/// Minkowski-difference continuous swept-AABB Time-of-Impact (TOI) solver.
final class SweptAabbToi {
  const SweptAabbToi();

  /// Sweeps [mover] along relative displacement [delta] against [obstacle].
  /// Returns `null` if no collision occurs within `[0, 1]`.
  static ToiHitResult? sweepBox({
    required BroadAabb mover,
    required BroadVec3 delta,
    required BroadAabb obstacle,
    int? targetId,
  }) {
    final moverCenter = mover.center;
    final moverHalf = mover.halfExtents;
    final expandedMin = obstacle.min - moverHalf;
    final expandedMax = obstacle.max + moverHalf;

    // Initial overlap check at t == 0.
    if (moverCenter.x >= expandedMin.x &&
        moverCenter.x <= expandedMax.x &&
        moverCenter.y >= expandedMin.y &&
        moverCenter.y <= expandedMax.y &&
        moverCenter.z >= expandedMin.z &&
        moverCenter.z <= expandedMax.z) {
      return ToiHitResult(
        targetId: targetId,
        toi: 0.0,
        normal: BroadVec3.zero,
        contactCenter: moverCenter,
        startedOverlapping: true,
      );
    }

    var tEntry = 0.0;
    var tExit = 1.0;
    var nx = 0.0;
    var ny = 0.0;
    var nz = 0.0;

    bool testAxis(double origin, double d, double minB, double maxB, int axisIdx) {
      if (d.abs() < 1e-10) {
        return origin >= minB && origin <= maxB;
      }
      final invD = 1.0 / d;
      var t1 = (minB - origin) * invD;
      var t2 = (maxB - origin) * invD;
      var sign = -1.0;
      if (t1 > t2) {
        final tmp = t1;
        t1 = t2;
        t2 = tmp;
        sign = 1.0;
      }
      if (t1 > tEntry) {
        tEntry = t1;
        nx = axisIdx == 0 ? sign : 0.0;
        ny = axisIdx == 1 ? sign : 0.0;
        nz = axisIdx == 2 ? sign : 0.0;
      }
      tExit = math.min(tExit, t2);
      return tEntry <= tExit;
    }

    if (!testAxis(moverCenter.x, delta.x, expandedMin.x, expandedMax.x, 0)) return null;
    if (!testAxis(moverCenter.y, delta.y, expandedMin.y, expandedMax.y, 1)) return null;
    if (!testAxis(moverCenter.z, delta.z, expandedMin.z, expandedMax.z, 2)) return null;

    if (tEntry < 0.0 || tEntry > 1.0 || tEntry > tExit) return null;

    return ToiHitResult(
      targetId: targetId,
      toi: tEntry,
      normal: BroadVec3(nx, ny, nz),
      contactCenter: moverCenter + delta * tEntry,
      startedOverlapping: false,
    );
  }

  /// Sweeps [mover] along [delta] against a list of candidate [obstacles], returning the earliest TOI hit.
  static ToiHitResult? sweepFirstHit({
    required BroadAabb mover,
    required BroadVec3 delta,
    required Iterable<BroadEntry> obstacles,
    int? excludeId,
    int layerMask = -1,
  }) {
    ToiHitResult? earliest;
    for (final obs in obstacles) {
      if (obs.id == excludeId) continue;
      if ((obs.layerMask & layerMask) == 0) continue;
      final hit = sweepBox(
        mover: mover,
        delta: delta,
        obstacle: obs.aabb,
        targetId: obs.id,
      );
      if (hit == null) continue;
      if (earliest == null || hit.toi < earliest.toi) {
        earliest = hit;
      }
    }
    return earliest;
  }
}
