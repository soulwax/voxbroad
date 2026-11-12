import 'dart:math' as math;

/// Lightweight immutable 3D vector for `voxbroad`.
final class BroadVec3 {
  final double x;
  final double y;
  final double z;

  const BroadVec3(this.x, this.y, this.z);

  static const zero = BroadVec3(0.0, 0.0, 0.0);

  BroadVec3 operator +(BroadVec3 o) => BroadVec3(x + o.x, y + o.y, z + o.z);
  BroadVec3 operator -(BroadVec3 o) => BroadVec3(x - o.x, y - o.y, z - o.z);
  BroadVec3 operator *(double s) => BroadVec3(x * s, y * s, z * s);

  double get lengthSquared => x * x + y * y + z * z;
  double get length => math.sqrt(lengthSquared);
}

/// Axis-aligned bounding box defined by inclusive [min] and [max] corners.
final class BroadAabb {
  final BroadVec3 min;
  final BroadVec3 max;

  const BroadAabb(this.min, this.max);

  factory BroadAabb.fromCenterHalfExtents(BroadVec3 center, BroadVec3 halfExtents) =>
      BroadAabb(
        BroadVec3(
          center.x - halfExtents.x.abs(),
          center.y - halfExtents.y.abs(),
          center.z - halfExtents.z.abs(),
        ),
        BroadAabb._absMax(center, halfExtents),
      );

  static BroadVec3 _absMax(BroadVec3 center, BroadVec3 halfExtents) => BroadVec3(
        center.x + halfExtents.x.abs(),
        center.y + halfExtents.y.abs(),
        center.z + halfExtents.z.abs(),
      );

  BroadVec3 get center => BroadVec3(
        (min.x + max.x) * 0.5,
        (min.y + max.y) * 0.5,
        (min.z + max.z) * 0.5,
      );

  BroadVec3 get halfExtents => BroadVec3(
        (max.x - min.x) * 0.5,
        (max.y - min.y) * 0.5,
        (max.z - min.z) * 0.5,
      );

  BroadAabb expanded(double margin) => BroadAabb(
        BroadVec3(min.x - margin, min.y - margin, min.z - margin),
        BroadVec3(max.x + margin, max.y + margin, max.z + margin),
      );

  bool intersects(BroadAabb other) =>
      min.x <= other.max.x &&
      max.x >= other.min.x &&
      min.y <= other.max.y &&
      max.y >= other.min.y &&
      min.z <= other.max.z &&
      max.z >= other.min.z;
}

/// Indexed broadphase proxy entry combining integer [id], [layerMask], [aabb], and [isStatic].
final class BroadEntry {
  final int id;
  final BroadAabb aabb;
  final int layerMask;
  final bool isStatic;

  const BroadEntry({
    required this.id,
    required this.aabb,
    this.layerMask = -1,
    this.isStatic = false,
  });
}

/// Canonical ordered candidate pair `(idA < idB)`.
final class BroadPair {
  final int idA;
  final int idB;

  BroadPair(int a, int b)
      : idA = a <= b ? a : b,
        idB = a <= b ? b : a;

  @override
  bool operator ==(Object other) =>
      other is BroadPair && idA == other.idA && idB == other.idB;

  @override
  int get hashCode => Object.hash(idA, idB);

  @override
  String toString() => 'BroadPair($idA, $idB)';
}
