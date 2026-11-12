import 'math.dart';

/// Axis selection for [SweepAndPrune1D].
enum SapAxis { x, y, z }

final class _SapEndpoint {
  final int entryId;
  final bool isMin;
  double value;

  _SapEndpoint(this.entryId, this.isMin, this.value);
}

/// Temporal-coherence-friendly 1D Sweep-and-Prune broadphase with exact 3D AABB confirmation.
final class SweepAndPrune1D {
  final SapAxis axis;
  final Map<int, BroadEntry> _entries = {};
  final List<_SapEndpoint> _endpoints = [];

  SweepAndPrune1D({this.axis = SapAxis.x});

  int get entryCount => _entries.length;

  (double, double) _axisInterval(BroadAabb box) {
    switch (axis) {
      case SapAxis.x:
        return (box.min.x, box.max.x);
      case SapAxis.y:
        return (box.min.y, box.max.y);
      case SapAxis.z:
        return (box.min.z, box.max.z);
    }
  }

  void clear() {
    _entries.clear();
    _endpoints.clear();
  }

  void insertOrUpdate(BroadEntry entry) {
    final existed = _entries.containsKey(entry.id);
    _entries[entry.id] = entry;
    final (minV, maxV) = _axisInterval(entry.aabb);
    if (!existed) {
      _endpoints.add(_SapEndpoint(entry.id, true, minV));
      _endpoints.add(_SapEndpoint(entry.id, false, maxV));
    } else {
      for (final ep in _endpoints) {
        if (ep.entryId == entry.id) {
          ep.value = ep.isMin ? minV : maxV;
        }
      }
    }
  }

  bool remove(int id) {
    if (_entries.remove(id) == null) return false;
    _endpoints.removeWhere((ep) => ep.entryId == id);
    return true;
  }

  void _insertionSortEndpoints() {
    for (var i = 1; i < _endpoints.length; i++) {
      final key = _endpoints[i];
      var j = i - 1;
      while (j >= 0 &&
          (_endpoints[j].value > key.value ||
              (_endpoints[j].value == key.value && !_endpoints[j].isMin && key.isMin))) {
        _endpoints[j + 1] = _endpoints[j];
        j--;
      }
      _endpoints[j + 1] = key;
    }
  }

  /// Evaluates SAP sorted endpoints and emits unique 3D-verified overlap pairs.
  List<BroadPair> computeOverlapPairs({bool ignoreStaticStatic = true}) {
    _insertionSortEndpoints();
    final active = <int>{};
    final pairs = <BroadPair>[];

    for (final ep in _endpoints) {
      if (ep.isMin) {
        final a = _entries[ep.entryId];
        if (a != null) {
          for (final otherId in active) {
            final b = _entries[otherId];
            if (b == null) continue;
            if ((a.layerMask & b.layerMask) == 0) continue;
            if (ignoreStaticStatic && a.isStatic && b.isStatic) continue;
            if (a.aabb.intersects(b.aabb)) {
              pairs.add(BroadPair(a.id, b.id));
            }
          }
        }
        active.add(ep.entryId);
      } else {
        active.remove(ep.entryId);
      }
    }

    pairs.sort((p1, p2) {
      final c1 = p1.idA.compareTo(p2.idA);
      return c1 != 0 ? c1 : p1.idB.compareTo(p2.idB);
    });
    return pairs;
  }
}
