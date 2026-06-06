from pxr import Usd, UsdUtils, UsdGeom
import os
from collections import defaultdict

mk11_root = os.getenv("MK11_ROOT")
if mk11_root is None:
    raise RuntimeError("MK11_ROOT environment variable is not set.")

lod: str = "low"
stage = Usd.Stage.Open(f"{mk11_root}/mk11/mk11-model/model-v3/LOD/model-v3-LOD-{lod}-prototypes-sandwich.usda")

stats = UsdUtils.ComputeUsdStageStats(stage)
K = UsdUtils.UsdStageStatsKeys

print("Stage Stats")
print(f"  Total Prims         : {stats.get(K.totalPrimCount, 0):,}")
print(f"  Total Instances     : {stats.get(K.totalInstanceCount, 0):,}")
print(f"  Prototypes          : {stats.get(K.prototypeCount, 0):,}")
print(f"  Used Layers         : {stats.get(K.usedLayerCount, 0):,}")

primary = stats.get(K.primary, {})
prim_counts = primary.get(K.primCounts, {})
print(f"\nPrimary Tree")
print(f"  Active Prims      : {prim_counts.get(K.activePrimCount, 0):,}")
print(f"  Inactive Prims    : {prim_counts.get(K.inactivePrimCount, 0):,}")
print(f"  Instances         : {prim_counts.get(K.instanceCount, 0):,}")
print(f"\n  Prim Counts by Type:")
by_type = primary.get(K.primCountsByType, {})
for prim_type, count in sorted(by_type.items(), key=lambda x: -x[1]):
    print(f"    {prim_type:<30} {count:,}")


# Prototype breakdown
print("\nPrototype Subtree Breakdown")
prototypes = stats.get(K.prototypes, {})
proto_rows = []
for proto_path, proto_data in prototypes.items():
    if not isinstance(proto_data, dict):
        continue
    pc   = proto_data.get(K.primCounts, {})
    bt   = proto_data.get(K.primCountsByType, {})
    proto_rows.append((
        proto_path,
        pc.get(K.activePrimCount, 0),
        bt.get("Mesh", 0),
        bt.get("BasisCurves", 0),
    ))

proto_rows.sort(key=lambda x: -x[1])
print(f"  {'Path':<45} {'Active':>8} {'Meshes':>8} {'Curves':>8}")
print(f"  {'-'*45} {'-'*8} {'-'*8} {'-'*8}")
for path, active, meshes, curves in proto_rows[:30]:
    print(f"  {str(path):<45} {active:>8,} {meshes:>8,} {curves:>8,}")

# Find prim paths with most direct children 
print("\nTop 15 Subtree Hotspots")
child_counts = []
for prim in stage.TraverseAll():
    n = len(prim.GetChildren())
    if n > 50:
        child_counts.append((prim.GetPath(), prim.GetTypeName(), n))

child_counts.sort(key=lambda x: -x[2])
for path, ptype, n in child_counts[:15]:
    print(f"  {str(path):<60} {ptype:<20} children={n:,}")