from pxr import Usd, UsdGeom
from collections import defaultdict

children_threshold : int = 100
def analyze_stage(usd_file):
    stage = Usd.Stage.Open(usd_file)
    if not stage:
        print("Failed to open USD stage:", usd_file)
        return

    total_prims = 0
    prim_type_counts = defaultdict(int)
    prims_per_depth = defaultdict(int)
    heavy_children = []

    def traverse(prim, depth=0):
        nonlocal total_prims
        total_prims += 1
        prim_type = prim.GetTypeName()
        prim_type_counts[prim_type] += 1
        prims_per_depth[depth] += 1

        children = list(prim.GetChildren())
        if len(children) > children_threshold:
            heavy_children.append((prim.GetPath(), len(children)))
        
        for child in children:
            traverse(child, depth + 1)

    traverse(stage.GetPseudoRoot())

    print(f"USD file: {usd_file}")
    print(f"Total prims: {total_prims}")
    print("Prim types and counts:")
    for t, c in prim_type_counts.items():
        print(f"  {t if t else 'None'}: {c}")

    print("Prims per depth:")
    for depth in sorted(prims_per_depth.keys()):
        print(f"  Depth {depth}: {prims_per_depth[depth]} prims")

    if heavy_children:
        print(f"\nPrim paths with unusually many children (> {children_threshold}):")
        for path, count in heavy_children:
            print(f"  {path}: {count} children")

if __name__ == "__main__":
    import sys
    if len(sys.argv) != 2:
        print("Usage: python get_prim_counts.py <usd_file>")
        sys.exit(1)
    
    usd_file = sys.argv[1]
    analyze_stage(usd_file)