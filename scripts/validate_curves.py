from pxr import Usd, UsdGeom
import os

mk11_root = os.getenv("MK11_ROOT")
if mk11_root is None:
    raise RuntimeError("MK11_ROOT environment variable is not set.")

stage = Usd.Stage.Open(f"{mk11_root}/mk11/mk11-model/model-v3/LOD/model-v3-LOD-low-prototypes-sandwich.usda")

primvar_counts = {}

prototypes = stage.GetPrimAtPath("/Mk11/Prototypes")
prototypes.SetActive(True)

for proto in prototypes.GetAllChildren():
    for container in proto.GetAllChildren():
        if container.GetName() not in ("Wireframe", "Sketch"):
            continue
        for prim in Usd.PrimRange(container):
            if prim.IsA(UsdGeom.BasisCurves) or prim.IsA(UsdGeom.NurbsCurves):
                pvAPI = UsdGeom.PrimvarsAPI(prim)
                for pv in pvAPI.GetPrimvars():
                    if "continuityType" in pv.GetPrimvarName():
                        val = pv.Get()
                        if val:
                            primvar_counts[str(prim.GetPath())] = len(val)

sorted_counts = dict(sorted(primvar_counts.items(), key=lambda x: x[1], reverse=False))

disable_threshold = True
#threshold = 65535
threshold = 32768
# RenderMan has a limit on the number of points that a curve with primvars can have, which is 65535 (16 bit int max)

for path, count in sorted_counts.items():
    if count > threshold or disable_threshold:
        print(f"{count:>8}  {path}")
