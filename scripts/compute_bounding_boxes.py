from pxr import Usd, UsdGeom
import os 

if __name__ == "__main__":

    mk11_root = os.getenv("MK11_ROOT")
    if mk11_root is None:
        raise RuntimeError("MK11_ROOT environment variable is not set.")

    LOD = "low"
    home_dir = os.path.expanduser("~")
    output_dir = f"{home_dir}/Desktop/renderman-test-output-{LOD}"
    os.makedirs(output_dir, exist_ok=True)

    render_stage_path = f"{mk11_root}/mk11/mk11-model/model-v3/LOD/model-v3-LOD-{LOD}-prototypes.usdc"

    stage = Usd.Stage.Open(render_stage_path)

    prototypes = stage.GetPrimAtPath("/Mk11/Prototypes")

    for prim in prototypes.GetChildren():
        mesh = stage.GetPrimAtPath(prim.GetPath().AppendChild('Mesh'))
        if not mesh.IsA(UsdGeom.Boundable):
            continue

        boundable = UsdGeom.Boundable(mesh)
        extent_attr = boundable.GetExtentAttr()

        # Only author if extent is not already set
        if not extent_attr or not extent_attr.HasAuthoredValue():
            print(f"{mesh.GetPath()} is missing extent. Computing and authoring extent...")
            '''
            extents = UsdGeom.Boundable.ComputeExtentFromPlugins(boundable, Usd.TimeCode.Default())
            
            if extents is not None:
                boundable.CreateExtentAttr(extents)
                print(f"Authored extent on: {prim.GetPath()}")
            else:
                print(f"WARNING: Could not compute extent for {prim.GetPath()}")
            '''
    #stage.Save()