# This file is intended for example purposes only, to show how to swap out 
# references to point to the same prototype in a non distructive way.

# Each prim in the assembly points to a prototype prim.
# both from performance and ease of use perspective, we 
# want to keep the same prototype reference for both left 
# and right parts, and just mirror the transform on the right side.

# for each assmebly part the original reference is 'deleted' and 
# the new reference is 'appended' to the reference list.

from pxr import Usd, UsdGeom, Sdf, Gf

# Mirrored Parts
def find_mirror_prims(target_stage: Usd.Stage):
    for prim in target_stage.TraverseAll():
        if "Mirror" in prim.GetName():
            print(prim.GetPath())

# References 
def print_references(prim_path: Sdf.Path):
    for layer in stage.GetLayerStack():
        prim_spec = layer.GetPrimAtPath(prim_path)
        if not prim_spec:
            continue
        ref_list = prim_spec.referenceList
        print(f"\nLayer: {layer.identifier}")
        print(f"  prependedItems : {ref_list.prependedItems}")
        print(f"  appendedItems  : {ref_list.appendedItems}")
        print(f"  deletedItems   : {ref_list.deletedItems}")
        print(f"  explicitItems  : {ref_list.explicitItems}")
        print(f"  orderedItems   : {ref_list.orderedItems}")

def get_prim_spec(prim_path: Sdf.Path, target_stage: Usd.Stage):
    for layer in target_stage.GetLayerStack():
        prim_spec = layer.GetPrimAtPath(prim_path)
        if prim_spec:
            return layer, prim_spec
    return None, None

def delete_reference(prim_path: Sdf.Path, delete_prim: Sdf.Path, target_stage: Usd.Stage):
    layer, prim_spec = get_prim_spec(prim_path, target_stage)
    if not prim_spec:
        print(f"Prim spec not found for {prim_path}")
        return
    ref_to_delete = Sdf.Reference(primPath=delete_prim)
    prim_spec.referenceList.deletedItems = \
        list(prim_spec.referenceList.deletedItems) + [ref_to_delete]

def add_reference(prim_path: Sdf.Path, add_prim: Sdf.Path, target_stage: Usd.Stage):
    layer, prim_spec = get_prim_spec(prim_path, target_stage)
    if not prim_spec:
        print(f"Prim spec not found for {prim_path}")
        return
    ref_to_add = Sdf.Reference(primPath=add_prim)
    prim_spec.referenceList.appendedItems = \
        list(prim_spec.referenceList.appendedItems) + [ref_to_add]

def swap_reference(target_prim_path: Sdf.Path, delete_prim: Sdf.Path, add_prim: Sdf.Path, target_stage: Usd.Stage):
    print(f"Swapping reference from {delete_prim} to {add_prim}")
    target_stage.OverridePrim(target_prim_path)
    delete_reference(target_prim_path, delete_prim, target_stage)
    add_reference(target_prim_path, add_prim, target_stage)

def get_xforms(prim: Usd.Prim, time: Usd.TimeCode = Usd.TimeCode.Default()) -> list[dict]:
    xformable = UsdGeom.Xformable(prim)
    return [
        {
            "name":      op.GetOpName(),
            "type":      op.GetOpType(),
            "precision": op.GetPrecision(),
            "value":     op.Get(time),
            "is_inverse": op.IsInverseOp(),
        }
        for op in xformable.GetOrderedXformOps()
    ]

def mirror_matrix(matrix: Gf.Matrix4d, axis: str = "z") -> Gf.Matrix4d:
    mirror = {
        "x": Gf.Matrix4d(-1,0,0,0,  0,1,0,0,  0,0,1,0,  0,0,0,1),
        "y": Gf.Matrix4d( 1,0,0,0,  0,-1,0,0, 0,0,1,0,  0,0,0,1),
        "z": Gf.Matrix4d( 1,0,0,0,  0,1,0,0,  0,0,-1,0, 0,0,0,1),
    }[axis]
    return mirror * matrix * mirror

def swap_and_mirror(
    prim_path: Sdf.Path,
    delete_prim: Sdf.Path,
    add_prim: Sdf.Path,
    source_stage: Usd.Stage,
    target_stage: Usd.Stage,
    axis: str = "z",
    time: Usd.TimeCode = Usd.TimeCode.Default()
):
    # swap the reference
    swap_reference(prim_path, delete_prim, add_prim, target_stage)

    over_prim = target_stage.OverridePrim(delete_prim)
    over_prim.SetActive(False)

    # read the matrix from source stage
    source_prim = source_stage.GetPrimAtPath(prim_path)
    if not source_prim.IsValid():
        print(f"  Source prim not found: {prim_path}")
        return

    source_xformable = UsdGeom.Xformable(source_prim)
    transform_op = next(
        (op for op in source_xformable.GetOrderedXformOps()
         if op.GetOpType() == UsdGeom.XformOp.TypeTransform),
        None
    )
    if not transform_op:
        print(f"  No TypeTransform found on: {prim_path}")
        return

    # copy matrix and apply -1 scale on axis
    matrix = transform_op.Get(time)

    scale = {"x": Gf.Vec3d(-1, 1, 1),
             "y": Gf.Vec3d( 1,-1, 1),
             "z": Gf.Vec3d( 1, 1,-1)}[axis]

    scaled_matrix = matrix * Gf.Matrix4d().SetScale(scale)

    # write into override stage
    override_prim = target_stage.GetPrimAtPath(prim_path)
    if not override_prim.IsValid():
        override_prim = target_stage.OverridePrim(prim_path)

    UsdGeom.Xformable(override_prim).AddTransformOp().Set(scaled_matrix)
    print(f"  Copied and scaled matrix by {scale} on {prim_path}")


if __name__ == "__main__":
    import os 
    working_dir = os.getcwd()

    stage = Usd.Stage.Open(f"{working_dir}/../../model-v1/LOD/model-v1-LOD-low-prototypes.usdc")
    over_stage = Usd.Stage.Open(f"{working_dir}/../../model-v1/overrides/overrides-mirror.usdc")

    # Whisker
    target_prim_path = Sdf.Path("/Mk11/Assembly/AD11_TA_001_Aero_Top_Assembly_1__612642da/AD11_BW_400_MASTER_Top_1__27080386/whisker_L_1__0dc72602")
    original_reference = Sdf.Path("/Mk11/Prototypes/whisker_L_1__7bfe5811")
    new_reference = Sdf.Path("/Mk11/Prototypes/whisker_test_1__74fe4d0c")
    swap_and_mirror(target_prim_path, original_reference, new_reference, stage, over_stage)

    # Side Panel
    target_prim_path = Sdf.Path("/Mk11/Assembly/AD11_TA_001_Aero_Top_Assembly_1__612642da/AD11_BW_400_MASTER_Top_1__27080386/AD11_BW_402_01_C_Side_Panel_L_1__09c71fb6")
    original_reference = Sdf.Path("/Mk11/Prototypes/AD11_BW_402_01_C_Side_Panel_L_1__8320f558")
    new_reference = Sdf.Path("/Mk11/Prototypes/AD11_BW_402_01_C_Side_Panel_R1_3__82638037")
    swap_and_mirror(target_prim_path, original_reference, new_reference, stage, over_stage)

    # Front Wing Element 1
    target_prim_path = Sdf.Path("/Mk11/Assembly/AD11_TA_001_Aero_Top_Assembly_1__612642da/AD11_FW_001_FW_TOP_ASSEMBLY_FDR_1__290806ac/MirrorAD11_FW_221_2_C_E3_IB_2__b4f081e7")
    original_reference = Sdf.Path("/Mk11/Prototypes/MirrorAD11_FW_221_2_C_E3_IB_2__e7912325")
    new_reference = Sdf.Path("/Mk11/Prototypes/AD11_FW_221_2_C_E3_IB_1__72ae800a")
    swap_and_mirror(target_prim_path, original_reference, new_reference, stage, over_stage)

    # Front Wing Element 2  
    target_prim_path = Sdf.Path("/Mk11/Assembly/AD11_TA_001_Aero_Top_Assembly_1__612642da/AD11_FW_001_FW_TOP_ASSEMBLY_FDR_1__290806ac/MirrorAD11_FW_211_C_E2_IB_3__69e3ffc9")
    original_reference = Sdf.Path("/Mk11/Prototypes/MirrorAD11_FW_211_C_E2_IB_3__ecb17eaf")
    new_reference = Sdf.Path("/Mk11/Prototypes/AD11_FW_211_C_IB_E2_1__6fae7b51")
    swap_and_mirror(target_prim_path, original_reference, new_reference, stage, over_stage)

    # Front Wing Element 3
    target_prim_path = Sdf.Path("/Mk11/Assembly/AD11_TA_001_Aero_Top_Assembly_1__612642da/AD11_FW_001_FW_TOP_ASSEMBLY_FDR_1__290806ac/MirrorAD11_FW_212_C_E2_OB_3__23e8e1df")
    original_reference = Sdf.Path("/Mk11/Prototypes/MirrorAD11_FW_212_C_E2_OB_3__f293730d")
    new_reference = Sdf.Path("/Mk11/Prototypes/AD11_FW_212_C_E2_OB_1__68ac31b5")
    swap_and_mirror(target_prim_path, original_reference, new_reference, stage, over_stage)

    # Front Wing Main Element
    target_prim_path = Sdf.Path("/Mk11/Assembly/AD11_TA_001_Aero_Top_Assembly_1__612642da/AD11_FW_001_FW_TOP_ASSEMBLY_FDR_1__290806ac/MirrorAD11_FW_200_C_MAINPLANE_3__68e3fe36")
    original_reference = Sdf.Path("/Mk11/Prototypes/MirrorAD11_FW_200_C_MAINPLANE_3__ebb17d1c")
    new_reference = Sdf.Path("/Mk11/Prototypes/AD11_FW_200_C_MAINPLANE_2__71ae7e77")
    swap_and_mirror(target_prim_path, original_reference, new_reference, stage, over_stage)

    # Front Wing Right Endplate
    target_prim_path = Sdf.Path("/Mk11/Assembly/AD11_TA_001_Aero_Top_Assembly_1__612642da/AD11_FW_001_FW_TOP_ASSEMBLY_FDR_1__290806ac/MirrorAD11_FW_203_C_ENDPLATE_OUTER_CURVED_2__b5f0837a")
    original_reference = Sdf.Path("/Mk11/Prototypes/MirrorAD11_FW_203_C_ENDPLATE_OUTER_CURVED_2__e6912192")
    new_reference = Sdf.Path("/Mk11/Prototypes/AD11_FW_203_C_ENDPLATE_OUTER_CURVED_1__6dae782b")
    swap_and_mirror(target_prim_path, original_reference, new_reference, stage, over_stage)

    # Front Wing Right Endplate Skirt
    target_prim_path = Sdf.Path("/Mk11/Assembly/AD11_TA_001_Aero_Top_Assembly_1__612642da/AD11_FW_001_FW_TOP_ASSEMBLY_FDR_1__290806ac/MirrorAD11_FW_232_C_CURVED_ENDPLATE_FOOT_2__b1f07d2e")
    original_reference = Sdf.Path("/Mk11/Prototypes/MirrorAD11_FW_232_C_CURVED_ENDPLATE_FOOT_2__e5911fff")
    new_reference = Sdf.Path("/Mk11/Prototypes/AD11_FW_232_C_CURVED_ENDPLATE_FOOT_1__ee936cc1")
    swap_and_mirror(target_prim_path, original_reference, new_reference, stage, over_stage)

    print(over_stage.GetRootLayer().ExportToString())
