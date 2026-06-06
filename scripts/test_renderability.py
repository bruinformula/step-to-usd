import os
import math
import subprocess
from pxr import Usd, UsdGeom, UsdRender, Gf, Sdf

RENDER_PRODUCT_PATH = "/Render/RenderProduct"
MK11_PRIM = "/Mk11"


def prim_to_filename(prim: Usd.Prim) -> str:
    return prim.GetPath().pathString.strip("/").replace("/", "_") + ".png"


def fit_camera_to_prim(stage: Usd.Stage, prim: Usd.Prim) -> None:
    bbox_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]
    )
    bbox = bbox_cache.ComputeWorldBound(prim)
    bound = bbox.ComputeAlignedBox()

    if bound.IsEmpty():
        print(f"  [WARN] Empty bbox for {prim.GetPath()}, skipping camera fit")
        return

    centroid = (bound.GetMin() + bound.GetMax()) / 2.0
    size = (bound.GetMax() - bound.GetMin()).GetLength()

    focal_length = 35.0
    v_aperture = 20.25
    fov_v = 2.0 * math.atan((v_aperture * 0.5) / focal_length)
    dist = (size * 0.5) / math.tan(fov_v * 0.5) * 1.1

    camera_pos = Gf.Vec3d(centroid[0], centroid[1], centroid[2] + dist)

    camera_prim = UsdGeom.Camera.Get(stage, "/Render/Camera")
    xform_api = UsdGeom.XformCommonAPI(camera_prim)
    xform_api.SetTranslate(camera_pos)
    xform_api.SetRotate(Gf.Vec3f(0, 0, 0), UsdGeom.XformCommonAPI.RotationOrderXYZ)


def setup_render_settings(stage: Usd.Stage, output_png: str) -> None:
    product = UsdRender.Product.Get(stage, RENDER_PRODUCT_PATH)
    product.GetProductNameAttr().Set(output_png)


def render_stage(stage: Usd.Stage, mask_prim: Usd.Prim, output_png: str, indent: int) -> int:
    cmd = [
        "usdrecord",
        str(stage.GetRootLayer().realPath),
        "--mask", f"{mask_prim.GetPath()},/Render",
        "--camera", "/Render/Camera",
        "--renderer", "RenderMan XPU",
        output_png,
    ]

    process = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
    )

    stdout_lines = []
    stderr_lines = []

    # Stream stdout
    for line in process.stdout:
        line = line.rstrip()
        stdout_lines.append(line)
        print(f"{' ' * indent}    {line}", flush=True)

    # Stream stderr
    for line in process.stderr:
        line = line.rstrip()
        stderr_lines.append(line)
        print(f"{' ' * indent}    {line}", flush=True)

    process.wait()
    return process.returncode


def render(stage: Usd.Stage, prim: Usd.Prim, problem_prims: list, indent: int) -> None:
    pad = '  ' * indent
    output_png = os.path.join(output_dir, prim_to_filename(prim))

    if os.path.exists(output_png):
        print(f"{pad}Already rendered: {prim.GetPath()}, skipping.")
        return

    setup_render_settings(stage, output_png)
    fit_camera_to_prim(stage, prim)
    stage.Save()

    print(f"{pad}Rendering: {prim.GetPath()}")
    return_code = render_stage(stage, prim, output_png, indent)

    if return_code == 0:
        return

    # Clean up partial file left by failed render
    if os.path.exists(output_png):
        os.remove(output_png)

    print(f"{pad}[ERROR] Failed: {prim.GetPath()}")

    children = list(prim.GetAllChildren())

    if not children:
        print(f"{pad}^ LEAF PROBLEM PRIM (no children to recurse into): {prim.GetPath()}")
        problem_prims.append(str(prim.GetPath()))
        return

    print(f"{pad}Recursing into {len(children)} children...")

    child_problem_count_before = len(problem_prims)

    for i, child_prim in enumerate(children):
        print(f"{pad}  [{i+1}/{len(children)}] {child_prim.GetPath()}")
        render(stage, child_prim, problem_prims, indent + 1)

    # If none of the children had problems, the parent assembly itself is the culprit
    if len(problem_prims) == child_problem_count_before:
        print(f"{pad}^ ASSEMBLY PROBLEM PRIM (children all OK, parent fails): {prim.GetPath()}")
        problem_prims.append(str(prim.GetPath()))


if __name__ == "__main__":

    mk11_root = os.getenv("MK11_ROOT")
    if mk11_root is None:
        raise RuntimeError("MK11_ROOT environment variable is not set.")

    LOD = "low"
    home_dir = os.path.expanduser("~")
    output_dir = f"{home_dir}/Desktop/renderman-test-output-{LOD}"
    os.makedirs(output_dir, exist_ok=True)

    render_stage_path = f"{mk11_root}/render/render-renderman-test/render-renderman-test-{LOD}.usda"

    print(f"Opening stage: {render_stage_path}")
    stage = Usd.Stage.Open(render_stage_path)
    if not stage:
        raise RuntimeError(f"Failed to open stage: {render_stage_path}")

    root_prim = stage.GetPrimAtPath(f"{MK11_PRIM}/Assembly")
    if not root_prim.IsValid():
        raise RuntimeError(f"Could not find {MK11_PRIM}/Assembly prim on stage.")

    children = list(root_prim.GetAllChildren())
    if not children:
        raise RuntimeError(f"{MK11_PRIM}/Assembly has no children to render.")

    print(f"Found {len(children)} prototype(s).")

    problem_prims = []

    render(stage, root_prim, problem_prims, 0)

    print(f"\nFound {len(problem_prims)} problem prim(s):")
    for prim_path in problem_prims:
        print(f"  {prim_path}")

    print("\nDone.")