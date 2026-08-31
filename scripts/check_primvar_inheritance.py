from pxr import Usd, Sdf
import sys

def test_step_label_attribute(stage_path, prototype_path):
    stage = Usd.Stage.Open(stage_path)

    proto_prim = stage.GetPrimAtPath(prototype_path)
    assert proto_prim.IsValid(), f"Prototype prim not found: {prototype_path}"

    # Check the attribute exists and has a value on the parent 
    label_attr = proto_prim.GetAttribute("cad:label")
    assert label_attr.IsValid(), \
        f"FAIL: cad:label attribute not found on {prototype_path}"
    
    label_value = label_attr.Get()
    assert label_value is not None, \
        f"FAIL: cad:label has no value on {prototype_path}"

    print(f"OK  parent {prototype_path}")
    print(f"    cad:label = '{label_value}'")

    # Verify children do NOT have their own copy
    children_to_check = ["Mesh", "Wireframe", "Sketch", "SketchPlanes"]

    for child_name in children_to_check:
        child_path = Sdf.Path(prototype_path).AppendChild(child_name)
        child_prim = stage.GetPrimAtPath(child_path)

        if not child_prim.IsValid():
            print(f"SKIP child {child_path} (not present)")
            continue

        # Regular attrs don't inherit — child should have no authored copy
        child_attr = child_prim.GetAttribute("cad:label")
        assert not child_attr.IsValid() or not child_attr.HasAuthoredValue(), \
            f"FAIL: {child_path} has its own cad:label — was this intentional?"

        # Walk up to find the label from a child's perspective
        ancestor = child_prim.GetParent()
        found_label = None
        while ancestor.IsValid():
            a = ancestor.GetAttribute("cad:label")
            if a.IsValid() and a.HasAuthoredValue():
                found_label = a.Get()
                break
            ancestor = ancestor.GetParent()

        assert found_label is not None, \
            f"FAIL: could not find cad:label by walking up from {child_path}"
        assert found_label == label_value, \
            f"FAIL: ancestor label '{found_label}' != expected '{label_value}'"

        print(f"OK  child  {child_path}")
        print(f"    cad:label reachable from ancestor: '{found_label}'")

    print("\nAll checks passed.")


if __name__ == "__main__":
    stage_path = sys.argv[1]
    proto_path = sys.argv[2]
    test_step_label_attribute(stage_path, proto_path)