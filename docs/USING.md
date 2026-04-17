# Table of Contents
- [A Lay Engineer's Introduction](#a-lay-engineers-introduction)
- [Usage](#usage)

# A Lay Engineer's Introduction

Boundary representation (BRep) geometry is a staple of CAD. It describes smooth, parametric surfaces that are great for precise measurements and analytical operations. Some workflows, however, prefer alternative geometry representations. If you've done FEA or 3D printing, you've seen this firsthand — the model is chopped into a mesh of triangles, discretizing an otherwise continuous surface. Simulation tools prefer meshes for numerical work, and high-end renderers are no different. Before you get a beautiful final image, the model has to be tessellated.

The catch is that CAD models and meshes are fundamentally different, so they live in different file formats. Formats like `.SLDPRT`, `.SLDASM`, and **STEP** store BRep data, while formats like **STL** or **OBJ** store triangle meshes — but without much else.

For rendering, that's not enough. **USD (Universal Scene Description)** acts like a "STEP file for movies," bundling mesh geometry with materials, lighting, cameras, and more.

The short version: start with BRep (STEP), turn it into a mesh, then package it in USD for rendering. View (EXPORTING.md)[EXPORTING.md] on extracting a `.STEP` model from a `.SLDASM`

---
# Usage

`StepUsdTesselate` takes a USD file containing one or more `StepContainer` prims and tessellates them into renderable geometry. All meshing parameters are read directly from the USD composed Usd Stage.

```bash
$ StepUsdTesselate -h
    StepUsdTesselate -- Meshes all StepContainer prims in a Usd scene
    Options: 
        -i, --inputUsdFile <path>        Path to the input Usd file. 
        -p, --prim <sdfPath>             Only tessellate the prim at this path including variants. Can be multiple paths.
        -q, --quiet                      Suppress all output.
        -v, --verbose                    Prints like everything.
        -h, --help                       Prints this message.

        usage: StepUsdTesselate -i <path> [options] 
```

#### STEP Parsing

The first thing `StepUsdTesselate` does is parse the STEP file. This is the slowest part of the pipeline by a significant margin. STEP files are dense ASCII exchange files that encode full BRep topology. For large assemblies, this is parsing them is unavoidably expensive.

Once parsed, step-to-usd writes an XBF file alongside the STEP file:

```
model1.STEP
model1.xbf   <- same path. same filename.
```

The XBF is OpenCascade's native binary format for its internal BRep representation (`TopoDS_Shape`). On subsequent runs, step-to-usd detects the XBF and deserializes it directly, bypassing the STEP parser entirely. For a large car assembly, this is substantially faster by roughly an order of magnitude.

#### Output Structure

After parsing, step-to-usd generates a directory named after the model and writes all output into it:

```
model2/
├── container.usda              <- your input file
├── model2.STEP
├── model2.xbf
└── model2/
    ├── model2-assembly.usdc    <- generated: the flat assembly prim
    ├── model2-prototypes.usdc  <- generated: tessellated prototype geometry
    └── model2-sandwich.usda    <- generated: sandwich layer
```

For models with variants, the prototypes are written per-variant.

```
model1/
├── container.usda              <- your input file
├── model1.STEP
├── model1.xbf
└── model1/
    ├── model1-assembly.usdc
    ├── model1-assembly-sandwich.usda
    └── LOD/
        ├── model1-LOD-high-prototypes.usdc 
        ├── model1-LOD-high-prototypes-sandwich.usda 
        ├── model1-LOD-low-prototypes.usdc
        └── model1-LOD-high-prototypes-sandwich.usda 
```

#### Composition Arc Walkthrough

Each `StepContainer` composes as a small stack of layers. From strongest to weakest opinion:

```
your input file (container_variant.usda)
└── payload -> model1-LOD-high-prototypes.usdc    (tessellated geometry)
      └── sublayer -> model1-container.usda       
            └── sublayer -> model1-assembly.usdc  (assembly structure)
```

Working from the bottom up:

**`model1-assembly.usdc`** contains the assembly prim — the flat list of prototype prims as they exist in the original STEP file, without any tessellation opinions applied yet.

**`model1-sandwich.usda`** (or `model2-*-*-*-sandwich.usda` in the variant case) is am intermediate that sublayers the assembly. This is  a place to insert overrides and opinions that apply before the prototypes are added. Useful things to author here include suppressing parts that shouldn't be rendered, mirror overrides, or material assignments that need to land below the prototype geometry in the composition stack. In the variant case, this is split into the assembly and the prototypes.

**`model1-prototypes.usdc`** (or `model1-*-*-prototypes.usdc` in the variant case) is brought in as a payload from your input file. It contains the tessellated mesh, wireframe, and sketch geometry for every prototype in that variant.

**Your input file** sits at the top of the stack. Variant selections, per-prototype overrides, and `step:defaultParams` relationships all live here and win over everything below.

### Selective Remeshing

By default, `StepUsdTesselate` processes every `StepContainer` in the input file. The `-p` flag narrows the scope to specific prims and specific variants, which is useful when iterating on a single part without paying the cost of a full rerun.

```bash
StepUsdTesselate -i scene.usd -p /World/Wonderful/Prototypes/rod_1__78316e9d # does everything

StepUsdTesselate -i scene.usd -p /World/Wonderful{LOD=high}Prototypes/rod_1__78316e9d # remesh draft, final, and default

StepUsdTesselate -i scene.usd -p /World/Wonderful{LOD=high}Prototypes/rod_1__78316e9d{quality=draft}

StepUsdTesselate -i scene.usd -p /World/Wonderful/Prototypes/rod_1__78316e9d{quality=draft} # remeshed in both LOD=high and low

```
Multiple `-p` flags can be combined to target a set of prims simultaneously.

### Wireframe and Sketch Geometry

In addition to surface meshes, step-to-usd generates curve geometry from BRep surface boundaries (the classic wireframe look) and sketch primitives. These are controlled via the `step:wireframe*` and `step:sketch*` attribute families on `StepTessellationOptions`.

Both support `linear` and `cubic` curve types. Sampling strategy is either `underlying` (uses the prexisting mesh vertices) or `resampled` (resamples to a target curve independently of the mesh). Setting `step:wireframeType` or `step:sketchType` to `"none"` disables that geometry type entirely.

Tessellation quality is controlled through USD attributes on `StepTessellationOptions` prims. The most important are `step:meshLinearDeflection` and `step:meshAngularDeflection` — smaller values produce finer meshes at the cost of performance ( read more here). Both are expressed as a fraction of the bounding-box diagonal, so they scale automatically with model size.

Parameters are inherited through a class prim, `/Part`, that also defines `Mesh`, `Sketch`, and `Wireframe` subprims. so you can set sensible defaults at the assembly level and override selectively on individual prototypes. The `step:defaultParams` relationship on `StepPrototypes` points to the `StepTessellationOptions` prim that serves as the fallback for the whole file.

### Variants

USD variant sets are a natural fit for LOD. A `StepContainer` can carry a `variantSet` that swaps between independently tessellated payloads, each backed by its own prototypes file and referencing a different `StepTessellationOptions` prim.

```usda
def StepTessellationOptions "HighOptions" {
    double step:meshAngularDeflection = 0.1
    double step:meshLinearDeflection = 0.1
}
def StepTessellationOptions "LowOptions" {
    double step:meshAngularDeflection = 10
    double step:meshLinearDeflection = 10
}

def StepContainer "Wonderful" (
    variants = { string LOD = "low" }
    prepend variantSets = "LOD"
) {
    asset step:sourceAsset = @../step/model.STEP@
    variantSet "LOD" = {
        "high" (
            prepend payload = @model/LOD/model-LOD-high-prototypes.usdc@
        ) {
            over StepPrototypes "Prototypes" {
                rel step:defaultParams = </HighOptions>
            }
        }
        "low" (
            prepend payload = @model/LOD/model-LOD-low-prototypes.usdc@
        ) {
            over StepPrototypes "Prototypes" {
                rel step:defaultParams = </LowOptions>
            }
        }
    }
}
```
Variants can also be applied at the prototype level independently of any container-level LOD, giving you per-part quality control without affecting the rest of the assembly.