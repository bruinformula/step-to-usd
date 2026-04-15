# step-to-usd
## Overview 

step-to-usd is a high-performance system for meshing STEP files into renderable USD assets. For more info visit [INFO.md](INFO.md), but the jist is as follows:

Meshing parameters are passed as USD attributes, with support for variant parameters on entire assemblies and invididual prototypes. This purist representation composes cleanly with USD.

Surface meshes aren't the only geometric primitive in town. Wireframes extracted from BRep surface boundaries and sketch primitives are generated alongside them - a launching point for tremendous look development.

Iteration is fast. After a STEP file is parsed, step-to-usd caches the internal OpenCascade representation — cutting rerun initiaization time. Better still, users can target individual prims for remeshing rather than reprocessing the entire assembly every time.

Cross-platform, no asterisks. Nothing in this project is tied to a specific OS or execution environment.

## Getting Started

### Dependencies

**OpenCascade** — ships with most package managers, including `brew` and `dnf`.

**OpenUSD** — as of this writing, USD isn't available via any package manager and must be [built from source](https://github.com/PixarAnimationStudios/OpenUSD/blob/dev/BUILDING.md).

### Basic Example

Create a scene like the one below and run `StepConvertUsd -i your_scene.usd`. Example models can be found in `test/`. See [INFO.md](INFO.md) for more.

```usda
#usda 1.0
(
    defaultPrim = "WonderfulModel"
    metersPerUnit = 0.001
    upAxis = "Z"
)
def StepTessellationOptions "DefaultOptions" {
    double step:meshAngularDeflection = 0.1
    double step:meshLinearDeflection = 0.1
    # other options...
}
def StepContainer "WonderfulModel" {
    asset step:sourceAsset = @model.STEP@
    def StepPrototypes "Prototypes" {
        rel step:defaultParams = </DefaultOptions>
    }
}
```