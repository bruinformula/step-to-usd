#include <iostream>

#include <gmsh.h>

#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/xform.h>
#include <pxr/base/tf/token.h>

int main(int argc, char** argv) {
    // gmsh test 
    gmsh::initialize();
    gmsh::model::add("dummy_model");

    int pointTag = gmsh::model::geo::addPoint(0.0, 0.0, 0.0);
    gmsh::model::geo::synchronize();

    std::cout << "Created Gmsh point with tag: " << pointTag << std::endl;

    gmsh::finalize();

    // usd test 
    pxr::UsdStageRefPtr stage = pxr::UsdStage::CreateInMemory();

    pxr::UsdGeomXform xform =
        pxr::UsdGeomXform::Define(stage, pxr::SdfPath("/Root"));

    std::cout << "Created USD stage and Xform: "
                << xform.GetPath().GetString() << std::endl;

    std::cout << "Dummy test completed successfully." << std::endl;
    return 0;
}
