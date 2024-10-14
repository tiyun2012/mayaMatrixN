#include "PointsRBF.h"
#include <maya/MFnNumericAttribute.h>
#include <maya/MArrayDataHandle.h>
#include <maya/MGlobal.h>
#include <maya/MFnPlugin.h>

// Define the static attributes
MTypeId PointsRBF::id(0x001226C6);  // Unique node ID
MObject PointsRBF::inputPointsAttr;

// Compute method: No specific output, compute is minimal
MStatus PointsRBF::compute(const MPlug& plug, MDataBlock& dataBlock)
{
    return MStatus::kSuccess;
}

// Creator function
void* PointsRBF::creator()
{
    return new PointsRBF();
}

// Initialize the node's attributes
MStatus PointsRBF::initialize()
{
    MFnNumericAttribute numAttr;

    // Input array of vector3 (renamed to inputPoints)
    inputPointsAttr = numAttr.createPoint("inputPoints", "inPoints");
    numAttr.setArray(true);  // This is an array of vector3 values
    numAttr.setUsesArrayDataBuilder(true);
    addAttribute(inputPointsAttr);

    return MStatus::kSuccess;
}

// Plugin registration
MStatus initializePlugin(MObject obj)
{
    MFnPlugin plugin(obj, "YourName", "1.0", "Any");
    return plugin.registerNode("PointsRBF", PointsRBF::id, PointsRBF::creator, PointsRBF::initialize);
}

MStatus uninitializePlugin(MObject obj)
{
    MFnPlugin plugin(obj);
    return plugin.deregisterNode(PointsRBF::id);
}
