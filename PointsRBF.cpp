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
    // As no computation is required, return success.
    return MStatus::kSuccess;
}

// postConstructor method to ensure the inputPoints attribute has at least 2 elements
void PointsRBF::postConstructor()
{
    MStatus status;

    MPlug inputPointsPlug(thisMObject(), inputPointsAttr);

    unsigned int numElements = inputPointsPlug.numElements();

    // Ensure there are at least 2 elements
    if (numElements < 2)
    {
        MGlobal::displayWarning("Input points array has less than 2 elements. Adding default points.");

        // Add the first point
        MPlug elementPlug0 = inputPointsPlug.elementByLogicalIndex(0, &status);
        if (status == MStatus::kSuccess) {
            elementPlug0.child(0).setDouble(1.0);  // Set X value
            elementPlug0.child(1).setDouble(0.0);  // Set Y value
            elementPlug0.child(2).setDouble(0.0);  // Set Z value
        }

        // Add the second point
        MPlug elementPlug1 = inputPointsPlug.elementByLogicalIndex(1, &status);
        if (status == MStatus::kSuccess) {
            elementPlug1.child(0).setDouble(0.0);  // Set X value
            elementPlug1.child(1).setDouble(1.0);  // Set Y value
            elementPlug1.child(2).setDouble(0.0);  // Set Z value
        }
    }
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
