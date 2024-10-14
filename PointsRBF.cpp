#include "PointsRBF.h"
#include <maya/MFnNumericAttribute.h>
#include <maya/MArrayDataHandle.h>
#include <maya/MGlobal.h>
#include <maya/MFnPlugin.h>
#include <maya/MFnNumericData.h>  // To handle numeric data
#include <maya/MArrayDataBuilder.h>  // Include this to use MArrayDataBuilder

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

    // Set the minimum number of elements to 2
    numAttr.setMin(2);

    // Add the attribute to the node
    addAttribute(inputPointsAttr);

    return MStatus::kSuccess;
}

// Post-constructor to pre-populate inputPoints array with 2 default values
void PointsRBF::postConstructor()
{
    MStatus status;

    // Access the datablock to set default values for the inputPoints array
    MObject thisNode = thisMObject();
    MPlug inputPointsPlug(thisNode, inputPointsAttr);

    if (inputPointsPlug.isArray())
    {
        MDataHandle handle;
        for (unsigned int i = 0; i < 2; ++i)
        {
            handle = inputPointsPlug.elementByLogicalIndex(i).asMDataHandle();
            MFnNumericData fnNumericData;
            MObject dataObject = fnNumericData.create(MFnNumericData::k3Double);
            fnNumericData.setData(0.0, 0.0, 0.0);  // Set default vector values
            handle.setMObject(dataObject);
        }
    }
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
