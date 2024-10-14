#include "matrix_n.h"
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnMatrixData.h>
#include <maya/MGlobal.h>
#include <maya/MFnPlugin.h>

// Define the static attributes
MTypeId MatrixN::id(0x001226C1);  // Updated ID for new node name
MObject MatrixN::inputAttr;
MObject MatrixN::outputAttr;

// Compute method: this is where the logic happens
MStatus MatrixN::compute(const MPlug& plug, MDataBlock& dataBlock)
{
    if (plug == outputAttr)
    {
        // Get the input matrix
        MDataHandle inputHandle = dataBlock.inputValue(inputAttr);
        MMatrix inputMatrix = inputHandle.asMatrix();

        // Example: Double the matrix values for the output (element-wise)
        MMatrix outputMatrix = inputMatrix * 2.0;

        // Set the output matrix
        MDataHandle outputHandle = dataBlock.outputValue(outputAttr);
        MFnMatrixData matrixDataFn;
        MObject newMatrixData = matrixDataFn.create(outputMatrix);
        outputHandle.set(newMatrixData);
        dataBlock.setClean(plug);

        return MStatus::kSuccess;
    }

    return MStatus::kUnknownParameter;
}

// Creator function
void* MatrixN::creator()
{
    return new MatrixN();
}

// Initialize the node's attributes
MStatus MatrixN::initialize()
{
    MFnMatrixAttribute mAttr;

    // Create a 2x2 input matrix attribute
    inputAttr = mAttr.create("inputMatrix", "inM");
    mAttr.setDefault(MMatrix::identity);  // Set default to identity matrix (which is 4x4 by default)
    mAttr.setStorable(true);
    mAttr.setKeyable(true);
    addAttribute(inputAttr);

    // Create an output matrix attribute
    outputAttr = mAttr.create("outputMatrix", "outM");
    mAttr.setWritable(false);
    mAttr.setStorable(false);
    addAttribute(outputAttr);

    // Define dependencies between input and output
    attributeAffects(inputAttr, outputAttr);

    return MStatus::kSuccess;
}

// Plugin registration
MStatus initializePlugin(MObject obj)
{
    MFnPlugin plugin(obj, "TS_Nhat", "1.0", "Any");
    return plugin.registerNode("matrix_n", MatrixN::id, MatrixN::creator, MatrixN::initialize);
}

MStatus uninitializePlugin(MObject obj)
{
    MFnPlugin plugin(obj);
    return plugin.deregisterNode(MatrixN::id);
}
