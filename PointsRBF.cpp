
#include "PointsRBF.h"
#include <maya/MFnPlugin.h>
#include <maya/MArrayDataBuilder.h>

MTypeId RBFPoints::id(0x00012345); // Unique ID for the node
MObject RBFPoints::inputPositions;
MObject RBFPoints::outputAttribute;

// Creator function to create an instance of the node
void* RBFPoints::creator() {
    MGlobal::displayInfo(MString("new node"));
    return new RBFPoints();
}

// Initialize function to define attributes
MStatus RBFPoints::initialize() {
    MFnNumericAttribute nAttr;

    inputPositions = nAttr.createPoint("inputPositions", "inPos");
    nAttr.setArray(true);  // This makes it an array
    nAttr.setStorable(true);  // Ensure values persist in the scene
    nAttr.setWritable(true);
    addAttribute(inputPositions);

    outputAttribute = nAttr.createPoint("output", "out");
    nAttr.setWritable(false);
    nAttr.setStorable(false);
    addAttribute(outputAttribute);

    attributeAffects(inputPositions, outputAttribute);

    return MS::kSuccess;
}

// postConstructor implementation
void RBFPoints::postConstructor() {
    MStatus status;

    MDataBlock dataBlock = forceCache();

    MArrayDataHandle arrayHandle = dataBlock.outputArrayValue(inputPositions, &status);
    if (!status) {
        status.perror("Failed to get array handle for inputPositions");
        return;
    }

    MArrayDataBuilder builder(inputPositions, 2, &status);
    if (!status) {
        status.perror("Failed to create MArrayDataBuilder");
        return;
    }

    MDataHandle elementHandle = builder.addElement(0, &status);
    elementHandle.setMFloatVector(MVector(1.0, 0.0, 0.0));
    elementHandle = builder.addElement(1, &status);
    elementHandle.setMFloatVector(MVector(0.0, 1.0, 0.0));

    arrayHandle.set(builder);
    arrayHandle.setAllClean();
}

// Overriding setDependentsDirty to detect input changes
MStatus RBFPoints::setDependentsDirty(const MPlug& plug, MPlugArray& plugArray) {
    if (plug == inputPositions) {
        rbf();
    }
    return MS::kSuccess;
}

// Compute function to process input and produce output
MStatus RBFPoints::compute(const MPlug& plug, MDataBlock& dataBlock) {
    if (plug == outputAttribute) {
        MArrayDataHandle inputArrayHandle = dataBlock.inputArrayValue(inputPositions);
        unsigned int numElements = inputArrayHandle.elementCount();

        for (unsigned int i = 0; i < numElements; ++i) {
            inputArrayHandle.jumpToElement(i);
            MDataHandle positionHandle = inputArrayHandle.inputValue();
            MVector position = positionHandle.asVector();
            MGlobal::displayInfo("Locator position: (" + MString() + position.x + ", " + position.y + ", " + position.z + ")");
        }

        MDataHandle outputHandle = dataBlock.outputValue(outputAttribute);
        outputHandle.setClean();

        return MS::kSuccess;
    }
    return MS::kUnknownParameter;
}

// Helper function for RBF
void RBFPoints::rbf() {
    MGlobal::displayInfo("Calling rbf");
}

// Plugin load and unload functions
MStatus initializePlugin(MObject obj) {
    MFnPlugin plugin(obj, "YourName", "1.0", "Any");
    MStatus status = plugin.registerNode(
        "RBFPoints",
        RBFPoints::id,
        RBFPoints::creator,
        RBFPoints::initialize
    );
    if (!status) {
        status.perror("Failed to register node");
        return status;
    }
    MGlobal::displayInfo("RBFPoints node registered.");
    return MS::kSuccess;
}

MStatus uninitializePlugin(MObject obj) {
    MFnPlugin plugin(obj);
    MStatus status = plugin.deregisterNode(RBFPoints::id);
    if (!status) {
        status.perror("Failed to deregister node");
        return status;
    }
    MGlobal::displayInfo("RBFPoints node deregistered.");
    return MS::kSuccess;
}
