
#ifndef RBF_POINTS_H
#define RBF_POINTS_H

#include <maya/MPxNode.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MArrayDataHandle.h>
#include <maya/MGlobal.h>
#include <maya/MVector.h>

// Node Class Definition
class RBFPoints : public MPxNode {
public:
    RBFPoints() {}
    virtual ~RBFPoints() override {}

    // Creator function to create an instance of the node
    static void* creator();

    // Initialize function to define attributes
    static MStatus initialize();

    // Set default values in postConstructor
    virtual void postConstructor() override;

    // Overriding setDependentsDirty to detect changes
    virtual MStatus setDependentsDirty(const MPlug& plug, MPlugArray& plugArray) override;

    // Compute function to process input and produce output
    virtual MStatus compute(const MPlug& plug, MDataBlock& dataBlock) override;

    // Static member variables for input and output attributes
    static MTypeId id;
    static MObject inputPositions;  // Array of locator positions (double3)
    static MObject outputAttribute; // Output attribute for demonstration
    static MObject restPosition; // New attribute to store the updated positions
    // Helper function for RBF
    void rbf();
    // New function to set default values for inputPositions and restPosition
    void defaultInput(MDataBlock& dataBlock);
};

#endif
