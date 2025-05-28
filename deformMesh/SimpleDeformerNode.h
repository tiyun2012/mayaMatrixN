#ifndef _SIMPLEDEFORMERNODE_H_
#define _SIMPLEDEFORMERNODE_H_

#include <maya/MPxDeformerNode.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MItGeometry.h>
#include <maya/MMatrix.h>
#include <maya/MTypeId.h>

class SimpleDeformerNode : public MPxDeformerNode {
public:
    SimpleDeformerNode() {}
    virtual ~SimpleDeformerNode() {}

    static void* creator();
    static MStatus initialize();
    static MObject restMeshAttribute;
    // Override the deform function
    virtual MStatus deform(MDataBlock& block, MItGeometry& iter, const MMatrix& mat, unsigned int multiIndex);
    void updateRestMesh();  // Declare the function
    MStatus compute(const MPlug& plug, MDataBlock& dataBlock);
    static MTypeId id;
};

#endif