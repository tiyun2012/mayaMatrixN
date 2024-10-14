#ifndef POINTS_RBF_H
#define POINTS_RBF_H

#include <maya/MPxNode.h>
#include <maya/MTypeId.h>
#include <maya/MVectorArray.h>  // For handling vector arrays

class PointsRBF : public MPxNode
{
public:
    PointsRBF() {}
    virtual ~PointsRBF() {}

    virtual MStatus compute(const MPlug& plug, MDataBlock& dataBlock) override;

    // Updated postConstructor to return void
    virtual void postConstructor() override;

    static void* creator();
    static MStatus initialize();

    static MTypeId id;

    // Input attribute
    static MObject inputPointsAttr;
};

#endif
