#ifndef BASICNODE_H
#define BASICNODE_H

#include <maya/MPxNode.h>
#include <maya/MTypeId.h>

class BasicNode : public MPxNode
{
public:
    BasicNode() {}
    virtual ~BasicNode() {}
    virtual MStatus compute(const MPlug& plug, MDataBlock& dataBlock) override;

    static void* creator();
    static MStatus initialize();

    static MTypeId id;
    static MObject inputAttr;
    static MObject outputAttr;
};

#endif
