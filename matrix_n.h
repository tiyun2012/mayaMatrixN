#ifndef MATRIX_N_H
#define MATRIX_N_H

#include <maya/MPxNode.h>
#include <maya/MTypeId.h>
#include <maya/MMatrix.h>  // Include for matrix handling

class MatrixN : public MPxNode
{
public:
    MatrixN() {}
    virtual ~MatrixN() {}
    virtual MStatus compute(const MPlug& plug, MDataBlock& dataBlock) override;

    static void* creator();
    static MStatus initialize();

    static MTypeId id;
    static MObject inputAttr;
    static MObject outputAttr;
};

#endif
