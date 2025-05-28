#include <maya/MPxDeformerNode.h>
#include <maya/MFnMesh.h>
#include <maya/MItGeometry.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MMatrix.h>
#include <maya/MGlobal.h>
#include <maya/MFnPlugin.h>

class MLDeformer : public MPxDeformerNode {
public:
    MLDeformer() {}
    virtual ~MLDeformer() {}

    static void* creator();
    static MStatus initialize();

    MStatus deform(MDataBlock& block, MItGeometry& iter, const MMatrix& mat, unsigned int multiIndex) override {
        MMatrix transformMatrix = block.inputValue(aTransformMatrix).asMatrix();
        double offset = transformMatrix[3][0]; // Use X translation as a test offset
        


        for (; !iter.isDone(); iter.next()) {
            MPoint pt = iter.position();
            //// make Vertise follow TransformMatrix
            MPoint transformedPt = pt * transformMatrix; // Apply transformation
            // pt.x += offset; // Simple offset for testing
            iter.setPosition(transformedPt);
        }
        return MS::kSuccess;
    }

    static MTypeId id;
    static MObject aTransformMatrix;
};

MTypeId MLDeformer::id(0x80000);
MObject MLDeformer::aTransformMatrix;

MStatus MLDeformer::initialize() {
    MFnMatrixAttribute mAttr;
    aTransformMatrix = mAttr.create("transformMatrix", "tm");
    mAttr.setReadable(true);
    mAttr.setWritable(true);
    addAttribute(aTransformMatrix);
    attributeAffects(aTransformMatrix, outputGeom);
    return MS::kSuccess;
}

void* MLDeformer::creator() {
    return new MLDeformer;
}

MStatus initializePlugin(MObject obj) {
    MFnPlugin plugin(obj, "MLDeformer", "1.0", "Any");
    return plugin.registerNode("mlDeformer", MLDeformer::id, MLDeformer::creator, MLDeformer::initialize, MPxNode::kDeformerNode);
}

MStatus uninitializePlugin(MObject obj) {
    MFnPlugin plugin(obj);
    return plugin.deregisterNode(MLDeformer::id);
}