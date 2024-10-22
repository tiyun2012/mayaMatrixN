#include "SimpleDeformerNode.h"
#include <maya/MFnPlugin.h>
#include <maya/MDataHandle.h>
#include <maya/MItGeometry.h>
#include <maya/MMatrix.h>
#include <maya/MFloatVector.h>
#include <maya/MGlobal.h>

MTypeId SimpleDeformerNode::id(0x00122C02); // Unique Node ID

void* SimpleDeformerNode::creator() {
    return new SimpleDeformerNode();
}

MStatus SimpleDeformerNode::initialize() {
    MStatus stat;

   

    return MS::kSuccess;
}

MStatus SimpleDeformerNode::deform(MDataBlock& block, MItGeometry& iter, const MMatrix& mat, unsigned int multiIndex) {
    MStatus stat;

    // Iterate through each vertex of the geometry
    for (; !iter.isDone(); iter.next()) {
        MPoint pt = iter.position();

        // Deform the y-coordinate by 1 unit
        pt.y += 1.0;

        iter.setPosition(pt);
    }

    return MS::kSuccess;
}

// Plugin registration
MStatus initializePlugin(MObject obj) {
    MStatus stat;
    MFnPlugin plugin(obj, "Your Name", "1.0", "Any");

    // Register the custom node
    stat = plugin.registerNode("simpleDeformerNode", SimpleDeformerNode::id, SimpleDeformerNode::creator, SimpleDeformerNode::initialize, MPxNode::kDeformerNode);
    CHECK_MSTATUS_AND_RETURN_IT(stat);

    return MS::kSuccess;
}

MStatus uninitializePlugin(MObject obj) {
    MStatus stat;
    MFnPlugin plugin(obj);

    // Deregister the custom node
    stat = plugin.deregisterNode(SimpleDeformerNode::id);
    CHECK_MSTATUS_AND_RETURN_IT(stat);

    return MS::kSuccess;
}
