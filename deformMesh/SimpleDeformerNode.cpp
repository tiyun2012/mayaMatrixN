#include "SimpleDeformerNode.h"
#include <maya/MFnPlugin.h>
#include <maya/MDataHandle.h>
#include <maya/MItGeometry.h>
#include <maya/MMatrix.h>
#include <maya/MFloatVector.h>
#include <maya/MGlobal.h>
#include <maya/MFnTypedAttribute.h>  // For mesh attribute types
#include <maya/MFnNumericAttribute.h>  // For numeric attributes if needed

MTypeId SimpleDeformerNode::id(0x00122C02); // Unique Node ID
MObject SimpleDeformerNode::restMeshAttribute;

void* SimpleDeformerNode::creator() {
    return new SimpleDeformerNode();
}

MStatus SimpleDeformerNode::initialize() {
    MStatus stat;
    MFnTypedAttribute tAttr;

    // Create the restMesh attribute (assuming it's a mesh)
    restMeshAttribute = tAttr.create("restMesh", "rm", MFnData::kMesh);
    tAttr.setStorable(true);
    tAttr.setKeyable(true);

    // Add the attribute to the node
    addAttribute(restMeshAttribute);

    // Ensure the deformer updates when restMesh is connected
    attributeAffects(restMeshAttribute, outputGeom);


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

void SimpleDeformerNode::updateRestMesh() {
    MGlobal::displayInfo("Hello restmesh");  // Use MGlobal to print in Maya's script editor
}

MStatus SimpleDeformerNode::compute(const MPlug& plug, MDataBlock& dataBlock) {
    // Existing code...

    //// Check if the restMesh attribute is being updated
    //if (plug == restMeshAttribute) {
    //    updateRestMesh();  // Call your new function
    //}

    // Existing code...
    return MS::kSuccess;
}

