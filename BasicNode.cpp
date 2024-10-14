#include "BasicNode.h"
#include <maya/MFnNumericAttribute.h>
#include <maya/MGlobal.h>
#include <maya/MFnPlugin.h>

// Define the static attributes
MTypeId BasicNode::id(0x001226C0);
MObject BasicNode::inputAttr;
MObject BasicNode::outputAttr;

// Compute method: this is where the logic happens
MStatus BasicNode::compute(const MPlug& plug, MDataBlock& dataBlock)
{
    if (plug == outputAttr)
    {
        float inputValue = dataBlock.inputValue(inputAttr).asFloat();
        float outputValue = inputValue * 2.0f;

        MDataHandle outputHandle = dataBlock.outputValue(outputAttr);
        outputHandle.set(outputValue);
        dataBlock.setClean(plug);

        return MStatus::kSuccess;
    }

    return MStatus::kUnknownParameter;
}

// Creator function
void* BasicNode::creator()
{
    return new BasicNode();
}

// Initialize the node's attributes
MStatus BasicNode::initialize()
{
    MFnNumericAttribute nAttr;

    inputAttr = nAttr.create("input", "in", MFnNumericData::kFloat, 0.0);
    nAttr.setStorable(true);
    nAttr.setKeyable(true);
    addAttribute(inputAttr);

    outputAttr = nAttr.create("output", "out", MFnNumericData::kFloat, 0.0);
    nAttr.setWritable(false);
    nAttr.setStorable(false);
    addAttribute(outputAttr);

    attributeAffects(inputAttr, outputAttr);

    return MStatus::kSuccess;
}

// Plugin registration
MStatus initializePlugin(MObject obj)
{
    MFnPlugin plugin(obj, "YourName", "1.0", "Any");
    return plugin.registerNode("BasicNode", BasicNode::id, BasicNode::creator, BasicNode::initialize);
}

MStatus uninitializePlugin(MObject obj)
{
    MFnPlugin plugin(obj);
    return plugin.deregisterNode(BasicNode::id);
}
