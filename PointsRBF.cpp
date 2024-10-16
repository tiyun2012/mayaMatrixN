#include <maya/MPxNode.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MArrayDataHandle.h>
#include <maya/MGlobal.h>
#include <maya/MFnPlugin.h>
#include <maya/MVector.h>

// Node Class Definition
class LocatorPositionNode : public MPxNode {
public:
    LocatorPositionNode() {}
    virtual ~LocatorPositionNode() override {}

    // Creator function to create an instance of the node
    static void* creator() {
        MGlobal::displayInfo(MString("new node"));
        return new LocatorPositionNode();
    }

    // Initialize function to define attributes
    static MStatus initialize();

    // Compute function to process input and produce output
    virtual MStatus compute(const MPlug& plug, MDataBlock& dataBlock) override;

    // Static member variables for input and output attributes
    static MTypeId id;
    static MObject inputPositions; // Array of locator positions (double3)
    static MObject outputAttribute; // Output attribute for demonstration
};

// Unique ID for the node (replace with your unique ID)
MTypeId LocatorPositionNode::id(0x00012345);

// Define static member variables (input and output attributes)
MObject LocatorPositionNode::inputPositions;
MObject LocatorPositionNode::outputAttribute;

MStatus LocatorPositionNode::initialize() {
    MFnNumericAttribute nAttr;

    // Define an array of double3 (vector) attributes for input positions
    inputPositions = nAttr.createPoint("inputPositions", "inPos");
    nAttr.setArray(true); // This makes it an array
    nAttr.setStorable(false);
    nAttr.setWritable(true);
    addAttribute(inputPositions);

    // Define an output attribute for demonstration
    outputAttribute = nAttr.createPoint("output", "out");
    nAttr.setWritable(false);
    nAttr.setStorable(false);
    addAttribute(outputAttribute);

    // Define the relationship between input and output
    attributeAffects(inputPositions, outputAttribute);

    return MS::kSuccess;
}

MStatus LocatorPositionNode::compute(const MPlug& plug, MDataBlock& dataBlock) {
    if (plug == outputAttribute) {
        // Access input values
        MArrayDataHandle inputArrayHandle = dataBlock.inputArrayValue(inputPositions);
        unsigned int numElements = inputArrayHandle.elementCount();

        MGlobal::displayInfo("Positions gathered from locators:");
        for (unsigned int i = 0; i < numElements; ++i) {
            inputArrayHandle.jumpToElement(i);
            MDataHandle positionHandle = inputArrayHandle.inputValue();
            MVector position = positionHandle.asVector(); // Get the vector (locator position)

            // Print the position to the Maya output
            MString positionInfo = "Locator " + MString() + i + ": (" + position.x + ", " + position.y + ", " + position.z + ")";
            MGlobal::displayInfo(positionInfo);
        }

        // Mark the output as clean
        MDataHandle outputHandle = dataBlock.outputValue(outputAttribute);
        outputHandle.setClean();

        return MS::kSuccess;
    }

    return MS::kUnknownParameter;
}

// Plugin load and unload functions
MStatus initializePlugin(MObject obj) {
    MFnPlugin plugin(obj, "YourName", "1.0", "Any");

    // Register the node
    MStatus status = plugin.registerNode(
        "LocatorPositionNode",                // Name of the node
        LocatorPositionNode::id,              // Unique ID
        LocatorPositionNode::creator,         // Creator function
        LocatorPositionNode::initialize       // Initialize function
    );

    if (!status) {
        status.perror("Failed to register node");
        return status;
    }

    MGlobal::displayInfo("LocatorPositionNode plugin loaded.");
    return MS::kSuccess;
}

MStatus uninitializePlugin(MObject obj) {
    MFnPlugin plugin(obj);

    // Deregister the node
    MStatus status = plugin.deregisterNode(LocatorPositionNode::id);

    if (!status) {
        status.perror("Failed to deregister node");
        return status;
    }

    MGlobal::displayInfo("LocatorPositionNode plugin unloaded.");
    return MS::kSuccess;
}
