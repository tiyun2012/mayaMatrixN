#include <maya/MPxNode.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MArrayDataHandle.h>
#include <maya/MGlobal.h>
#include <maya/MFnPlugin.h>
#include <maya/MVector.h>
#include <maya/MArrayDataBuilder.h>
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
    // Set default values in postConstructor
    virtual void postConstructor() override;

    // Overriding setDependentsDirty to detect changes
    virtual MStatus setDependentsDirty(const MPlug& plug, MPlugArray& plugArray) override;

    // Compute function to process input and produce output
    virtual MStatus compute(const MPlug& plug, MDataBlock& dataBlock) override;

    // Static member variables for input and output attributes
    static MTypeId id;
    static MObject inputPositions; // Array of locator positions (double3)
    static MObject outputAttribute; // Output attribute for demonstration
    void rbf();
};

// Unique ID for the node (replace with your unique ID)
MTypeId LocatorPositionNode::id(0x00012346);

// Define static member variables (input and output attributes)
MObject LocatorPositionNode::inputPositions;
MObject LocatorPositionNode::outputAttribute;

MStatus LocatorPositionNode::initialize() {
    MFnNumericAttribute nAttr;

    // Define an array of double3 (vector) attributes for input positions
    inputPositions = nAttr.createPoint("inputPositions", "inPos");
    nAttr.setArray(true); // This makes it an array
    nAttr.setStorable(true); // Ensure values persist in the scene
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

void LocatorPositionNode::postConstructor() {
    MStatus status;

    // Force the cache to get a writable data block for this node
    MDataBlock dataBlock = forceCache();

    // Create an MArrayDataHandle for the inputPositions attribute
    MArrayDataHandle arrayHandle = dataBlock.outputArrayValue(inputPositions, &status);
    if (!status) {
        status.perror("Failed to get array handle for inputPositions");
        return; // Early exit if there's an error
    }

    // Create an MArrayDataBuilder to properly initialize the array with two elements
    MArrayDataBuilder builder(inputPositions, 2, &status);
    if (!status) {
        status.perror("Failed to create MArrayDataBuilder");
        return;
    }

    // Set first element to (1.0, 0.0, 0.0)
    MDataHandle elementHandle = builder.addElement(0, &status);
    if (!status) {
        status.perror("Failed to add first element");
        return;
    }
    elementHandle.setMFloatVector(MVector(1.0, 0.0, 0.0));
    MGlobal::displayInfo("First element set to (1.0, 0.0, 0.0)");

    // Set second element to (0.0, 1.0, 0.0)
    elementHandle = builder.addElement(1, &status);
    if (!status) {
        status.perror("Failed to add second element");
        return;
    }
    elementHandle.setMFloatVector(MVector(1.0, 1.0, 0.0));
    MGlobal::displayInfo("Second element set to (0.0, 1.0, 0.0)");

    // Set the arrayHandle with the values from the builder
    arrayHandle.set(builder);

    // Read back the values before marking as clean
    MGlobal::displayInfo("Reading back values before marking as clean:");
    for (unsigned int i = 0; i < arrayHandle.elementCount(); ++i) {
        arrayHandle.jumpToElement(i);
        MDataHandle handle = arrayHandle.inputValue();
        MVector vec = handle.asVector();
        MString msg = "Element " + MString() + i + ": (" + vec.x + ", " + vec.y + ", " + vec.z + ")";
        MGlobal::displayInfo(msg); // Print the actual values after they are set
    }

    // Mark the data as clean
    arrayHandle.setAllClean();
    MGlobal::displayInfo("Data block marked clean");

    // Read back the values again after marking as clean
    MGlobal::displayInfo("Reading back values after marking as clean:");
    for (unsigned int i = 0; i < arrayHandle.elementCount(); ++i) {
        arrayHandle.jumpToElement(i);
        MDataHandle handle = arrayHandle.inputValue();
        MVector vec = handle.asVector();
        MString msg = "Element " + MString() + i + ": (" + vec.x + ", " + vec.y + ", " + vec.z + ")";
        MGlobal::displayInfo(msg); // Print the actual values after marking clean
    }
}




MStatus LocatorPositionNode::compute(const MPlug& plug, MDataBlock& dataBlock) {
    MGlobal::displayInfo("compute() called");
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

// Override setDependentsDirty to detect input changes
MStatus LocatorPositionNode::setDependentsDirty(const MPlug& plug, MPlugArray& plugArray) {
    MGlobal::displayInfo("setDependentsDirty() called for plug: " + plug.name());
    // Check if the inputPositions attribute is changing
    if (plug == inputPositions) {
        // Call rbf() when the input changes
        rbf();
    }

    // Call the base class implementation
    return MPxNode::setDependentsDirty(plug, plugArray);
}

void LocatorPositionNode::rbf() {
    MGlobal::displayInfo(MString("Calling rbf"));
}
