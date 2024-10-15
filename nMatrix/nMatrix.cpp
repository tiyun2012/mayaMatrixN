#include <maya/MPxNode.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnPlugin.h>
#include <maya/MFloatArray.h>
#include <maya/MGlobal.h>
#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MArrayDataHandle.h>
#include <maya/MArrayDataBuilder.h>   // Correct header

#include <Eigen/Dense> // Eigen library for matrix operations

class CustomMatrixNode : public MPxNode {
public:
    CustomMatrixNode() {};
    virtual ~CustomMatrixNode() {};

    // Override postConstructor to initialize the array size
    void postConstructor() override;

    static void* creator() { return new CustomMatrixNode(); }
    static MStatus initialize();
    virtual MStatus compute(const MPlug& plug, MDataBlock& dataBlock);

public:
    static MObject inputMatrixAttr;   // The custom 5x5 matrix input
    static MObject outputAttr;        // Output attribute for demonstration

    static MTypeId id;
};

// Define the Node ID
MTypeId CustomMatrixNode::id(0x00080033);

// Define the attributes
MObject CustomMatrixNode::inputMatrixAttr;
MObject CustomMatrixNode::outputAttr;

// Post-constructor to set up the array size to 25 (for 5x5 matrix)
void CustomMatrixNode::postConstructor() {
    MStatus stat;

    // Get data block and prepare the array builder using the proper method
    MDataBlock dataBlock = forceCache();
    MArrayDataHandle arrayHandle = dataBlock.outputArrayValue(inputMatrixAttr, &stat);
    CHECK_MSTATUS(stat);

    // Create an array data builder with 25 elements (for 5x5 matrix)
    MArrayDataBuilder builder = arrayHandle.builder(&stat);
    CHECK_MSTATUS(stat);

    // Initialize array elements to 0.0 (optional, or set other initial values as needed)
    for (unsigned int i = 0; i < 25; ++i) {
        MDataHandle handle = builder.addElement(i);
        handle.set(0.0f);  // Initialize all values to 0.0f
    }

    // Set the builder back to the data block to finalize the initialization
    arrayHandle.set(builder);
    arrayHandle.setAllClean();  // Mark the array as clean after initialization
}

// Initialize the attributes and node
MStatus CustomMatrixNode::initialize() {
    MStatus stat;

    // Input attribute: 5x5 matrix as 25 individual floats
    MFnNumericAttribute nAttr;
    inputMatrixAttr = nAttr.create("inputMatrix", "inMat", MFnNumericData::kFloat, 0.0);
    nAttr.setArray(true);                 // Array to hold multiple values (5x5 matrix)
    nAttr.setUsesArrayDataBuilder(true);  // Array can be dynamically constructed
    nAttr.setStorable(true);              // Storable in Maya
    nAttr.setWritable(true);              // Input can be written
    nAttr.setReadable(false);             // Input is not directly readable
    nAttr.setMin(25);                     // 25 elements for a 5x5 matrix

    // Output attribute (just for demonstration)
    outputAttr = nAttr.create("outputValue", "outVal", MFnNumericData::kFloat, 0.0);
    nAttr.setWritable(false);             // Output writable only by the node
    nAttr.setStorable(false);             // Not storable

    // Add attributes to the node
    addAttribute(inputMatrixAttr);
    addAttribute(outputAttr);

    // Attribute dependencies
    attributeAffects(inputMatrixAttr, outputAttr);

    return MS::kSuccess;
}

// Compute method to process input and output
MStatus CustomMatrixNode::compute(const MPlug& plug, MDataBlock& dataBlock) {
    MStatus stat;

    // Check if the plug is requesting the output attribute
    if (plug != outputAttr) {
        return MS::kUnknownParameter;
    }

    // Get the input matrix data (as an array of floats)
    MArrayDataHandle inputArrayHandle = dataBlock.inputArrayValue(inputMatrixAttr, &stat);
    CHECK_MSTATUS_AND_RETURN_IT(stat);

    // Prepare an Eigen 5x5 matrix
    Eigen::Matrix<float, 5, 5> customMatrix;

    // Loop through the input handle and fill the Eigen matrix
    for (int i = 0; i < 25; ++i) {
        inputArrayHandle.jumpToArrayElement(i); // Jump to the current element in the array
        float value = inputArrayHandle.inputValue().asFloat(); // Get the float value
        int row = i / 5;  // Row index
        int col = i % 5;  // Column index
        customMatrix(row, col) = value; // Fill the matrix with row/column indexing
    }

    // Example operation: Perform Eigen matrix inverse
    Eigen::Matrix<float, 5, 5> resultMatrix = customMatrix.inverse();

    // For demonstration, we'll output the sum of the inverted matrix
    float outputValue = resultMatrix.sum();

    // Set the output value in the data block
    MDataHandle outputHandle = dataBlock.outputValue(outputAttr);
    outputHandle.setFloat(outputValue);  // Set the output float value
    outputHandle.setClean();             // Mark it as clean (up-to-date)

    return MS::kSuccess;
}

// Plugin initialization
MStatus initializePlugin(MObject obj) {
    MStatus status;
    MFnPlugin plugin(obj, "YourName", "1.0", "Any");

    // Register the custom node
    status = plugin.registerNode("customMatrixNode", CustomMatrixNode::id, CustomMatrixNode::creator,
        CustomMatrixNode::initialize);
    if (!status) {
        status.perror("registerNode");
        return status;
    }

    return MS::kSuccess;
}

// Plugin uninitialization
MStatus uninitializePlugin(MObject obj) {
    MStatus status;
    MFnPlugin plugin(obj);

    // Deregister the custom node
    status = plugin.deregisterNode(CustomMatrixNode::id);
    if (!status) {
        status.perror("deregisterNode");
        return status;
    }

    return MS::kSuccess;
}
