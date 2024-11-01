
#include "PointsRBF.h"
#include <maya/MFnPlugin.h>
#include <maya/MArrayDataBuilder.h>
#include<maya/MFnDependencyNode.h>
#include<maya/MFnMeshData.h>
MTypeId RBFPoints::id(0x00012345); // Unique ID for the node
MObject RBFPoints::inputPositions;
MObject RBFPoints::outputAttribute;
MObject RBFPoints::restPosition;
MObject RBFPoints::inputMesh;
MObject RBFPoints::outputMesh;
MObject RBFPoints::deformedMesh;
// Creator function to create an instance of the node
void* RBFPoints::creator() {
    MGlobal::displayInfo(MString("new node"));
    return new RBFPoints();
}

// Initialize function to define attributes
MStatus RBFPoints::initialize() {
    MFnNumericAttribute nAttr;
    MFnTypedAttribute tAttr;
    MStatus status = initAttrs(nAttr, tAttr);
    if (status != MS::kSuccess) {
        return status;
    }    

    attributeAffects(inputPositions, outputAttribute);
    attributeAffects(inputPositions, outputMesh);
    attributeAffects(inputMesh, outputMesh);
    //
    
    return MS::kSuccess;
}

// postConstructor implementation
void RBFPoints::postConstructor() {

    MStatus status;
    MDataBlock dataBlock = forceCache();
    // Call defaultInput() to set default values for inputPositions and restPosition
    defaultInput(dataBlock);
    std::vector < ctsdev::vector3 > restPoints=MayaPointToVector(dataBlock,restPosition,status);
    
    RBFPoints::eigenDistanceMatrix.resize(restPoints.size(), restPoints.size());
    //printMayaPointToVector(restPoints);
    updateDistanceMatrix(restPoints, eigenDistanceMatrix);
    Eigen::VectorXd TargetVector(restPoints.size());
    for (size_t i = 0; i < restPoints.size(); ++i) {
        TargetVector[i] = restPoints[i].y;
    }
    try {
        MGlobal::displayInfo(MString(" calculate Weights"));
        ctsdev::LUsolveWithRegularization(eigenDistanceMatrix, TargetVector, weights, best_epsilon);
        for (size_t i = 0; i < weights.size(); ++i) {

            // Format the output as a string
            std::ostringstream oss;
            oss << "weight: " << i << ":"<<weights[i];

            // Print the formatted string to Maya's script editor
            MGlobal::displayInfo(oss.str().c_str());
        }
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
    }


}

// Overriding setDependentsDirty to detect input changes
MStatus RBFPoints::setDependentsDirty(const MPlug& plug, MPlugArray& plugArray) {
    if (plug == inputPositions||plug.isChild()) {
        MGlobal::displayInfo("Input positions have changed");

        /*MArrayDataHandle inputArrayHandle = dataBlock.inputArrayValue(inputPositions);
        inputArrayHandle.setAllClean();*/
        //dataBlock.setClean(plug);
    }
    else if(plug==restPosition){
        MGlobal::displayInfo(MString(" restposition  changed"));
  


    }
    else if(plug == inputMesh) 
    {
        MGlobal::displayInfo("inputMesh changed.");

        MStatus status;

        ////// Get the source plug for inputMesh
        MPlug inputMeshPlug = plug.source(&status);
        if (status != MS::kSuccess || !inputMeshPlug.isConnected()) {
            MGlobal::displayError("Failed to find the source plug for inputMesh.");
            return MS::kFailure;
        }

        ////// Get the MObject of the connected mesh
        inMesh = inputMeshPlug.asMObject();
        if (inMesh.isNull()) {
            MGlobal::displayError("Failed to retrieve MObject from the source plug.");
            return MS::kFailure;
        }

        ////// Call the function to store vertex x and z coordinates
        status = storeVertexPositions(inMesh,inPoints);
        if (status != MS::kSuccess) {
            MGlobal::displayError("Failed to store vertex x and z coordinates.");
            return MS::kSuccess;
        }
        
    }
    

    MPxNode::setDependentsDirty(plug, plugArray);
    return MS::kSuccess;
}


MFloatPointArray previousInPoints;

MStatus RBFPoints::compute(const MPlug& plug, MDataBlock& dataBlock)
{
    MStatus status;

    // Check if we are computing the output mesh
    if (plug == outputMesh||plug==inputPositions)
    {
        
        
        // calculate weight
        std::vector < ctsdev::vector3 > restPoints = MayaPointToVector(dataBlock, restPosition, status);

        RBFPoints::eigenDistanceMatrix.resize(restPoints.size(), restPoints.size());
        //printMayaPointToVector(restPoints);
        updateDistanceMatrix(restPoints, eigenDistanceMatrix);
        Eigen::VectorXd TargetVector(restPoints.size());
        for (size_t i = 0; i < restPoints.size(); ++i) 
        {
            TargetVector[i] = restPoints[i].y;
        }
        try 
        {
            MGlobal::displayInfo(MString(" calculate Weights"));
            ctsdev::LUsolveWithRegularization(eigenDistanceMatrix, TargetVector, weights, best_epsilon);
            for (size_t i = 0; i < weights.size(); ++i) {

                // Format the output as a string
                std::ostringstream oss;
                oss << "weight: " << i << ":" << weights[i];

                // Print the formatted string to Maya's script editor
                MGlobal::displayInfo(oss.str().c_str());
            }
        }
        catch (const std::exception& e) {
            std::cerr << e.what() << std::endl;
        }
        // calculate interpolate
        // Get the input mesh handle
        MDataHandle inputMeshHandle = dataBlock.inputValue(inputMesh, &status);
        if (status != MS::kSuccess)
        {
            MGlobal::displayError("Failed to get input mesh handle");
            return status;
        }

        // Get the input mesh object
        MObject inputMeshObj = inputMeshHandle.asMesh();
        if (inputMeshObj.isNull())
        {
            MGlobal::displayError("Input mesh is null.");
            return MS::kFailure;
        }

        // Create a function set for the input mesh
        MFnMesh fnMeshOrig(inputMeshObj, &status);
        if (status != MS::kSuccess)
        {
            MGlobal::displayError("Create MFn for InputMesh failed");
            return status;
        }
        
        // Get the points from the input mesh
      /*  MFloatPointArray inPoints;
        fnMeshOrig.getPoints(inPoints, MSpace::kWorld);*/// calculate before
 
        // Compare the current points with the previous points to avoid unnecessary recomputation
        //if (previousInPoints.length() == inPoints.length()) {
        //    bool pointsAreSame = true;
        //    for (unsigned int i = 0; i < inPoints.length(); ++i) {
        //        if (inPoints[i].distanceTo(previousInPoints[i]) > 0.0001) {  // Small tolerance for floating-point precision
        //            pointsAreSame = false;
        //            break;
        //        }
        //    }

        //    if (pointsAreSame) {
        //        MGlobal::displayInfo("Input mesh hasn't changed, skipping computation.");
        //        return MS::kSuccess;
        //    }
        //}

        //// Store the current input points for future comparison
        //previousInPoints = inPoints;

        MGlobal::displayInfo("Input mesh has changed, recomputing output mesh.");

        // Copy input points to output points
        MFloatPointArray outPoints = inPoints;
        for (unsigned i = 0; i < inPoints.length(); ++i)
        {

        }
        // Create the mesh data using MFnMeshData
        MFnMeshData dataCreator;
        MObject outMeshData = dataCreator.create(&status);
        if (status != MS::kSuccess)
        {
            MGlobal::displayError("Failed to create output mesh data.");
            return MS::kFailure;
        }

        // Create the output mesh with the same topology as the input mesh
        MObject outputMeshObj = fnMeshOrig.copy(inputMeshObj, outMeshData, &status);
        if (status != MS::kSuccess)
        {
            MGlobal::displayError("Failed to copy input mesh for output mesh.");
            return status;
        }

        // Now create a function set for the output mesh
        MFnMesh fnMeshOutMesh(outputMeshObj, &status);
        if (status != MS::kSuccess)
        {
            MGlobal::displayError("Failed to create MFn for output mesh.");
            return status;
        }

        // Set the points on the output mesh in object space
        status = fnMeshOutMesh.setPoints(outPoints, MSpace::kObject);
        if (status != MS::kSuccess)
        {
            MGlobal::displayError("Failed to set points on the output mesh.");
            return status;
        }

        // Set the output mesh handle
        MDataHandle outputMeshHandle = dataBlock.outputValue(outputMesh, &status);
        if (status != MS::kSuccess)
        {
            MGlobal::displayError("Failed to get output mesh handle.");
            return status;
        }

        // Set the data to the handle
        outputMeshHandle.set(outMeshData);

        // Mark the data block as clean
        dataBlock.setClean(plug);

        MGlobal::displayInfo("Mesh vertices updated successfully.");
    }

    return MS::kUnknownParameter;
}





// Helper function for RBF
void RBFPoints::rbf(MDataBlock& dataBlock, MStatus& status) {
    MGlobal::displayInfo("Calling rbf");
    inputPositionsToStd = MayaPointToVector(dataBlock, inputPositions, status);
    updateMeshYValues(indeformedMesh, inPoints, weights,inputPositionsToStd);

}
void RBFPoints::updateDistanceMatrix(const std::vector<ctsdev::vector3>& restPositions, Eigen::MatrixXd& eigenDistanceMatrix) {
    // Step 1: Use the BasicMatrix distance_matrix_2d function to calculate the distance matrix
    std::vector<std::vector<double>> distanceMatrix;
    ctsdev::distance_matrix_2d(restPositions, distanceMatrix);

    // Step 2: Convert the std::vector<std::vector<double>> to Eigen::MatrixXd
    size_t numPoints = restPositions.size();
    eigenDistanceMatrix.resize(numPoints, numPoints);  // Resize the Eigen matrix

    for (size_t i = 0; i < numPoints; ++i) {
        for (size_t j = 0; j < numPoints; ++j) {
            eigenDistanceMatrix(i, j) = distanceMatrix[i][j];
            std::ostringstream oss;
            oss << "eigen: " << i<<j << ":" << eigenDistanceMatrix(i,j);

            // Print the formatted string to Maya's script editor
            MGlobal::displayInfo(oss.str().c_str());
        }
    }

    MGlobal::displayInfo("Distance matrix updated and converted to Eigen::MatrixXd.");
}
// Function to set default values for inputPositions and restPosition
void RBFPoints::defaultInput(MDataBlock& dataBlock) {
    MStatus status;

    // Set default values for inputPositions
    MArrayDataHandle inputArrayHandle = dataBlock.outputArrayValue(inputPositions, &status);
    MArrayDataBuilder inputBuilder(inputPositions, 2, &status);
    if (!status) {
        status.perror("Failed to create input MArrayDataBuilder");
        return;
    }

    MDataHandle elementHandle = inputBuilder.addElement(0, &status);
    elementHandle.setMFloatVector(MVector(1.0, 0.0, 0.0));
    elementHandle = inputBuilder.addElement(1, &status);
    elementHandle.setMFloatVector(MVector(0.0, 1.0, 0.0));
    inputArrayHandle.set(inputBuilder);
    inputArrayHandle.setAllClean();

    // Set default values for restPosition
    MArrayDataHandle restArrayHandle = dataBlock.outputArrayValue(restPosition, &status);
    MArrayDataBuilder restBuilder(restPosition, 2, &status);
    if (!status) {
        status.perror("Failed to create rest MArrayDataBuilder");
        return;
    }

    elementHandle = restBuilder.addElement(0, &status);
    elementHandle.setMFloatVector(MVector(1.0, 0.0, 0.0));
    elementHandle = restBuilder.addElement(1, &status);
    elementHandle.setMFloatVector(MVector(0.0, 1.0, 0.0));
    restArrayHandle.set(restBuilder);
    restArrayHandle.setAllClean();

    MGlobal::displayInfo("Default values set for inputPositions and restPosition.");
}
// Static function to convert Maya points (MArrayDataHandle) to std::vector of vector3
std::vector<ctsdev::vector3> RBFPoints::MayaPointToVector(MDataBlock& dataBlock, const MObject& attribute, MStatus& status) {
    std::vector<ctsdev::vector3> positions;

    // Retrieve the MArrayDataHandle from the dataBlock using the provided attribute
    MArrayDataHandle posHandle = dataBlock.inputArrayValue(attribute, &status);
    if (status != MS::kSuccess) {
        MGlobal::displayError("Failed to get array data handle.");
        return positions;  // Return an empty vector on failure
    }

    // Retrieve the number of elements
    unsigned int numElements = posHandle.elementCount();
    if (numElements == 0) {
        MGlobal::displayWarning("No elements found in the array handle.");
        return positions;  // No elements to process
    }

    // Loop through the elements in the MArrayDataHandle and extract positions
    for (unsigned int i = 0; i < numElements; ++i) {
        // Move to the element by physical index
        posHandle.jumpToElement(i);
        MDataHandle dataHandle = posHandle.inputValue(&status);
        if (status != MS::kSuccess) {
            MGlobal::displayError("Failed to get data handle for element.");
            continue;  // Skip this element on failure
        }

        // Get the float3 data from the handle
        float3& position = dataHandle.asFloat3();

        // Log each position for debugging
      /*  std::ostringstream oss;
        oss << "MVector (Element " << i << "): (x = " << position[0] << ", y = " << position[1] << ", z = " << position[2] << ")";
        MGlobal::displayInfo(oss.str().c_str());*/

        // Convert float3 to your custom vector3 struct
        ctsdev::vector3 point;
        point.x = position[0];
        point.y = position[1];
        point.z = position[2];
        positions.push_back(point);
    }

    MGlobal::displayInfo(MString("Converted points for attribute: ") + attribute.apiTypeStr());
    return positions;
}


void RBFPoints::printMayaPointToVector(const std::vector<ctsdev::vector3>& points)
{
    for (size_t i = 0; i < points.size(); ++i) {
        const ctsdev::vector3& point = points[i];

        // Format the output as a string
        std::ostringstream oss;
        oss << "Point " << i << ": (x = " << point.x << ", y = " << point.y << ", z = " << point.z << ")";

        // Print the formatted string to Maya's script editor
        MGlobal::displayInfo(oss.str().c_str());
    }
}


// Plugin load and unload functions
MStatus initializePlugin(MObject obj) {
    MFnPlugin plugin(obj, "YourName", "1.0", "Any");
    MStatus status = plugin.registerNode(
        "RBFPoints",
        RBFPoints::id,
        RBFPoints::creator,
        RBFPoints::initialize
    );
    if (!status) {
        status.perror("Failed to register node");
        return status;
    }
    MGlobal::displayInfo("RBFPoints node registered.");
    return MS::kSuccess;
}

MStatus uninitializePlugin(MObject obj) {
    MFnPlugin plugin(obj);
    MStatus status = plugin.deregisterNode(RBFPoints::id);
    if (!status) {
        status.perror("Failed to deregister node");
        return status;
    }
    MGlobal::displayInfo("RBFPoints node deregistered.");
    return MS::kSuccess;
}
MStatus RBFPoints::initAttrs(MFnNumericAttribute& nAttr, MFnTypedAttribute& tAttr) {
    // Create inputPositions attribute
    inputPositions = nAttr.createPoint("inputPositions", "inPos");
    nAttr.setArray(true);  // This makes it an array
    nAttr.setStorable(true);  // Ensure values persist in the scene
    nAttr.setWritable(true);
    addAttribute(inputPositions);

    // Create outputAttribute attribute
    outputAttribute = nAttr.createPoint("output", "out");
    nAttr.setWritable(false);
    nAttr.setStorable(false);
    addAttribute(outputAttribute);

    // Create restPosition attribute
    restPosition = nAttr.createPoint("restPosition", "in");
    nAttr.setArray(true);  // This makes it an array
    nAttr.setStorable(true);  // Ensure values persist in the scene
    nAttr.setWritable(true);
    addAttribute(restPosition);

    // Create inputMesh attribute
    inputMesh = tAttr.create("inputMesh", "inMesh", MFnData::kMesh);
    tAttr.setStorable(true);   // Allow the mesh to be stored
    tAttr.setWritable(true);   // Writable to allow connections
    tAttr.setReadable(false);  // We don't need to output this attribute
    addAttribute(inputMesh);
    // Create deformedtMesh attribute
    deformedMesh = tAttr.create("deformedMesh", "defMesh", MFnData::kMesh);
    tAttr.setStorable(false);   // Allow the mesh to be stored
    tAttr.setWritable(true);   // Writable to allow connections
    tAttr.setReadable(true);  // We don't need to output this attribute
    addAttribute(deformedMesh);
    // Create outMesh attribute
    outputMesh = tAttr.create("outputMesh", "outMesh", MFnData::kMesh);
    tAttr.setStorable(false);   // Allow the mesh to be stored
    tAttr.setWritable(false);   // Writable to allow connections
    tAttr.setReadable(true);  // We don't need to output this attribute
    addAttribute(outputMesh);
    return MS::kSuccess;
}
// Function to retrieve and store vector values into a std::vector<ctsdev::vector3>
MStatus RBFPoints::getVectorValuesFromPlug(const MPlug& plug, std::vector<ctsdev::vector3>& outValues) {
    MStatus status;

    // Clear the output vector to make sure we start fresh
    outValues.clear();

    if (plug.isArray()) {
        // Handle array plug, such as inputPositions or restPosition
        unsigned int numElements = plug.numElements();
        for (unsigned int i = 0; i < numElements; ++i) {
            MPlug elementPlug = plug.elementByPhysicalIndex(i, &status);
            if (status == MS::kSuccess) {
                // Get the individual components (x, y, z) of the MVector
                MPlug xPlug = elementPlug.child(0);  // X component
                MPlug yPlug = elementPlug.child(1);  // Y component
                MPlug zPlug = elementPlug.child(2);  // Z component

                double x = xPlug.asDouble();
                double y = yPlug.asDouble();
                double z = zPlug.asDouble();

                // Store the vector into outValues
                ctsdev::vector3 point = { x, y, z };
                outValues.push_back(point);

                //// Print the retrieved vector for debugging
                //std::ostringstream oss;
                //oss << "MVector (Element " << i << "): (x = " << x << ", y = " << y << ", z = " << z << ")";
                //MGlobal::displayInfo(oss.str().c_str());
            }
            else {
                MGlobal::displayError("Failed to get element plug value.");
            }
        }
    }
    else {
        // If it's a single value (not an array), just get the components
        MPlug xPlug = plug.child(0);  // X component
        MPlug yPlug = plug.child(1);  // Y component
        MPlug zPlug = plug.child(2);  // Z component

        double x = xPlug.asDouble();
        double y = yPlug.asDouble();
        double z = zPlug.asDouble();

        // Store the vector into outValues
        ctsdev::vector3 point = { x, y, z };
        outValues.push_back(point);

        // Print the retrieved vector for debugging
        std::ostringstream oss;
        oss << "MVector: (x = " << x << ", y = " << y << ", z = " << z << ")";
        MGlobal::displayInfo(oss.str().c_str());
    }

    return MS::kSuccess;
}
// Function to update the y-value of each vertex using `interpolateRBF` for a given mesh (inMesh)
MStatus RBFPoints::updateMeshYValues(const MObject& inMesh, MFloatPointArray& inPoints, const std::vector<double>& weights, const std::vector<ctsdev::vector3>& controlPoints) {
    MStatus status;

    // Check if the inMesh is valid
    if (inMesh.isNull()) {
        MGlobal::displayError("Invalid defopmesh provided.");
        return MS::kFailure;
    }

    // Use MFnMesh to manipulate the mesh data
    MFnMesh fnMesh(inMesh, &status);
    if (status != MS::kSuccess) {
        MGlobal::displayError("Failed to create MFnMesh for deformed mesh.");
        return status;
    }

    // Get the vertex positions
    MFloatPointArray vertices= inPoints;
    //status = fnMesh.getPoints(vertices, MSpace::kWorld);  // Get the points in world space
    //if (status != MS::kSuccess) {
    //    MGlobal::displayError("Failed to get vertex positions.");
    //    return status;
    //}

    // Iterate over each vertex and update the y-value using interpolateRBF
    std::ostringstream oss;
    oss << "length of inpooints: " << vertices.length() << endl;
    for (unsigned int i = 0; i < vertices.length(); ++i) {
        double outputPoint[3] = { vertices[i].x, vertices[i].y, vertices[i].z };  // Use vertex's x and z

        // Call the interpolateRBF function to calculate the new y value
        ctsdev::interpolateRBF(const_cast<std::vector<double>&>(weights), outputPoint, controlPoints);

        // Update the y value of the vertex
        vertices[i].y = float(1.0);//static_cast<float>(outputPoint[1]);
        status = fnMesh.setPoint(i,vertices[i], MSpace::kWorld);  // Set the points back in world space
        if (status != MS::kSuccess) {
            MGlobal::displayError("Failed to set updated vertex positions.");
            return status;
        }

        std::ostringstream oss;
        oss << "verterx[" << i << "]= (" << vertices[i].x << "," << vertices[i].y << "," << vertices[i].z << ")" <<"Y:"<< outputPoint[1] << endl;
        MGlobal::displayInfo(oss.str().c_str());
    }

    // Set the updated points back to the mesh
    //status = fnMesh.setPoints(vertices, MSpace::kWorld);  // Set the points back in world space
    //if (status != MS::kSuccess) {
    //    MGlobal::displayError("Failed to set updated vertex positions.");
    //    return status;
    //}

    MGlobal::displayInfo("Mesh vertices updated successfully.");
    return MS::kSuccess;
}

// Function to check if the input has changed and store vertex x and z positions
MStatus RBFPoints::storeVertexXZ(const MObject& inMesh) {
    MStatus status;

    // Clear the previous values from vertexXZ
    vertexXZ.clear();

    // Check if the inMesh is valid
    if (inMesh.isNull()) {
        MGlobal::displayError("Invalid restmesh provided.");
        return MS::kFailure;
    }

    // Use MFnMesh to manipulate the mesh data
    MFnMesh fnMesh(inMesh, &status);
    if (status != MS::kSuccess) {
        MGlobal::displayError("Failed to create MFnMesh for restmesh.");
        return status;
    }

    // Get the vertex positions
    MFloatPointArray vertices;
    status = fnMesh.getPoints(vertices, MSpace::kWorld);  // Get the points in world space
    if (status != MS::kSuccess) {
        MGlobal::displayError("Failed to get vertex positions from restmesh.");
        return status;
    }

    // Iterate over each vertex and store the x and z coordinates in the vertexXZ vector
    for (unsigned int i = 0; i < vertices.length(); ++i) {
        double x = vertices[i].x;
        double z = vertices[i].z;
        vertexXZ.push_back(std::make_pair(x, z));

        // Print for debugging
        std::ostringstream oss;
        oss << "Stored vertex (x = " << x << ", z = " << z << ")";
        MGlobal::displayInfo(oss.str().c_str());
    }

    MGlobal::displayInfo("Stored all vertex x and z coordinates successfully.");
    return MS::kSuccess;
}


MStatus RBFPoints::storeVertexPositions(const MObject& inMesh, MFloatPointArray& inPoints) {
    MStatus status;

    // Clear the previous values from vertexXZ
    inPoints.clear();

    // Check if the inMesh is valid
    if (inMesh.isNull()) {
        MGlobal::displayError("Invalid mesh provided.");
        return MS::kFailure;
    }

    // Use MFnMesh to manipulate the mesh data
    MFnMesh fnMesh(inMesh, &status);
    if (status != MS::kSuccess) {
        MGlobal::displayError("Failed to create MFnMesh.");
        return status;
    }

    // Get the vertex positions
    //MFloatPointArray vertices;
    status = fnMesh.getPoints(inPoints, MSpace::kWorld);  // Get the points in world space
    if (status != MS::kSuccess) {
        MGlobal::displayError("Failed to get vertex positions.");
        return status;
    }



    for (unsigned int i = 0; i < inPoints.length(); ++i)
    {
        std::ostringstream oss;
        oss << "vertex[" << i << "]" << inPoints[i] << endl;
        MGlobal::displayInfo(oss.str().c_str());
    }
    MGlobal::displayInfo("Stored all vertex x and z coordinates successfully.");
    return MS::kSuccess;
}