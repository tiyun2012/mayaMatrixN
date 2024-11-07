#include <Eigen/Dense>
#include <maya/MFnMesh.h>
#include <maya/MGlobal.h>
#include <maya/MPointArray.h>
#include <maya/MItGeometry.h>
#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MPlug.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MPxDeformerNode.h>
#include <maya/MFnPlugin.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnData.h> 
#include<maya/MFnMatrixArrayData.h>
#include<maya/MFnMatrixAttribute.h>
#include<maya/MFnArrayAttrsData.h>
#include <maya/MArrayDataBuilder.h>
#include<maya/MFnIntArrayData.h>
#include<maya/MFnDoubleArrayData.h>

class RBFDeformerNode : public MPxDeformerNode {
public:
    static MTypeId id;
    static MObject aEpsilon;
    static MObject aControlMesh;
    static MObject aControlMeshTransform;
    static MObject aMaxInfluence;
    static MObject aClosestIndices;
    static MObject aWeightBasedClosestIndices;
    static MObject aCenterOfInfluenceVertex;


    bool isInitialized = false;

    static void* creator() { return new RBFDeformerNode(); }
    static MStatus initialize();

    void computeInitialWeights(const Eigen::MatrixXd& vertices,
        const Eigen::MatrixXd& controlPoints,
        Eigen::MatrixXd& weightsMatrixOrig);
    void updateWeightsAndOffsets(const Eigen::MatrixXd& weightsMatrixOrig, double epsilon,
        const Eigen::MatrixXd& controlPoints,
        Eigen::MatrixXd& weightsMatrixUpdated, Eigen::MatrixXd& offsets,
        const Eigen::MatrixXd& vertices);
    Eigen::MatrixXd applyRBFDeformation(const Eigen::MatrixXd& weightsMatrixUpdated,
        const Eigen::MatrixXd& offsets,
        const Eigen::MatrixXd& deformedControlPoints);
    virtual MStatus setDependentsDirty(const MPlug& plug, MPlugArray& plugArray) override;
    MStatus deform(MDataBlock& dataBlock, MItGeometry& iter,
        const MMatrix& localToWorldMatrix, unsigned int geomIndex) override;
    std::vector<int> getNClosestControlPoints(int vertexIndex, int n, const Eigen::MatrixXd& vertices, const Eigen::MatrixXd& controlPoints);
private:
    //bool controlMeshChanged = false;
    //bool controlMeshSourceChanged = false;
    bool hasControlMesh;
    bool enableRecalcualte = true;
    MPointArray mayaRestVertices;//maya
    Eigen::MatrixXd eigenRestVertices;
    Eigen::MatrixXd offsets;
    Eigen::MatrixXd weightsMatrix;
    Eigen::MatrixXd EigenControlPoints;
    Eigen::MatrixXd RestEigenControlPoints;
    MPointArray mayaControlPoints;
    MPointArray mayaRestControlPoints;
    Eigen::MatrixXd weightsMatrixUpdated;
    Eigen::MatrixXd deformedControlPoints;
    Eigen::MatrixXd weightsMatrixOrig;
    bool epsilonUpdated = false;
    bool maxInfluentUpdated = true;



};

MTypeId RBFDeformerNode::id(0x00002);
MObject RBFDeformerNode::aEpsilon;
MObject RBFDeformerNode::aControlMesh;
MObject RBFDeformerNode::aControlMeshTransform;
MObject RBFDeformerNode::aMaxInfluence;
MObject RBFDeformerNode::aClosestIndices;
MObject RBFDeformerNode::aWeightBasedClosestIndices;
MObject RBFDeformerNode::aCenterOfInfluenceVertex;
MStatus RBFDeformerNode::initialize()
{
    MFnTypedAttribute tAttr;
    MFnNumericAttribute nAttr;
    MFnMatrixAttribute mAttr;
    MStatus status;  // Declare status here

    // Control Mesh attribute
    aControlMesh = tAttr.create("controlMesh", "controlMesh", MFnData::kMesh, MObject::kNullObj, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    tAttr.setStorable(false);
    tAttr.setReadable(false);
    tAttr.setWritable(true);
    tAttr.setKeyable(false);
    tAttr.setCached(false);
    addAttribute(aControlMesh);

    // Epsilon attribute
    aEpsilon = nAttr.create("epsilon", "epsilon", MFnNumericData::kDouble, 8.0, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    nAttr.setMin(0.1);
    nAttr.setStorable(true);
    nAttr.setKeyable(true);
    nAttr.setCached(false);
    addAttribute(aEpsilon);
    // add matrix attribute
    aControlMeshTransform = mAttr.create("inMeshMatrix", "iMMatrix", MFnMatrixAttribute::kDouble, &status);
    mAttr.setConnectable(true);
    mAttr.setStorable(true);
    addAttribute(aControlMeshTransform);
    // Attribute affects

    attributeAffects(aControlMesh, outputGeom);
    attributeAffects(aEpsilon, outputGeom);
    attributeAffects(aControlMeshTransform, outputGeom);

    // Max Influence attribute
    aMaxInfluence = nAttr.create("maxInfluence", "maxInf", MFnNumericData::kInt, 4, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    nAttr.setMin(1); // Set minimum value
    nAttr.setStorable(true);
    nAttr.setKeyable(true);
    nAttr.setDefault(4); // Default value
    nAttr.setReadable(true);
    nAttr.setWritable(true);
    addAttribute(aMaxInfluence);

    // Attribute affects
    attributeAffects(aMaxInfluence, outputGeom);

    // Create the integer array attribute
    aClosestIndices = tAttr.create("closestIndices", "cidx", MFnData::kIntArray, MObject::kNullObj, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // Set it as a multi (array) attribute
    tAttr.setArray(true);
    tAttr.setUsesArrayDataBuilder(true);

    // Optional: Set other attribute properties if needed
    tAttr.setReadable(true);
    tAttr.setWritable(true);
    tAttr.setStorable(true);
    tAttr.setKeyable(true);
    tAttr.setCached(true);
    // Add the attribute to the node
    status = addAttribute(aClosestIndices);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    // Attribute affects
    attributeAffects(aClosestIndices, outputGeom);

    // Create the Double array attribute
    aWeightBasedClosestIndices = tAttr.create("WeightBasedClosestIndices", "wtBCIndices", MFnData::kDoubleArray, MObject::kNullObj, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    // Set it as a multi (array) attribute
    tAttr.setArray(true);
    tAttr.setUsesArrayDataBuilder(true);

    // Optional: Set other attribute properties if needed
    tAttr.setReadable(true);
    tAttr.setWritable(true);
    tAttr.setStorable(true);
    tAttr.setKeyable(true);
    tAttr.setCached(true);
    // Add the attribute to the node
    status = addAttribute(aWeightBasedClosestIndices);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    // Attribute affects
    attributeAffects(aWeightBasedClosestIndices, outputGeom);

    // Create inputPositions attribute
    aCenterOfInfluenceVertex = nAttr.createPoint("CenterOfInfluenceVertex", "CInf");
    //nAttr.setUsesArrayDataBuilder(true);
    nAttr.setArray(true);  // This makes it an array
    nAttr.setStorable(true);  // Ensure values persist in the scene
    nAttr.setWritable(true);
    nAttr.setCached(true);
    nAttr.setReadable(true);
    addAttribute(aCenterOfInfluenceVertex);
    attributeAffects(aCenterOfInfluenceVertex, outputGeom);
    return MS::kSuccess;
}


void RBFDeformerNode::computeInitialWeights(const Eigen::MatrixXd& vertices,
    const Eigen::MatrixXd& controlPoints,
    Eigen::MatrixXd& weightsMatrixOrig) {
    Eigen::Index numVertices = vertices.rows();
    Eigen::Index numControlPoints = controlPoints.rows();

    weightsMatrixOrig.resize(numVertices, numControlPoints);
    //offsets.resize(numVertices, 3);

    for (Eigen::Index i = 0; i < numVertices; ++i) {
        Eigen::VectorXd distances(numControlPoints);
        for (Eigen::Index j = 0; j < numControlPoints; ++j) {
            distances(j) = (vertices.row(i) - controlPoints.row(j)).norm();
        }
        double sumDistances = distances.sum();
        Eigen::VectorXd weights = distances / sumDistances;
        weightsMatrixOrig.row(i) = weights;


    }
}

void RBFDeformerNode::updateWeightsAndOffsets(const Eigen::MatrixXd& weightsMatrixOrig, double epsilon,
    const Eigen::MatrixXd& controlPoints,
    Eigen::MatrixXd& weightsMatrixUpdated,
    Eigen::MatrixXd& offsets,
    const Eigen::MatrixXd& vertices) {
    Eigen::Index numVertices = weightsMatrixOrig.rows();
    Eigen::Index numControlPoints = weightsMatrixOrig.cols();

    weightsMatrixUpdated.resize(numVertices, numControlPoints);
    for (Eigen::Index i = 0; i < numVertices; ++i) {
        Eigen::VectorXd weights = weightsMatrixOrig.row(i);
        weights = (1.0 - weights.array().pow(0.01)).pow(epsilon);
        // Normalize weights
        double sumWeights = weights.sum();
        weights /= sumWeights;

        weightsMatrixUpdated.row(i) = weights;

        // Recompute offset with new weights
        Eigen::Vector3d originalVertex = weights.transpose() * controlPoints;
        offsets.resize(numVertices, 3);
        offsets.row(i) = originalVertex.transpose() - vertices.row(i);
    }
}

Eigen::MatrixXd RBFDeformerNode::applyRBFDeformation(const Eigen::MatrixXd& weightsMatrixUpdated,
    const Eigen::MatrixXd& offsets,
    const Eigen::MatrixXd& deformedControlPoints) {
    return (weightsMatrixUpdated * deformedControlPoints) - offsets;
}

MStatus RBFDeformerNode::setDependentsDirty(const MPlug& plug, MPlugArray& plugArray)
{
    if (plug == aControlMesh)
    {

        if (plug.isConnected())
        {
            hasControlMesh = true;
        }
        else
        {
            hasControlMesh = false;
            enableRecalcualte = true;
        }
    }
    if (plug == aEpsilon && hasControlMesh)
    {
        epsilonUpdated = true;
    }
    if (plug == aMaxInfluence && hasControlMesh)
    {
        maxInfluentUpdated = true;
    }
    return MStatus();
}

MStatus RBFDeformerNode::deform(MDataBlock& dataBlock, MItGeometry& iter,
    const MMatrix& localToWorldMatrix, unsigned int geomIndex) {
    MStatus status;
    float envelope = dataBlock.inputValue(MPxDeformerNode::envelope, &status).asFloat();
    if (envelope == 0.0f) return MS::kSuccess;

    double epsilon = dataBlock.inputValue(aEpsilon, &status).asDouble();

    //MGlobal::displayInfo(MString("controlMeshChanged:") + controlMeshChanged);
    //bool hasControlMesh = !aControlMesh.isNull();
    if (hasControlMesh == false) {
        //MGlobal::displayWarning("control mesh is not connected");
        //MGlobal::displayWarning(MString("should recalculate weights :")+(enableRecalcualte) );
        return status;
    }
    unsigned int vertexCount = iter.count();
    int maxInfluence = dataBlock.inputValue(aMaxInfluence, &status).asInt();
    double bMaxInfluen = double(maxInfluence);
    //enableRecalcualte
     // -----------controlPoints ---------------------------
    MObject controlMeshObj = dataBlock.inputValue(aControlMesh, &status).asMesh();
    MFnMesh controlMeshFun(controlMeshObj, &status);
    // Define an array to store vertex positions
    //MPointArray mayaControlPoints;
    // Retrieve all vertex positions in world space
    status = controlMeshFun.getPoints(mayaControlPoints, MSpace::kWorld);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    int numberControlPoints = mayaControlPoints.length();
    EigenControlPoints.resize(numberControlPoints, 3);
    //Eigen::MatrixXd EigenControlPoints(numberControlPoints, 3);
    for (unsigned int i = 0; i < numberControlPoints; ++i)
    {
        EigenControlPoints.row(i) << mayaControlPoints[i].x, mayaControlPoints[i].y, mayaControlPoints[i].z;
        //MGlobal::displayInfo(MString("RestPoint: ") + mayaControlPoints[i].x + " " + mayaControlPoints[i].y + " " + mayaControlPoints[i].z);
    }
    if (enableRecalcualte)
    {
        //MGlobal::displayWarning("update logic when controlMeshSourceChanged.");
        // -----------rest mesh---------------------------
        RestEigenControlPoints = EigenControlPoints;
        mayaRestControlPoints = mayaControlPoints;
        iter.allPositions(mayaRestVertices);
        int numberVertices = mayaRestVertices.length();

        eigenRestVertices.resize(numberVertices, 3);

        for (unsigned int i = 0; i < numberVertices; ++i)
        {
            eigenRestVertices.row(i) << mayaRestVertices[i].x, mayaRestVertices[i].y, mayaRestVertices[i].z;
        }
        enableRecalcualte = false;
    }


    if (epsilonUpdated)
    {
        epsilonUpdated = false;
    }
    if (maxInfluentUpdated == true)
    {
        //store rest MayaControlPoints

        // Retrieve maxInfluence value
        
        int vertexNumber = mayaRestVertices.length();
        CHECK_MSTATUS_AND_RETURN_IT(status);



        // Create an MArrayDataBuilder for the multi-attribute
        MArrayDataBuilder builder(&dataBlock, aClosestIndices, vertexCount, &status);
        CHECK_MSTATUS_AND_RETURN_IT(status);
        // Create an MArrayDataBuilder for the multi-attribute
        MArrayDataBuilder weightbuilder(&dataBlock, aWeightBasedClosestIndices, vertexCount, &status);
        CHECK_MSTATUS_AND_RETURN_IT(status);
        // Center
        MArrayDataHandle centerInfVerticesHandle = dataBlock.outputArrayValue(aCenterOfInfluenceVertex, &status);
        MArrayDataBuilder centerBuilder(&dataBlock,aCenterOfInfluenceVertex, vertexCount, &status);
        CHECK_MSTATUS_AND_RETURN_IT(status);


        //for (unsigned int i = 0; i < vertexNumber; ++i)
        for (; !iter.isDone(); iter.next())
        {
            unsigned ptindex = iter.index();
            MPoint pt = iter.position();


            // Get the closest control points for the current vertex
            std::vector<int> indexInfluent = getNClosestControlPoints(ptindex, maxInfluence, eigenRestVertices, RestEigenControlPoints);

            // Add an element to the builder
            MDataHandle hElement = builder.addElement(ptindex, &status);
            CHECK_MSTATUS_AND_RETURN_IT(status);
            // Add an element to the weightbuilder
            MDataHandle hWeightElement = weightbuilder.addElement(ptindex, &status);
            CHECK_MSTATUS_AND_RETURN_IT(status);


            // Create and set the data for this element
            MFnIntArrayData intArrayDataFn;
            MIntArray intArray;
            MFnDoubleArrayData DoubleArrayWeightsDataFn;
            MDoubleArray doubleArrayWeights;


            // Populate intArray with values from indexInfluent
            double sumdis = 0;
            std::vector<double> dis;
            dis.resize(maxInfluence);
            MPoint center = MPoint();
            for (size_t idx = 0; idx < maxInfluence; ++idx)
            {
                unsigned int vertexInfindex = indexInfluent[idx];
                intArray.append(vertexInfindex);
                MPoint rpt = mayaRestControlPoints[vertexInfindex];
                center += rpt;
                double r = sqrt(pow((rpt.x - pt.x), 2) + pow((rpt.y - pt.y), 2) + pow((rpt.z - pt.z), 2));

                sumdis += r;
                dis[idx] = r;
            }
            // store center points
            center.x /= bMaxInfluen;
            center.y /= bMaxInfluen;
            center.z /= bMaxInfluen;
            

            MDataHandle centerElementHandle = centerBuilder.addElement(ptindex, &status);
            centerElementHandle.setMFloatVector(MVector(center.x,center.y,center.z));
            
            std::vector<double> weights;
            double sumWeights = 0.0;
            double eps = 1e-8; // Small value to prevent division by zero


            for (size_t idx = 0; idx < maxInfluence; ++idx)
            {
                double weight = 1.0 / std::pow(dis[idx] + eps, 2);
                weights.push_back(weight);
                sumWeights += weight;

            }
            // Normalize weights
            for (double& weight : weights)
            {
                weight /= sumWeights;
                doubleArrayWeights.append(weight);
            }

            // Create an MObject from the MIntArray
            MObject intArrayDataObj = intArrayDataFn.create(intArray, &status);
            CHECK_MSTATUS_AND_RETURN_IT(status);
            // Create an MObject from the doubleArrayWeights
            MObject doubleArrayWeightsObj = DoubleArrayWeightsDataFn.create(doubleArrayWeights, &status);
            CHECK_MSTATUS_AND_RETURN_IT(status);

            // **Set the data to the element handle using setMObject**
            hElement.setMObject(intArrayDataObj);
            // **Set the data to the element handle using setMObject**
            hWeightElement.setMObject(doubleArrayWeightsObj);


            // Optionally, mark the element handle as clean
            hElement.setClean();
            hWeightElement.setClean();



        }

        // Get the output array handle and set the builder
        MArrayDataHandle hArray = dataBlock.outputArrayValue(aClosestIndices, &status);
        CHECK_MSTATUS_AND_RETURN_IT(status);

        status = hArray.set(builder);
        CHECK_MSTATUS_AND_RETURN_IT(status);

        // Mark the array as clean
        hArray.setAllClean();

        // Get the output array handle and set the builder
        MArrayDataHandle hDoubleArray = dataBlock.outputArrayValue(aWeightBasedClosestIndices, &status);
        CHECK_MSTATUS_AND_RETURN_IT(status);

        status = hDoubleArray.set(weightbuilder);
        CHECK_MSTATUS_AND_RETURN_IT(status);

                // Mark the array as clean
        hDoubleArray.setAllClean();
        // center clean
        centerInfVerticesHandle.set(centerBuilder);
        centerInfVerticesHandle.setAllClean();



        maxInfluentUpdated = false;
    }

    //   //


    for (; !iter.isDone(); iter.next())
    {
        // Access the output array attribute
        MArrayDataHandle weightIndicesHandle = dataBlock.outputArrayValue(aWeightBasedClosestIndices, &status);
        CHECK_MSTATUS_AND_RETURN_IT(status);

        // Get the number of elements in the array
        unsigned int numElements = weightIndicesHandle.elementCount(&status);
        CHECK_MSTATUS_AND_RETURN_IT(status);

        // Access the output array attribute
        MArrayDataHandle closestPointConstrolIndicesHandle = dataBlock.outputArrayValue(aClosestIndices, &status);
        CHECK_MSTATUS_AND_RETURN_IT(status);

        // Get the number of elements in the array
        unsigned int numControlElements = closestPointConstrolIndicesHandle.elementCount(&status);
        CHECK_MSTATUS_AND_RETURN_IT(status);


        
        //unsigned int vertexacount = iter.count();
        if (vertexCount != numElements && vertexCount != numControlElements)
        {
            //MGlobal::displayError(MString("vertexacount != numElements && vertexacount != numControlElements is not valid"));
            status = MS::kFailure;
            return status;
        }
        unsigned ptindex = iter.index();
        // Jump to the current array element
        status = weightIndicesHandle.jumpToArrayElement(ptindex);
        CHECK_MSTATUS_AND_RETURN_IT(status);
        //Jump to the current closest controlpoint index
        status = closestPointConstrolIndicesHandle.jumpToArrayElement(ptindex);
        CHECK_MSTATUS_AND_RETURN_IT(status);

        // Get the data handle for the current element
        MDataHandle elementHandle = weightIndicesHandle.inputValue(&status);
        CHECK_MSTATUS_AND_RETURN_IT(status);
        MDataHandle closestIndicesElement = closestPointConstrolIndicesHandle.inputValue(&status);
        CHECK_MSTATUS_AND_RETURN_IT(status);

        // Get the MObject containing the MDoubleArrayData
        MObject doubleArrayDataObj = elementHandle.data();
        CHECK_MSTATUS_AND_RETURN_IT(status);
        MObject closestArrayIndicesDataObj = closestIndicesElement.data();
        CHECK_MSTATUS_AND_RETURN_IT(status);

        // Create an MFnDoubleArrayData function set to access the data
        MFnDoubleArrayData doubleArrayDataFn(doubleArrayDataObj, &status);
        CHECK_MSTATUS_AND_RETURN_IT(status);
        MFnIntArrayData closestArrayDataFn(closestArrayIndicesDataObj, &status);
        CHECK_MSTATUS_AND_RETURN_IT(status);

        // Get the MDoubleArray from the function set
        MDoubleArray doubleArray = doubleArrayDataFn.array(&status);
        CHECK_MSTATUS_AND_RETURN_IT(status);
        MIntArray indiceArray = closestArrayDataFn.array(&status);
        // Now you have the double array for the current element
        // You can process it as needed
        //MGlobal::displayInfo(MString("Element ") + i + ":");
        

        if (doubleArray.length() != indiceArray.length())
        {
            status = MS::kFailure;
            //MGlobal::displayError(MString(" doubleArray.length() != indiceArray.length() is not fited"));
            return status;
        }


        // center
        MArrayDataHandle CenterHandle = dataBlock.inputArrayValue(aCenterOfInfluenceVertex, &status);
        CHECK_MSTATUS_AND_RETURN_IT(status);
        CenterHandle.jumpToElement(ptindex);
        MDataHandle datacenterHandle = CenterHandle.inputValue(&status);
        MVector Origcenter = datacenterHandle.asVector();

        MPoint currentCenter = MPoint();
        for (size_t idx = 0; idx < maxInfluence; ++idx)
        {
            unsigned int cindex = indiceArray[idx];
            MPoint rpt = mayaControlPoints[cindex];
            currentCenter += rpt;

        }
        // store center points
        currentCenter.x /= bMaxInfluen;
        currentCenter.y /= bMaxInfluen;
        currentCenter.z /= bMaxInfluen;
    

        MVector moved = MVector((currentCenter.x- Origcenter.x)*envelope, (currentCenter.y - Origcenter.y) * envelope, (currentCenter.z - Origcenter.z) * envelope);
        MPoint deltaPos = MPoint();
        deltaPos.x = moved.x; deltaPos.y = moved.y; deltaPos.z = moved.z;
        
        MPoint pt = iter.position() + deltaPos;
        //MPoint pt(deformation(ptindex, 0) * envelope, deformation(ptindex, 1) * envelope, deformation(ptindex, 2) * envelope);
        iter.setPosition(pt);
    }
    //


    return MS::kSuccess;
}

std::vector<int> RBFDeformerNode::getNClosestControlPoints(int vertexIndex, int n, const Eigen::MatrixXd& vertices, const Eigen::MatrixXd& controlPoints)
{
    // Get the position of the input mesh vertex at index i as a row vector
    Eigen::RowVector3d vertexPosition = vertices.row(vertexIndex);

    // Number of control points
    int numControlPoints = controlPoints.rows();

    // Vector to store distances and corresponding control point indices
    std::vector<std::pair<double, int>> distanceIndexPairs;
    distanceIndexPairs.reserve(numControlPoints); // Reserve space for efficiency

    // Compute distances from the input vertex to each control point
    for (int j = 0; j < numControlPoints; ++j) {
        // Ensure controlPoints.row(j) is also a row vector
        Eigen::RowVector3d controlPointPosition = controlPoints.row(j);

        // Subtract row vectors and compute the norm
        double distance = (vertexPosition - controlPointPosition).norm();
        distanceIndexPairs.emplace_back(distance, j);
    }

    // Handle the case where n exceeds the number of control points
    int numInfluences = std::min(n, numControlPoints);

    // Partially sort the vector to get the n closest control points
    std::nth_element(distanceIndexPairs.begin(), distanceIndexPairs.begin() + numInfluences, distanceIndexPairs.end(),
        [](const std::pair<double, int>& a, const std::pair<double, int>& b) {
            return a.first < b.first;
        });

    // Extract the indices of the n closest control points
    std::vector<int> closestIndices;
    closestIndices.reserve(numInfluences);
    for (int k = 0; k < numInfluences; ++k) {
        closestIndices.push_back(distanceIndexPairs[k].second);
    }

    // Optional: Sort the indices based on distance (smallest to largest)
    std::sort(closestIndices.begin(), closestIndices.end(),
        [&distanceIndexPairs](int a, int b) {
            return distanceIndexPairs[a].first < distanceIndexPairs[b].first;
        });

    return closestIndices;
}


MStatus initializePlugin(MObject obj) {
    MFnPlugin plugin(obj, "YourName", "1.0", "Any");
    return plugin.registerNode("RBFDeformerNode", RBFDeformerNode::id,
        RBFDeformerNode::creator, RBFDeformerNode::initialize, MPxNode::kDeformerNode);
}

MStatus uninitializePlugin(MObject obj) {
    MFnPlugin plugin(obj);
    return plugin.deregisterNode(RBFDeformerNode::id);
}