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
class RBFDeformerNode : public MPxDeformerNode {
public:
    static MTypeId id;
    static MObject aEpsilon;
    static MObject aControlMesh;
    static MObject aControlMeshTransform;
    static MObject aMaxInfluence;
    
    
    
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
    Eigen::MatrixXd weightsMatrixUpdated;
    Eigen::MatrixXd deformedControlPoints;
    Eigen::MatrixXd weightsMatrixOrig;
    bool epsilonUpdated = false;
    bool maxInfluentUpdated = false;

    
   
};

MTypeId RBFDeformerNode::id(0x00002);
MObject RBFDeformerNode::aEpsilon;
MObject RBFDeformerNode::aControlMesh;
MObject RBFDeformerNode::aControlMeshTransform;
MObject RBFDeformerNode::aMaxInfluence;
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
    aControlMeshTransform = mAttr.create("inMeshMatrix","iMMatrix", MFnMatrixAttribute::kDouble,&status);
    mAttr.setConnectable(true);
    mAttr.setStorable(true);
    addAttribute(aControlMeshTransform);
    // Attribute affects

    attributeAffects(aControlMesh, outputGeom);
    attributeAffects(aEpsilon, outputGeom);
    attributeAffects(aControlMeshTransform,outputGeom);

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
    if (hasControlMesh==false) {
        MGlobal::displayWarning("control mesh is not connected");
        MGlobal::displayWarning(MString("should recalculate weights :")+(enableRecalcualte) );
        return status;
    }
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
        MGlobal::displayInfo(MString("RestPoint: ") + mayaControlPoints[i].x + " " + mayaControlPoints[i].y + " " + mayaControlPoints[i].z);
    }
    if (enableRecalcualte)
    {
        MGlobal::displayWarning("update logic when controlMeshSourceChanged.");
        // -----------rest mesh---------------------------
        RestEigenControlPoints = EigenControlPoints;
        iter.allPositions(mayaRestVertices);
        int numberVertices = mayaRestVertices.length();
        
        eigenRestVertices.resize(numberVertices,3);

        for (unsigned int i = 0; i < numberVertices; ++i)
        {
            eigenRestVertices.row(i) << mayaRestVertices[i].x, mayaRestVertices[i].y, mayaRestVertices[i].z;
            MGlobal::displayInfo(MString("restMesh: ") + mayaRestVertices[i].x + " " + mayaRestVertices[i].y + " " + mayaRestVertices[i].z);
        }
        
       
        computeInitialWeights(eigenRestVertices, RestEigenControlPoints, weightsMatrixOrig);
        updateWeightsAndOffsets(weightsMatrixOrig,epsilon, EigenControlPoints,weightsMatrixUpdated,offsets,eigenRestVertices);
        
        
        //---------------------update paramter for next calulation---------
        enableRecalcualte = false;
        MGlobal::displayWarning(MString("set enableRecalcualte: ") + (enableRecalcualte));
        MGlobal::displayInfo(MString("number of verticescontrolPoint:") + (numberControlPoints));
    }
    if(epsilonUpdated)
    {
        updateWeightsAndOffsets(weightsMatrixOrig, epsilon, RestEigenControlPoints, weightsMatrixUpdated, offsets, eigenRestVertices);
        epsilonUpdated = false;
    }
    if (maxInfluentUpdated == true)
    {
        // Retrieve maxInfluence value
        int maxInfluence = dataBlock.inputValue(aMaxInfluence, &status).asInt();
        CHECK_MSTATUS_AND_RETURN_IT(status);

        std::vector<int> indexInfluent = getNClosestControlPoints(204,maxInfluence,eigenRestVertices,RestEigenControlPoints);
        for (unsigned ind : indexInfluent)
        {
            MGlobal::displayInfo(MString("indexInfluent: ") + ind);
        }
        MGlobal::displayInfo(MString("maxInfluentUpdated: ")+ maxInfluence);
    }

       //
    //deformedControlPoints=
    Eigen::MatrixXd deformation = applyRBFDeformation(weightsMatrixUpdated, offsets, EigenControlPoints);
    //for (unsigned int i = 0; i < deformation.rows(); ++i) {
    //    MPoint pt(iter.position().x + deformation(i, 0) * envelope,
    //        iter.position().y + deformation(i, 1) * envelope,
    //        iter.position().z + deformation(i, 2) * envelope);
    //    iter.setPosition(pt);
    for (; !iter.isDone(); iter.next())
    {
        unsigned ptindex = iter.index();
        MPoint pt(deformation(ptindex, 0) * envelope, deformation(ptindex, 1) * envelope, deformation(ptindex, 2) * envelope);
        iter.setPosition(pt);
    }
    MGlobal::displayWarning(MString("hasControlMesh: ") + (hasControlMesh));
    MGlobal::displayWarning(MString("enableRecalcualte: ") + (enableRecalcualte));
    MGlobal::displayInfo(MString("number of mayaRestVertices:") + (mayaRestVertices.length()));
    MGlobal::displayInfo(MString("number of EigenControlPoints:") + EigenControlPoints.rows());
    MGlobal::displayInfo(MString("number of weightsMatrixOrig:") + weightsMatrixOrig.rows());
    MGlobal::displayInfo(MString("number  column of weightsMatrixOrig:") + weightsMatrixOrig.cols());
    MGlobal::displayInfo(MString("number of weightsMatrixUpdated:") + weightsMatrixUpdated.rows());
    MGlobal::displayInfo(MString("epsilon:") + epsilon);
    

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

//std::vector<int>RBFDeformerNode::getNClosestControlPoints(int vertexIndex, int n,
//    const Eigen::MatrixXd& vertices, const Eigen::MatrixXd& controlPoints) {
//
//    // Get the position of the input mesh vertex at index i
//    Eigen::Vector3d vertexPosition = vertices.row(vertexIndex);
//
//    // Number of control points
//    int numControlPoints = controlPoints.rows();
//
//    // Vector to store distances and corresponding control point indices
//    std::vector<std::pair<double, int>> distanceIndexPairs;
//
//    // Compute distances from the input vertex to each control point
//    for (int j = 0; j < numControlPoints; ++j) {
//        double distance = (vertexPosition - controlPoints.row(j)).norm();
//        distanceIndexPairs.emplace_back(distance, j);
//    }
//
//    // Partially sort the vector to get the n closest control points
//    std::nth_element(distanceIndexPairs.begin(), distanceIndexPairs.begin() + n, distanceIndexPairs.end(),
//        [](const std::pair<double, int>& a, const std::pair<double, int>& b) {
//            return a.first < b.first;
//        });
//
//    // Extract the indices of the n closest control points
//    std::vector<int> closestIndices;
//    closestIndices.reserve(n);
//    for (int k = 0; k < n && k < numControlPoints; ++k) {
//        closestIndices.push_back(distanceIndexPairs[k].second);
//    }
//
//    // Optional: Sort the indices based on distance (smallest to largest)
//    std::sort(closestIndices.begin(), closestIndices.end(),
//        [&distanceIndexPairs](int a, int b) {
//            return distanceIndexPairs[a].first < distanceIndexPairs[b].first;
//        });
//
//    return closestIndices;
//}