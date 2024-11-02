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
    
    Eigen::MatrixXd deformedControlPoints;
    Eigen::MatrixXd weightsMatrixOrig;
    Eigen::MatrixXd weightsMatrixUpdated;
    Eigen::MatrixXd offsets;
    bool isInitialized = false;

    static void* creator() { return new RBFDeformerNode(); }
    static MStatus initialize();

    void computeInitialWeightsAndOffsets(const Eigen::MatrixXd& vertices,
        const Eigen::MatrixXd& controlPoints,
        Eigen::MatrixXd& weightsMatrixOrig,
        Eigen::MatrixXd& offsets);
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
private:
    //bool controlMeshChanged = false;
    //bool controlMeshSourceChanged = false;
    bool hasControlMesh;
    bool enableRecalcualte = true;
    MPointArray mayaRestVertices;//maya
    Eigen::MatrixXd eigenRestVertices;

   
};

MTypeId RBFDeformerNode::id(0x00002);
MObject RBFDeformerNode::aEpsilon;
MObject RBFDeformerNode::aControlMesh;
MObject RBFDeformerNode::aControlMeshTransform;
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

    return MS::kSuccess;
}


void RBFDeformerNode::computeInitialWeightsAndOffsets(const Eigen::MatrixXd& vertices,
    const Eigen::MatrixXd& controlPoints,
    Eigen::MatrixXd& weightsMatrixOrig,
    Eigen::MatrixXd& offsets) {
    Eigen::Index numVertices = vertices.rows();
    Eigen::Index numControlPoints = controlPoints.rows();

    weightsMatrixOrig.resize(numVertices, numControlPoints);
    offsets.resize(numVertices, 3);

    for (Eigen::Index i = 0; i < numVertices; ++i) {
        Eigen::VectorXd distances(numControlPoints);
        for (Eigen::Index j = 0; j < numControlPoints; ++j) {
            distances(j) = (vertices.row(i) - controlPoints.row(j)).norm();
        }
        double sumDistances = distances.sum();
        Eigen::VectorXd weights = distances / sumDistances;
        weightsMatrixOrig.row(i) = weights;

        Eigen::Vector3d originalVertex = weights.transpose() * controlPoints;
        offsets.row(i) = originalVertex.transpose() - vertices.row(i);
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
        weights /= weights.sum();
        weightsMatrixUpdated.row(i) = weights;

        Eigen::Vector3d originalVertex = weights.transpose() * controlPoints;
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
    
    if ((hasControlMesh == true) && enableRecalcualte)
    {
        MGlobal::displayWarning("update logic when controlMeshSourceChanged.");
        // -----------rest mesh---------------------------
        iter.allPositions(mayaRestVertices);
        int numberVertices = mayaRestVertices.length();
        
        eigenRestVertices.resize(numberVertices,3);

        for (unsigned int i = 0; i < numberVertices; ++i)
        {
            eigenRestVertices.row(i) << mayaRestVertices[i].x, mayaRestVertices[i].y, mayaRestVertices[i].z;
            MGlobal::displayInfo(MString("restMesh: ") + mayaRestVertices[i].x + " " + mayaRestVertices[i].y + " " + mayaRestVertices[i].z);
        }
        
        // -----------rest points---------------------------
        MObject controlMeshObj = dataBlock.inputValue(aControlMesh, &status).asMesh();
        MFnMesh controlMeshFun(controlMeshObj,&status);
        // Define an array to store vertex positions
        MPointArray mayaControlPoints;
        // Retrieve all vertex positions in world space
        status = controlMeshFun.getPoints(mayaControlPoints, MSpace::kWorld);
        CHECK_MSTATUS_AND_RETURN_IT(status);
        int numberControlPoints = mayaControlPoints.length();
        
        Eigen::MatrixXd EigenControlPoints(numberControlPoints, 3);
        for (unsigned int i = 0; i < numberControlPoints; ++i)
        {
            EigenControlPoints.row(i) << mayaControlPoints[i].x, mayaControlPoints[i].y, mayaControlPoints[i].z;
            MGlobal::displayInfo(MString("RestPoint: ") + mayaControlPoints[i].x + " " + mayaControlPoints[i].y + " " + mayaControlPoints[i].z);
        }

        //---------------------update paramter for next calulation---------
        enableRecalcualte = false;
        MGlobal::displayWarning(MString("set enableRecalcualte: ") + (enableRecalcualte));
    }
    MGlobal::displayWarning(MString("hasControlMesh: ") + (hasControlMesh));
    MGlobal::displayWarning(MString("enableRecalcualte: ") + (enableRecalcualte));
    MGlobal::displayInfo(MString("number of vertices:") + (mayaRestVertices.length()));
    //set changed to default
    //controlMeshChanged = false;

    //MGlobal::displayInfo(MString("controlMeshChanged:") + controlMeshChanged);
    //if (controlMeshObj.isNull()) {
    //    MGlobal::displayWarning("Control mesh is null.");
    //    return MS::kSuccess;
    //}

    //if (!isInitialized) {
    //    MPointArray inputPoints;
    //    iter.allPositions(inputPoints);
    //    Eigen::MatrixXd vertices(inputPoints.length(), 3);
    //    for (unsigned int i = 0; i < inputPoints.length(); ++i) {
    //        vertices.row(i) << inputPoints[i].x, inputPoints[i].y, inputPoints[i].z;
    //    }

    //    MFnMesh controlMeshFn(controlMeshObj);
    //    MPointArray controlPointsArray;
    //    controlMeshFn.getPoints(controlPointsArray, MSpace::kWorld);

    //    restControlPoints.resize(controlPointsArray.length(), 3);
    //    deformedControlPoints.resize(controlPointsArray.length(), 3);
    //    for (unsigned int i = 0; i < controlPointsArray.length(); ++i) {
    //        restControlPoints.row(i) << controlPointsArray[i].x, controlPointsArray[i].y, controlPointsArray[i].z;
    //        deformedControlPoints.row(i) = restControlPoints.row(i);
    //    }
    //    computeInitialWeightsAndOffsets(vertices, restControlPoints, weightsMatrixOrig, offsets);
    //    isInitialized = true;
    //}

    ////// Check if the control mesh has changed
    ////bool controlMeshChanged = false;
    ////MFnMesh controlMeshFn(controlMeshObj);
    ////MPointArray controlPointsArray;
    ////controlMeshFn.getPoints(controlPointsArray, MSpace::kWorld);

    ////if (controlPointsArray.length() != deformedControlPoints.rows()) {
    ////    controlMeshChanged = true;
    ////}
    ////else {
    ////    for (unsigned int i = 0; i < controlPointsArray.length(); ++i) {
    ////        if (controlPointsArray[i].x != deformedControlPoints(i, 0) ||
    ////            controlPointsArray[i].y != deformedControlPoints(i, 1) ||
    ////            controlPointsArray[i].z != deformedControlPoints(i, 2)) {
    ////            controlMeshChanged = true;
    ////            break;
    ////        }
    ////    }
    ////}

    ////if (controlMeshChanged) {
    ////    MGlobal::displayInfo("Control mesh updated");
    ////}

    //// Existing deformation application logic remains unchanged
    //Eigen::MatrixXd deformation = applyRBFDeformation(weightsMatrixUpdated, offsets, deformedControlPoints);
    //for (unsigned int i = 0; i < deformation.rows(); ++i) {
    //    MPoint pt(iter.position().x + deformation(i, 0) * envelope,
    //        iter.position().y + deformation(i, 1) * envelope,
    //        iter.position().z + deformation(i, 2) * envelope);
    //    iter.setPosition(pt);
    //}

    return MS::kSuccess;
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