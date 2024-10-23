
#ifndef RBF_POINTS_H
#define RBF_POINTS_H

#include <maya/MPxNode.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MArrayDataHandle.h>
#include <maya/MGlobal.h>
#include <maya/MVector.h>
#include<maya/MFnTypedAttribute.h>
#include<BasicMatrix.h>
#include<string>
#include<sstream>
#include<maya/MPlug.h>
#include<maya/MFnMesh.h>
#include<maya/MFloatPointArray.h>
// Node Class Definition
class RBFPoints : public MPxNode {
public:
    RBFPoints() {}
    virtual ~RBFPoints() override {}

    // Creator function to create an instance of the node
    static void* creator();

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
    static MObject inputPositions;  // Array of locator positions (double3)
    static MObject outputAttribute; // Output attribute for demonstration
    static MObject restPosition; // New attribute to store the updated positions
    static MObject inputMesh;// mesh to want to use get (x,z)
    static MObject outputMesh;
    static MObject deformedMesh;
    //--------------other attributes
    Eigen::MatrixXd eigenDistanceMatrix;// Cached distance matrix as an Eigen matrix
    std::vector<double> weights;
    MObject inMesh;// cached inputmesh plug
    MObject indeformedMesh;// cached deformedMesh plug can remove
    MObject outMeshData;
    std::vector<std::pair<double, double>> vertexXZ;  // Vector to store vertex x and z coordinates
    MFloatPointArray inPoints;//vertex of restmesh
    MFloatPointArray outPoints;//vertex of restmesh
    std::vector<ctsdev::vector3> inputPositionsToStd;
    double best_epsilon = 0;
    // Helper function for RBF
    void rbf(MDataBlock& dataBlock, MStatus& status);
    // New function to set default values for inputPositions and restPosition
    void defaultInput(MDataBlock& dataBlock);
    // Function to update distance matrix when restPosition changes and store it in Eigen format
    void updateDistanceMatrix(const std::vector<ctsdev::vector3>& restPositions, Eigen::MatrixXd& eigenDistanceMatrix);
    static MStatus initAttrs(MFnNumericAttribute& nAttr, MFnTypedAttribute& tAttr);
    MStatus getVectorValuesFromPlug(const MPlug& plug, std::vector<ctsdev::vector3>& outValues);
    MStatus updateMeshYValues(const MObject& inMesh, MFloatPointArray& inPoints, const std::vector<double>& weights, const std::vector<ctsdev::vector3>& controlPoints);
    MStatus storeVertexXZ(const MObject& inMesh);
    MStatus storeVertexPositions(const MObject& inMesh, MFloatPointArray &inPoints);
    //std::vector<ctsdev::vector3>MayaPointToVector; //cached 
    static std::vector<ctsdev::vector3> MayaPointToVector(MDataBlock& dataBlock, const MObject& attribute, MStatus& status);
    static void printMayaPointToVector(const std::vector<ctsdev::vector3>& points);
};

#endif
