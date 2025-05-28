#include <maya/MPxDeformerNode.h>
#include <maya/MFnMesh.h>
#include <maya/MItGeometry.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MMatrix.h>
#include <maya/MPointArray.h>
#include <maya/MGlobal.h>
#include <maya/MFnPlugin.h>
#include <maya/MPxCommand.h>
#include <maya/MSyntax.h>
#include <maya/MArgDatabase.h>
#include <maya/MFnDagNode.h>
#include <maya/MSelectionList.h>
#include <fstream>
#include <vector>
#include <filesystem>
#include <sstream>

class MLDeformer : public MPxDeformerNode {
public:
    MLDeformer() {}
    virtual ~MLDeformer() {}

    static void* creator();
    static MStatus initialize();

    MStatus deform(MDataBlock& block, MItGeometry& iter, const MMatrix& mat, unsigned int multiIndex) override {
        MMatrix transformMatrix = block.inputValue(aTransformMatrix).asMatrix();
        double offset = transformMatrix[3][0]; // Use X translation as a test offset

        MGlobal::displayInfo(MString("Transform Matrix X: ") + transformMatrix[3][0]);

        for (; !iter.isDone(); iter.next()) {
            MPoint pt = iter.position();
            pt.x += offset; // Simple offset for testing
            iter.setPosition(pt);
        }
        return MS::kSuccess;
    }

    static MTypeId id;
    static MObject aTransformMatrix;
};

MTypeId MLDeformer::id(0x80000);
MObject MLDeformer::aTransformMatrix;

MStatus MLDeformer::initialize() {
    MFnMatrixAttribute mAttr;
    aTransformMatrix = mAttr.create("transformMatrix", "tm");
    mAttr.setReadable(true);
    mAttr.setWritable(true);
    addAttribute(aTransformMatrix);
    attributeAffects(aTransformMatrix, outputGeom);
    return MS::kSuccess;
}

void* MLDeformer::creator() {
    return new MLDeformer;
}

class MLDeformerCollectCmd : public MPxCommand {
public:
    static void* creator() { return new MLDeformerCollectCmd; }

    MStatus doIt(const MArgList& args) override {
        MArgDatabase argData(syntax(), args);
        MObject meshObj, transformObj;
        int startFrame, endFrame;

        // Parse arguments
        MStatus status;
        MSelectionList meshList, transformList;
        status = argData.getFlagArgument("-m", 0, meshList);
        if (!status) {
            MGlobal::displayError("Failed to get mesh argument.");
            return status;
        }
        status = meshList.getDependNode(0, meshObj);
        if (!status || meshObj.isNull()) {
            MGlobal::displayError("Invalid or empty mesh selection.");
            return MS::kFailure;
        }
        status = argData.getFlagArgument("-t", 0, transformList);
        if (!status) {
            MGlobal::displayError("Failed to get transform argument.");
            return status;
        }
        status = transformList.getDependNode(0, transformObj);
        if (!status || transformObj.isNull()) {
            MGlobal::displayError("Invalid or empty transform selection.");
            return MS::kFailure;
        }
        status = argData.getFlagArgument("-sf", 0, startFrame);
        if (!status) {
            MGlobal::displayError("Failed to get start frame argument.");
            return status;
        }
        status = argData.getFlagArgument("-ef", 0, endFrame);
        if (!status) {
            MGlobal::displayError("Failed to get end frame argument.");
            return status;
        }

        // Validate inputs
        if (endFrame < startFrame) {
            MGlobal::displayError("End frame must be >= start frame.");
            return MS::kFailure;
        }

        // Get mesh and transform data
        MFnMesh meshFn(meshObj, &status);
        if (!status) {
            MGlobal::displayError("Invalid mesh object.");
            return status;
        }
        MFnDagNode transformFn(transformObj, &status);
        if (!status) {
            MGlobal::displayError("Invalid transform object.");
            return status;
        }
        MPointArray vertices;

        // Use absolute path for output file
        std::string filePath = "E:/dev/RBF/pointsData/BasicMLFakePhysic/training_data.csv";
        std::ofstream file(filePath);
        if (!file.is_open()) {
            MGlobal::displayError(MString("Failed to open file: ") + filePath.c_str());
            return MS::kFailure;
        }

        // Print current working directory for debugging
        std::string cwd = std::filesystem::current_path().string();
        MGlobal::displayInfo(MString("Current working directory: ") + cwd.c_str());

        // Test file writing
        std::ofstream testFile("E:/dev/RBF/pointsData/BasicMLFakePhysic/test.txt");
        if (testFile.is_open()) {
            testFile << "Test";
            testFile.close();
            MGlobal::displayInfo("Test file written successfully.");
        } else {
            MGlobal::displayError("Failed to write test file.");
        }

        // Collect data for each frame
        for (int frame = startFrame; frame <= endFrame; ++frame) {
            MGlobal::executeCommand(MString("currentTime ") + frame);
            status = meshFn.getPoints(vertices);
            if (!status) {
                MGlobal::displayError(MString("Failed to get vertices at frame ") + frame);
                file.close();
                return status;
            }
            MMatrix matrix = transformFn.transformationMatrix(&status);
            if (!status) {
                MGlobal::displayError(MString("Failed to get transform matrix at frame ") + frame);
                file.close();
                return status;
            }

            MGlobal::displayInfo(MString("Frame ") + frame + ": Vertices collected: " + vertices.length());

            // Write frame number, transform matrix, and vertex positions
            file << frame << ",";
            for (int i = 0; i < 4; ++i)
                for (int j = 0; j < 4; ++j)
                    file << matrix(i, j) << ",";
            for (unsigned int i = 0; i < vertices.length(); ++i)
                file << vertices[i].x << "," << vertices[i].y << "," << vertices[i].z << ",";
            file << "\n";
            file.flush();
            MGlobal::displayInfo(MString("Wrote frame ") + frame);
        }
        file.close();
        // Fixed concatenation
        std::ostringstream oss;
        oss << "Data collected for frames " << startFrame << " to " << endFrame << " and saved to " << filePath;
        MGlobal::displayInfo(MString(oss.str().c_str()));
        return MS::kSuccess;
    }

    static MSyntax newSyntax() {
        MSyntax syntax;
        syntax.addFlag("-m", "-mesh", MSyntax::kSelectionItem);
        syntax.addFlag("-t", "-transform", MSyntax::kSelectionItem);
        syntax.addFlag("-sf", "-startFrame", MSyntax::kLong);
        syntax.addFlag("-ef", "-endFrame", MSyntax::kLong);
        return syntax;
    }
};

MStatus initializePlugin(MObject obj) {
    MFnPlugin plugin(obj, "MLDeformer", "1.0", "Any");
    MStatus status = plugin.registerNode("mlDeformer", MLDeformer::id, MLDeformer::creator, MLDeformer::initialize, MPxNode::kDeformerNode);
    if (status) {
        status = plugin.registerCommand("mlDeformerCollect", MLDeformerCollectCmd::creator, MLDeformerCollectCmd::newSyntax);
    }
    return status;
}

MStatus uninitializePlugin(MObject obj) {
    MFnPlugin plugin(obj);
    MStatus status = plugin.deregisterCommand("mlDeformerCollect");
    if (status) {
        status = plugin.deregisterNode(MLDeformer::id);
    }
    return status;
}