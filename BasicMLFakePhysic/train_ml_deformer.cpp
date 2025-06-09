#include <Eigen/Dense>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <iostream>

struct TrainingData {
    std::vector<Eigen::MatrixXd> inputs;  // Transform matrix components (e.g., translation)
    std::vector<Eigen::MatrixXd> outputs; // 98 points x 3 coordinates
};

TrainingData parseCSV(const std::string& filePath) {
    TrainingData data;
    std::ifstream file(filePath);
    std::string line;

    if (!file.is_open()) {
        std::cerr << "Failed to open CSV file: " << filePath << std::endl;
        return data;
    }

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string token;
        std::vector<double> values;

        // Parse CSV line
        while (std::getline(ss, token, ',')) {
            try {
                values.push_back(std::stod(token));
            } catch (...) {
                std::cerr << "Invalid value in CSV: " << token << std::endl;
                continue;
            }
        }

        // Expected: 1 (frame) + 16 (matrix) + 294 (98 points x 3)
        if (values.size() != 311) {
            std::cerr << "Invalid row size: " << values.size() << ", expected 311" << std::endl;
            continue;
        }

        // Extract translation (matrix[3][0], matrix[3][1], matrix[3][2])
        Eigen::MatrixXd input(1, 3);
        input(0, 0) = values[13]; // matrix[3][0] (x translation)
        input(0, 1) = values[14]; // matrix[3][1] (y translation)
        input(0, 2) = values[15]; // matrix[3][2] (z translation)

        // Extract 98 points
        Eigen::MatrixXd output(98, 3);
        for (int i = 0; i < 98; ++i) {
            output(i, 0) = values[17 + i * 3];     // x
            output(i, 1) = values[17 + i * 3 + 1]; // y
            output(i, 2) = values[17 + i * 3 + 2]; // z
        }

        data.inputs.push_back(input);
        data.outputs.push_back(output);
    }

    file.close();
    return data;
}

void normalizeData(TrainingData& data, Eigen::VectorXd& inputMean, Eigen::VectorXd& inputStd, Eigen::VectorXd& outputMean, Eigen::VectorXd& outputStd) {
    Eigen::MatrixXd allInputs(data.inputs.size(), 3);
    Eigen::MatrixXd allOutputs(data.inputs.size() * 98, 3);

    for (size_t i = 0; i < data.inputs.size(); ++i) {
        allInputs.row(i) = data.inputs[i].row(0);
        for (int j = 0; j < 98; ++j) {
            allOutputs.row(i * 98 + j) = data.outputs[i].row(j);
        }
    }

    inputMean = allInputs.colwise().mean();
    inputStd = ((allInputs.rowwise() - inputMean.transpose()).array().square().colwise().sum() / (allInputs.rows() - 1)).sqrt();
    outputMean = allOutputs.colwise().mean();
    outputStd = ((allOutputs.rowwise() - outputMean.transpose()).array().square().colwise().sum() / (allOutputs.rows() - 1)).sqrt();

    // Avoid division by zero
    for (int i = 0; i < inputStd.size(); ++i) {
        if (inputStd(i) < 1e-6) inputStd(i) = 1.0;
    }
    for (int i = 0; i < outputStd.size(); ++i) {
        if (outputStd(i) < 1e-6) outputStd(i) = 1.0;
    }

    for (size_t i = 0; i < data.inputs.size(); ++i) {
        data.inputs[i].row(0) = (data.inputs[i].row(0) - inputMean.transpose()).array() / inputStd.transpose().array();
        for (int j = 0; j < 98; ++j) {
            data.outputs[i].row(j) = (data.outputs[i].row(j) - outputMean.transpose()).array() / outputStd.transpose().array();
        }
    }
}

Eigen::MatrixXd trainLinearRegression(const TrainingData& data) {
    Eigen::MatrixXd X(data.inputs.size(), 4); // 3 inputs + bias
    Eigen::MatrixXd Y(data.inputs.size(), 98 * 3); // 98 points x 3 coordinates

    for (size_t i = 0; i < data.inputs.size(); ++i) {
        X(i, 0) = data.inputs[i](0, 0);
        X(i, 1) = data.inputs[i](0, 1);
        X(i, 2) = data.inputs[i](0, 2);
        X(i, 3) = 1.0; // Bias term
        for (int j = 0; j < 98; ++j) {
            Y(i, j * 3) = data.outputs[i](j, 0);
            Y(i, j * 3 + 1) = data.outputs[i](j, 1);
            Y(i, j * 3 + 2) = data.outputs[i](j, 2);
        }
    }

    // Normal equations: (X^T X)^(-1) X^T Y
    Eigen::MatrixXd weights = (X.transpose() * X).ldlt().solve(X.transpose() * Y);
    return weights;
}

void saveModel(const Eigen::MatrixXd& weights, const Eigen::VectorXd& inputMean, const Eigen::VectorXd& inputStd,
               const Eigen::VectorXd& outputMean, const Eigen::VectorXd& outputStd, const std::string& filePath) {
    std::ofstream file(filePath);
    if (!file.is_open()) {
        std::cerr << "Failed to save model to: " << filePath << std::endl;
        return;
    }

    file << "weights\n";
    file << weights.rows() << "," << weights.cols() << "\n";
    for (int i = 0; i < weights.rows(); ++i) {
        for (int j = 0; j < weights.cols(); ++j) {
            file << weights(i, j);
            if (j < weights.cols() - 1) file << ",";
        }
        file << "\n";
    }

    file << "inputMean\n" << inputMean.transpose() << "\n";
    file << "inputStd\n" << inputStd.transpose() << "\n";
    file << "outputMean\n" << outputMean.transpose() << "\n";
    file << "outputStd\n" << outputStd.transpose() << "\n";

    file.close();
}

int main() {
    std::string filePath = "E:/dev/RBF/pointsData/BasicMLFakePhysic/training_data.csv";
    TrainingData data = parseCSV(filePath);

    if (data.inputs.empty()) {
        std::cerr << "No data loaded." << std::endl;
        return 1;
    }

    Eigen::VectorXd inputMean, inputStd, outputMean, outputStd;
    normalizeData(data, inputMean, inputStd, outputMean, outputStd);

    Eigen::MatrixXd weights = trainLinearRegression(data);
    saveModel(weights, inputMean, inputStd, outputMean, outputStd, "E:/dev/RBF/pointsData/BasicMLFakePhysic/model.csv");

    std::cout << "Model trained and saved for 98 points." << std::endl;
    return 0;
}