import pandas as pd
import numpy as np
import os
import struct
import sys

# Get script's directory and project root
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
DATA_DIR = os.path.join(PROJECT_ROOT, "data")

def prepare_training_data(input_csv=os.path.join(DATA_DIR, "training_data.csv"), 
                         output_npy=os.path.join(DATA_DIR, "processed_data.npz"),
                         output_bin=os.path.join(DATA_DIR, "processed_data.bin"),
                         subsample_factor=1):
    """
    Read training_data.csv, process data, and save for ML training.
    
    Args:
        input_csv (str): Path to input CSV file (relative to project root).
        output_npy (str): Path to save NumPy arrays.
        output_bin (str): Path to save binary data for C++.
        subsample_factor (int): Use every nth frame (1 = no subsampling).
    
    Returns:
        bool: True if successful, False otherwise.
    """
    # Ensure data directory exists
    os.makedirs(DATA_DIR, exist_ok=True)

    # Validate input file
    if not os.path.exists(input_csv):
        print(f"Error: {input_csv} not found!")
        return False

    # Read CSV
    try:
        df = pd.read_csv(input_csv, header=None)
        print(f"Loaded {len(df)} frames from {input_csv}")
    except Exception as e:
        print(f"Error reading CSV: {e}")
        return False

    # Subsample frames
    df = df[::subsample_factor]
    print(f"Subsampled to {len(df)} frames (factor: {subsample_factor})")

    # Validate shape
    expected_columns = 1 + 16 + 98 * 3  # Frame + 4x4 matrix + 98 vertices
    if df.shape[1] != expected_columns:
        print(f"Error: Expected {expected_columns} columns, got {df.shape[1]}")
        return False

    # Extract data
    frames = df.iloc[:, 0].values  # Frame numbers (optional)
    inputs = df.iloc[:, 1:17].values  # 4x4 matrix (16 values)
    outputs = df.iloc[:, 17:].values  # 98 vertices * 3 (294 values)

    # Verify shapes
    if inputs.shape[1] != 16 or outputs.shape[1] != 98 * 3:
        print(f"Error: Invalid input shape {inputs.shape} or output shape {outputs.shape}")
        return False

    # Normalize data
    input_mean = np.mean(inputs, axis=0)
    input_std = np.std(inputs, axis=0)
    input_std[input_std == 0] = 1
    inputs_normalized = (inputs - input_mean) / input_std

    output_mean = np.mean(outputs, axis=0)
    output_std = np.std(outputs, axis=0)
    output_std[output_std == 0] = 1
    outputs_normalized = (outputs - output_mean) / output_std

    # Save NumPy data
    try:
        np.savez(output_npy, 
                 inputs=inputs_normalized, 
                 outputs=outputs_normalized, 
                 input_mean=input_mean, 
                 input_std=input_std, 
                 output_mean=output_mean, 
                 output_std=output_std,
                 frames=frames)
        print(f"Saved NumPy data to {output_npy}")
    except Exception as e:
        print(f"Error saving NumPy data: {e}")
        return False

    # Save binary data for C++
    try:
        with open(output_bin, "wb") as f:
            f.write(struct.pack("iii", len(inputs), 16, 98 * 3))
            for i in range(len(inputs)):
                f.write(struct.pack("f" * 16, *inputs_normalized[i]))
                f.write(struct.pack("f" * (98 * 3), *outputs_normalized[i]))
            f.write(struct.pack("f" * 16, *input_mean))
            f.write(struct.pack("f" * 16, *input_std))
            f.write(struct.pack("f" * (98 * 3), *output_mean))
            f.write(struct.pack("f" * (98 * 3), *output_std))
        print(f"Saved binary data to {output_bin}")
    except Exception as e:
        print(f"Error saving binary data: {e}")
        return False

    return True

def load_processed_data(file_path=os.path.join(DATA_DIR, "processed_data.npz")):
    """
    Load processed NumPy data.
    
    Args:
        file_path (str): Path to .npz file.
    
    Returns:
        dict: Dictionary with inputs, outputs, and normalization parameters.
    """
    if not os.path.exists(file_path):
        print(f"Error: {file_path} not found!")
        return None

    try:
        data = np.load(file_path)
        return {
            "inputs": data["inputs"],
            "outputs": data["outputs"],
            "input_mean": data["input_mean"],
            "input_std": data["input_std"],
            "output_mean": data["output_mean"],
            "output_std": data["output_std"],
            "frames": data["frames"]
        }
    except Exception as e:
        print(f"Error loading data: {e}")
        return None

def load_binary_data(file_path=os.path.join(DATA_DIR, "processed_data.bin")):
    """
    Load binary data for C++ training.
    
    Args:
        file_path (str): Path to .bin file.
    
    Returns:
        dict: Dictionary with inputs, outputs, and normalization parameters.
    """
    if not os.path.exists(file_path):
        print(f"Error: {file_path} not found!")
        return None

    try:
        with open(file_path, "rb") as f:
            num_samples, input_dim, output_dim = struct.unpack("iii", f.read(12))
            inputs = np.zeros((num_samples, input_dim), dtype=np.float32)
            outputs = np.zeros((num_samples, output_dim), dtype=np.float32)
            for i in range(num_samples):
                inputs[i] = struct.unpack("f" * input_dim, f.read(4 * input_dim))
                outputs[i] = struct.unpack("f" * output_dim, f.read(4 * output_dim))
            input_mean = np.array(struct.unpack("f" * input_dim, f.read(4 * input_dim)))
            input_std = np.array(struct.unpack("f" * input_dim, f.read(4 * input_dim)))
            output_mean = np.array(struct.unpack("f" * output_dim, f.read(4 * output_dim)))
            output_std = np.array(struct.unpack("f" * output_dim, f.read(4 * output_dim)))
        return {
            "inputs": inputs,
            "outputs": outputs,
            "input_mean": input_mean,
            "input_std": input_std,
            "output_mean": output_mean,
            "output_std": output_std
        }
    except Exception as e:
        print(f"Error loading binary data: {e}")
        return None

if __name__ == "__main__":
    # Support command-line argument for input CSV
    input_csv = os.path.join(DATA_DIR, "training_data.csv")
    if len(sys.argv) > 1:
        input_csv = sys.argv[1]
    output_npy = input_csv.replace(".csv", ".npz")
    output_bin = input_csv.replace(".csv", ".bin")
    
    prepare_training_data(input_csv=input_csv, output_npy=output_npy, output_bin=output_bin, subsample_factor=1)
    data = load_processed_data(output_npy)
    if data:
        print(f"NumPy Inputs shape: {data['inputs'].shape}")
        print(f"NumPy Outputs shape: {data['outputs'].shape}")
    data = load_binary_data(output_bin)
    if data:
        print(f"Binary Inputs shape: {data['inputs'].shape}")
        print(f"Binary Outputs shape: {data['outputs'].shape}")