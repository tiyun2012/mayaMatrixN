# Set MAYA_LOCATION
[System.Environment]::SetEnvironmentVariable("MAYA_LOCATION", "C:\Program Files\Autodesk\Maya2024", "User")

# Set DEVKIT_LOCATION
[System.Environment]::SetEnvironmentVariable("DEVKIT_LOCATION", "E:\dev\RBF\devkit2024", "User")

# Get the current value of MAYA_LOCATION from the user environment
$mayaLocation = [System.Environment]::GetEnvironmentVariable("MAYA_LOCATION", "User")

# Create the full path to the bin directory
$mayaBinPath = Join-Path $mayaLocation "bin"

# Get the current PATH for the user
$currentPath = [System.Environment]::GetEnvironmentVariable("PATH", "User")

# Check if Maya's bin path is already in the PATH
if (-not ($currentPath -contains $mayaBinPath)) {
    # Append the Maya bin path to the user's PATH
    $newPath = $currentPath + ";" + $mayaBinPath
    [System.Environment]::SetEnvironmentVariable("PATH", $newPath, "User")
}
