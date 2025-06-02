import maya.cmds as cmds
import os

# Path to the plugin
PLUGIN_PATH = r"E:\dev\RBF\pointsData\BasicMLFakePhysic\build\Debug\MLVertexBaseTransform.mll"
DEFAULT_OUTPUT_PATH = r"E:\dev\RBF\pointsData\BasicMLFakePhysic\training_data.csv"

def load_plugin():
    """Load the MLVertexBaseTransform plugin if not already loaded."""
    if not cmds.pluginInfo("MLVertexBaseTransform", query=True, loaded=True):
        try:
            cmds.loadPlugin(PLUGIN_PATH)
            print(f"Plugin loaded: {PLUGIN_PATH}")
        except Exception as e:
            cmds.error(f"Failed to load plugin: {e}")
    else:
        print("Plugin is already loaded.")

def unload_plugin():
    """Unload the MLVertexBaseTransform plugin if loaded."""
    if cmds.pluginInfo("MLVertexBaseTransform", query=True, loaded=True):
        try:
            cmds.unloadPlugin("MLVertexBaseTransform")
            print("Plugin unloaded.")
        except Exception as e:
            cmds.error(f"Failed to unload plugin: {e}")
    else:
        print("Plugin is not loaded.")

def reload_plugin():
    """Reload the MLVertexBaseTransform plugin by unloading and loading."""
    unload_plugin()
    load_plugin()

def test_deformer():
    """Create a test scene and apply the deformer to verify functionality."""
    cube = cmds.polyCube(name="pCube1")[0]
    locator = cmds.spaceLocator(name="locator1")[0]
    cmds.setKeyframe(locator, attribute="translateX", time=1, value=0)
    cmds.setKeyframe(locator, attribute="translateX", time=199, value=10)
    deformer = cmds.deformer(cube, type="mlDeformer", name="mlDeformer1")[0]
    cmds.connectAttr(f"{locator}.matrix", f"{deformer}.transformMatrix")
    print(f"Test scene created: Move {locator} along X to see deformation on {cube}.")
    return cube, locator, deformer

def select_transform():
    """Select and store the transform node."""
    selected = cmds.ls(selection=True, type="transform")
    if selected:
        cmds.text("transformLabel", edit=True, label=f"Transform: {selected[0]}")
        return selected[0]
    cmds.warning("No transform selected!")
    return None

def select_mesh():
    """Select and store the mesh shape node."""
    selected = cmds.ls(selection=True, shapes=True, type="mesh")
    if selected:
        cmds.text("meshLabel", edit=True, label=f"Mesh: {selected[0]}")
        return selected[0]
    cmds.warning("No mesh shape selected!")
    return None

def browse_output_path():
    """Open a file dialog to select the output file path."""
    file_path = cmds.fileDialog2(fileFilter="CSV Files (*.csv)", caption="Select Output File", fileMode=0, startingDirectory=DEFAULT_OUTPUT_PATH)
    if file_path:
        cmds.textField("outputPathField", edit=True, text=file_path[0])
        return file_path[0]
    return None

def collect_data():
    """Collect transform and vertex data for the specified frame range."""
    transform = cmds.text("transformLabel", query=True, label=True).split(": ")[1]
    mesh = cmds.text("meshLabel", query=True, label=True).split(": ")[1]
    start_frame = cmds.intField("startFrameField", query=True, value=True)
    end_frame = cmds.intField("endFrameField", query=True, value=True)
    output_path = cmds.textField("outputPathField", query=True, text=True)

    if transform == "None" or mesh == "None":
        cmds.error("Select both a transform and a mesh!")
        return
    if end_frame < start_frame:
        cmds.error("End frame must be greater than or equal to start frame!")
        return
    if not output_path:
        cmds.error("Specify an output file path!")
        return

    try:
        cmds.mlDeformerCollect(m=mesh, t=transform, sf=start_frame, ef=end_frame, o=output_path)
        if os.path.exists(output_path):
            print(f"Data collected for frames {start_frame} to {end_frame} and saved to {output_path}.")
        else:
            cmds.error(f"Data collection completed, but {output_path} was not found!")
    except Exception as e:
        cmds.error(f"Data collection failed: {e}")

def create_ui():
    """Create a UI to manage the plugin and collect data."""
    window_name = "MLDeformerManager"
    if cmds.window(window_name, exists=True):
        cmds.deleteUI(window_name)

    window = cmds.window(window_name, title="ML Deformer Manager", widthHeight=(400, 400))
    cmds.columnLayout(adjustableColumn=True)
    
    cmds.button(label="Load Plugin", command="load_plugin()")
    cmds.button(label="Reload Plugin", command="reload_plugin()")
    cmds.button(label="Unload Plugin", command="unload_plugin()")
    cmds.button(label="Test Deformer", command="test_deformer()")
    cmds.separator(height=10)
    cmds.text(label="Data Collection")
    cmds.button(label="Select Transform", command="select_transform()")
    cmds.text("transformLabel", label="Transform: None")
    cmds.button(label="Select Mesh", command="select_mesh()")
    cmds.text("meshLabel", label="Mesh: None")
    cmds.intField("startFrameField", value=1, minValue=1, maxValue=1000)
    cmds.intField("endFrameField", value=199, minValue=1, maxValue=1000)
    cmds.text(label="Output File Path:")
    cmds.textField("outputPathField", text=DEFAULT_OUTPUT_PATH, width=300)
    cmds.button(label="Browse...", command="browse_output_path()")
    cmds.button(label="Collect Data", command="collect_data()")
    
    cmds.showWindow(window)

if __name__ == "__main__":
    create_ui()