import maya.api.OpenMaya as om

def ray_triangle_intersection(ray_origin, ray_vector, tri_vertices):
    EPSILON = 1e-6
    v0, v1, v2 = tri_vertices
    edge1 = v1 - v0
    edge2 = v2 - v0
    h = ray_vector ^ edge2  # Cross product
    a = edge1 * h  # Dot product

    if -EPSILON < a < EPSILON:
        return False  # Ray is parallel to the triangle

    f = 1.0 / a
    s = ray_origin - v0
    u = f * (s * h)  # Dot product
    if u < 0.0 or u > 1.0:
        return False  # Ray does not intersect

    q = s ^ edge1  # Cross product
    v = f * (ray_vector * q)  # Dot product
    if v < 0.0 or u + v > 1.0:
        return False  # Ray does not intersect

    t = f * (edge2 * q)  # Dot product
    if t > EPSILON:  # Ray intersects
        return True
    else:
        return False  # No intersection

def check_triangle_intersection(tri1_vertices, tri2_vertices):
    # Check if any edge of tri1 intersects tri2
    for i in range(3):
        edge_start = tri1_vertices[i]
        edge_end = tri1_vertices[(i + 1) % 3]
        ray_vector = edge_end - edge_start
        if ray_triangle_intersection(edge_start, ray_vector, tri2_vertices):
            return True

    # Check if any edge of tri2 intersects tri1
    for i in range(3):
        edge_start = tri2_vertices[i]
        edge_end = tri2_vertices[(i + 1) % 3]
        ray_vector = edge_end - edge_start
        if ray_triangle_intersection(edge_start, ray_vector, tri1_vertices):
            return True

    return False

# Example usage with Maya API
def get_triangle_vertices_from_mesh(mesh_path, triangle_index):
    # Get the selected mesh
    selection_list = om.MSelectionList()
    selection_list.add(mesh_path)
    dag_path = selection_list.getDagPath(0)
    
    # Get the MFnMesh object
    mesh_fn = om.MFnMesh(dag_path)
    
    # Get the vertices of the specified triangle
    triangle_vertices = mesh_fn.getPolygonVertices(triangle_index)
    points = mesh_fn.getPoints(om.MSpace.kWorld)
    
    return [points[triangle_vertices[i]] for i in range(3)]

# Example: Check if two triangles intersect
tri1 = get_triangle_vertices_from_mesh("pPlane2", 0)  # Triangle from mesh 1
tri2 = get_triangle_vertices_from_mesh("pPlane1", 0)  # Triangle from mesh 2

if check_triangle_intersection(tri1, tri2):
    print("Triangles intersect!")
else:
    print("Triangles do not intersect.")
