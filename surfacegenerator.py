import numpy as np
import trimesh
from pathlib import Path

def generate_mesh_from_function(func, x_range, y_range, resolution=50):
    """Generates a mesh from a mathematical function z = f(x, y)."""
    x = np.linspace(*x_range, resolution)
    y = np.linspace(*y_range, resolution)
    x_grid, y_grid = np.meshgrid(x, y)
    z_grid = func(x_grid, y_grid)

    vertices = np.column_stack((x_grid.flatten(), y_grid.flatten(), z_grid.flatten()))
    faces = []

    # Create faces for the mesh grid
    for i in range(resolution - 1):
        for j in range(resolution - 1):
            idx = i * resolution + j
            faces.append([idx, idx + 1, idx + resolution])
            faces.append([idx + 1, idx + resolution + 1, idx + resolution])

    return trimesh.Trimesh(vertices=vertices, faces=faces)

def export_to_stl(mesh, filename):
    """Exports the mesh to an STL file."""
    mesh.export(filename)
    print(f"Mesh exported to {filename}")

def generate_urdf(mesh_filename, urdf_filename, mesh_scale=(1, 1, 1)):
    """Generates a URDF referencing the mesh."""
    urdf_content = f"""<?xml version="1.0" ?>
<robot name="custom_plane">
  <link name="plane_link">
    <visual>
      <geometry>
        <mesh filename="{mesh_filename}" scale="{mesh_scale[0]} {mesh_scale[1]} {mesh_scale[2]}"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <mesh filename="{mesh_filename}" scale="{mesh_scale[0]} {mesh_scale[1]} {mesh_scale[2]}"/>
      </geometry>
    </collision>
  </link>
</robot>
"""
    with open(urdf_filename, "w") as file:
        file.write(urdf_content)
    print(f"URDF exported to {urdf_filename}")

# Example function: wavy surface
def example_function(x, y):
    return 0.25 * np.sin(0.5 * np.pi * x) * np.cos(0.5 * np.pi * y)

if __name__ == "__main__":
    # Configuration
    x_range = (-20, 20) #(-10, 50)
    y_range = (-10, 50) #(-20, 20)
    resolution = 500
    mesh_dir = Path("urdf")
    mesh_dir.mkdir(exist_ok=True)

    stl_filename = mesh_dir / "plane_mesh_flat_bossed.stl"
    urdf_filename = mesh_dir / "plane_flat_bossed.urdf"

    # Generate mesh and export
    mesh = generate_mesh_from_function(example_function, x_range, y_range, resolution)
    export_to_stl(mesh, stl_filename)

    # Generate URDF
    generate_urdf(mesh_filename=stl_filename.name, urdf_filename=urdf_filename)
