import json
import numpy as np
import pyvista as pv
from vtk import VTK_TETRA, VTK_TRIANGLE, VTK_VERTEX
from tet_visualization_with_points import visualize_tet_mesh
import sys
def test_app_1():
    # points = [
    #     (0, np.array([0.25, 0.25, 0.25, 0.25])),  # 在第一个四面体中的中心点
    #     (1, np.array([0.1, 0.2, 0.3, 0.4]))       # 在第二个四面体中的一个点
    # ]

    # Load the data from the JSON file
    file_path = 'operation_log_2024-11-11_18-01-04/operation_log_1.json'
    with open(file_path, 'r') as f:
        data = json.load(f)

    # Extract data for before and after
    T_before = np.array(data['T_before']['values'])
    V_before = np.array(data['V_before']['values'])
    T_after = np.array(data['T_after']['values'])
    V_after = np.array(data['V_after']['values'])
    F_bd_before = np.array(data['F_bd_before']['values'])
    F_bd_after = np.array(data['F_bd_after']['values'])

    # Create a plotter with 1 row and 2 columns for subplots
    plotter = pv.Plotter(shape=(1, 2))

    # Add "before" mesh to the first subplot
    visualize_tet_mesh(T_before, V_before, title="Before", plotter=plotter, subplot_index=(0, 0), show_edges_only=True)

    # Add "after" mesh to the second subplot
    visualize_tet_mesh(T_after, V_after, title="After", plotter=plotter, subplot_index=(0, 1), show_edges_only=True)

    # Add "before" mesh to the first subplot
    # visualize_tet_mesh(F_bd_before, V_before, title="Before", plotter=plotter, subplot_index=(1, 0), show_edges_only=False)

    # Add "after" mesh to the second subplot
    # visualize_tet_mesh(F_bd_after, V_after, title="After", plotter=plotter, subplot_index=(1, 1), show_edges_only=False)
    # Link the two subplots so they share the same view
    plotter.link_views()
    # Show both subplots in one window
    plotter.show()


def test_app_2():
    mesh = pv.read("sphere_coarse_tets.vtu")
    cell_data = mesh.cells.reshape((-1, 5))  # 每个单元格格式：[cell_size, v1, v2, v3, v4]
    tet_indices = cell_data[cell_data[:, 0] == 4][:, 1:]  # 4 表示四面体的顶点数量
    points = [(tet_id * 100, np.array([0.25, 0.25, 0.25, 0.25])) for tet_id in range(tet_indices.shape[0] // 100)]
    # visualize_tet_mesh(tet_indices, mesh.points, points=points, title="Sphere Mesh", show_edges_only=False, show_mesh = False)

    # get position of these points
    point_positions = []
    for tet_id, bary_coord in points:
        vertex_indices = tet_indices[tet_id]
        vertices = mesh.points[vertex_indices]
        point_coord = np.dot(bary_coord, vertices)
        point_positions.append(point_coord)


    # 创建 UnstructuredGrid 对象并设置点
    points_array = np.vstack(point_positions)
    n_points = points_array.shape[0]
    cells = np.hstack([[1, i] for i in range(n_points)])  # 每个点作为一个独立的顶点单元
    cells = np.array(cells, dtype=np.int64)

    # 定义网格
    grid = pv.UnstructuredGrid(cells, np.full(n_points, VTK_VERTEX), points_array)

    # 将点数据保存为 .vtu 文件
    grid.save("points.vtu")

    print("Points saved to points.vtu")

def position_to_vtu(filename, output_filename):
    # read position array from csv file
    position_array = np.loadtxt(filename, delimiter=',')
    n_points = position_array.shape[0]
    cells = np.hstack([[1, i] for i in range(n_points)])  # 每个点作为一个独立的顶点单元
    cells = np.array(cells, dtype=np.int64)
    grid = pv.UnstructuredGrid(cells, np.full(n_points, VTK_VERTEX), position_array)
    grid.save(output_filename)
    print(f"Points saved to {output_filename}")


if __name__ == "__main__":
    # get filename and output filename from command line
    filename = sys.argv[1]
    output_filename = sys.argv[2]
    position_to_vtu(filename, output_filename)
