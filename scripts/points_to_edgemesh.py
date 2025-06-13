# Start Generation Here
import numpy as np
import pyvista as pv
from vtk import VTK_LINE
import sys

def generate_edge_mesh(file1, file2, output_filename):
    # 读取file1中的点集V1
    V1 = np.loadtxt(file1, delimiter=',')
    
    # 读取file2中的点集V2
    V2 = np.loadtxt(file2, delimiter=',')
    
    # 确保V1和V2的大小相同
    if V1.shape[0] != V2.shape[0]:
        print("Error: The point sets V1 and V2 must have the same number of points.")
        return
    
    # 创建点集和边集
    points = np.vstack((V1, V2))
    n_points = V1.shape[0]
    edges = np.hstack([[2, i, i + n_points] for i in range(n_points)])
    edges = np.array(edges, dtype=np.int64)
    
    # 创建UnstructuredGrid对象并设置点和边
    grid = pv.UnstructuredGrid(edges, np.full(n_points, VTK_LINE), points)
    
    # 将边数据保存为.vtu文件
    grid.save(output_filename)
    print(f"Edge mesh saved to {output_filename}")

if __name__ == "__main__":
    # 从命令行获取文件名和输出文件名
    file1 = sys.argv[1]
    file2 = sys.argv[2]
    output_filename = sys.argv[3]
    generate_edge_mesh(file1, file2, output_filename)
# End Generation Here

