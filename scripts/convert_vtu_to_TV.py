import numpy as np
import pyvista as pv
import argparse



def main(input_file):
    # 读取 .vtu 文件
    mesh = pv.read(input_file)
    # 获取顶点坐标矩阵 V (n_points, 3)
    V = mesh.points  # 顶点坐标

    # 获取四面体单元顶点索引矩阵 T
    # 筛选出四面体单元
    tetra_cells = mesh.cells.reshape((-1, 5))  # 每行格式：[cell_type, v1, v2, v3, v4]
    T = tetra_cells[:, 1:]  # 只取顶点索引部分
    # 保存 V 和 T 到 CSV 文件
    np.savetxt("V_matrix.csv", V, delimiter=",", header="x,y,z", comments='')
    np.savetxt("T_matrix.csv", T, delimiter=",", header="v1,v2,v3,v4", fmt='%d', comments='')

    print("Saved V and T matrices to V_matrix.csv and T_matrix.csv.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Extract V and T matrices from a VTU file.")
    parser.add_argument("input_file", type=str, help="Path to the input .vtu file")
    args = parser.parse_args()

    main(args.input_file)