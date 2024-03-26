import vtk
import sys

def write_vtu_to_obj(vtu_file_path, obj_file_path):
    # 读取VTU文件
    reader = vtk.vtkXMLUnstructuredGridReader()
    reader.SetFileName(vtu_file_path)
    reader.Update()
    unstructured_grid = reader.GetOutput()
    
    # 打开OBJ文件以写入
    with open(obj_file_path, 'w') as obj_file:
        # 写入顶点
        for i in range(unstructured_grid.GetNumberOfPoints()):
            point = unstructured_grid.GetPoint(i)
            obj_file.write(f"v {point[0]} {point[1]} {point[2]}\n")
        
        # 写入面
        for i in range(unstructured_grid.GetNumberOfCells()):
            cell = unstructured_grid.GetCell(i)
            if cell.GetCellType() == vtk.VTK_TRIANGLE:  # 仅处理三角形单元
                ids = cell.GetPointIds()
                obj_file.write(f"f {ids.GetId(0) + 1} {ids.GetId(1) + 1} {ids.GetId(2) + 1}\n")
            # 可以添加对其他单元类型的处理

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python script.py <input.vtu> <output.obj>")
        sys.exit(1)
    
    vtu_file_path = sys.argv[1]
    obj_file_path = sys.argv[2]
    write_vtu_to_obj(vtu_file_path, obj_file_path)
