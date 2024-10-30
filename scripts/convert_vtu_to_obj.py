import meshio as mio 
import sys

def __main__():
    input_path,output_path = sys.argv[1:]
    
    mesh = mio.read(input_path)

    if "triangle" in mesh.cells_dict:
        mio.write(output_path + ".obj", mesh)
    elif "line" in mesh.cells_dict:
        file = open(output_path + ".obj", "w")
        for p in mesh.points:
            file.write(f'v {p[0]} {p[1]} {p[2]}\n')
        for l in mesh.cells_dict["line"]:
            file.write(f'l {l[0] + 1} {l[1] + 1}\n')
        file.close()
    else:
        print("wrong input data format")


if __name__ == "__main__":
    __main__()