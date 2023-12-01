import meshio as mio
import igl
import sys

class MshConverter:

    def __init__(self, output_pattern = "{}_{}.msh", binary=True):
        self.output_pattern = output_pattern
        self.binary = binary

    def convert(self, input_filename, output_prefix=None):

        filename_split = input_filename.split('.')

        if output_prefix is None:
            output_prefix = ".".join(filename_split[:-1])

        ext = filename_split[-1]


        topologies = dict()
        if ext in {"ply","stl"}:
            m = mio.read(input_filename)
            topologies["pos"] = m
        elif ext == "obj":
            topologies = self.get_topologies_obj(input_filename)
        elif ext == "mesh":
            topologies = self.get_topologies_mesh(input_filename)

        for typ, m in topologies.items():
            name = self.output_pattern.format(output_prefix,typ)
            m.write(name, file_format="gmsh",binary=self.binary)

    def get_topologies_obj(self, input_filename):
        v,tc,n,f,ftc,fn = igl.read_obj(input_filename)

        topologies = dict()
        igl_topologies = [
                (v,f,"pos"),
                (tc,ftc,"tex"),
                (n,fn,"normals")
                ]
        for V,F,typ in igl_topologies:
            if len(V) == 0:
                continue
            topologies[typ] = mio.Mesh(V, [("triangle", F)])
        return topologies


    def get_topologies_mesh(self, input_filename):
        m = mio.read(input_filename)
        T = []
        topologies = dict()
        for c in m.cells:
            if c.type == "tetra":
                T = c.data
                break
        if len(T) == 0:
            print(f"{file} doesnt contain any tets, skipping")
            return topologies
        m = mio.Mesh(m.points, [("tetra", T)])
        topologies["pos"] = m
        return topologies




def __main__():
    converter = MshConverter()

    args = sys.argv[1:]
    converter.convert(*args)



if __name__ == "__main__":
    __main__()
