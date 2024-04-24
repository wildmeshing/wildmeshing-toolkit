import meshio as mio
import igl
import sys



class Converter:

    def __init__(self, use_single_pattern = True, single_output_pattern = "{prefix}.{extension}", multi_output_pattern = "{prefix}_{type}.{extension}"):
        self.use_single_pattern = single_output_pattern
        self.single_output_pattern = single_output_pattern
        self.multi_output_pattern = multi_output_pattern

    def convert(self, input_filename, output_prefix):
        data = self.__prepare_output_data__(input_filename, output_prefix, "msh")
        for name, m in data:
            self.__write__(m,name)

    def __write__(self, mesh, path):
        raise NotImplementedError()


    # outputs a dict {"mesh": meshio.Mesh, "output_filename"}
    def __prepare_output_data__(self, input_filename, output_prefix, extension):

        topologies = self.__get_topologies__(input_filename)
        use_single_pattern = self.use_single_pattern and len(topologies) == 1
        if not use_single_pattern:
            print("Note! multiple topologies found! outputing files with _pos/_tex or whatnot!")

        output_data = []

        for typ, m in topologies.items():
            if use_single_pattern:
                name = self.single_output_pattern.format(prefix=output_prefix , extension= extension)
            else:
                name = self.multi_output_pattern.format(prefix=output_prefix ,type=typ,extension= extension)

            output_data.append((name,m))
        return output_data




    
    def __get_topologies__(self, input_filename):
        filename_split = input_filename.split('.')


        ext = filename_split[-1]


        topologies = dict()
        if ext in {"ply","stl"}:
            m = mio.read(input_filename)
            topologies["pos"] = m
        elif ext == "obj":
            topologies = self.__get_topologies_obj__(input_filename)
        elif ext == "mesh":
            topologies = self.__get_topologies_mesh__(input_filename)
        return topologies


    def __get_topologies_obj__(self, input_filename):
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


    def __get_topologies_mesh__(self, input_filename):
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


class MshConverter(Converter):

    # TODO: pass in extra args if desired
    def __init__(self, binary=True):
        super().__init__()
        self.binary = binary

    def __write__(self,m,name):
        m.write(name, file_format="gmsh",binary=self.binary)


def __main__():
    converter = MshConverter()

    input_path,output_path = sys.argv[1:]
    converter.convert(input_path,output_path)



if __name__ == "__main__":
    __main__()
