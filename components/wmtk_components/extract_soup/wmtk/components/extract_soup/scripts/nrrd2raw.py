import numpy as np
import nrrd

input_filename = 'D:/Desktop/script/voxelman_innerorgans_healed.nrrd'
output_filename = 'D:/Desktop/script/torso.raw'

def read_nrrd_data(filename):
    data, header = nrrd.read(filename)
    return data

def write_array_data(filename, data):
    dim1, dim2, dim3 = data.shape

    with open(filename, 'wb') as f:
        f.write(dim1.to_bytes(4, byteorder='little'))
        f.write(dim2.to_bytes(4, byteorder='little'))
        f.write(dim3.to_bytes(4, byteorder='little'))

        for i in range(data.shape[0]):
            for j in range(data.shape[1]):
                for k in range(data.shape[2]):
                    v = int(data[i][j][k])
                    f.write(v.to_bytes(4, byteorder='little'))

def add_padding(array):
    dim1, dim2, dim3 = array.shape

    padded_array = np.zeros((dim1 + 2, dim2 + 2, dim3 + 2), dtype=array.dtype)

    padded_array[1:-1, 1:-1, 1:-1] = array

    return padded_array

nrrd_data = read_nrrd_data(input_filename)

write_array_data(output_filename, add_padding(nrrd_data))
