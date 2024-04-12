import numpy as np
import nrrd

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
                    f.write(data[i][j][k].to_bytes(4, byteorder='little'))

nrrd_data = read_nrrd_data('D:/Desktop/script/voxelman_head_healed.nrrd')

write_array_data('D:/Desktop/script/head_data.raw', nrrd_data)
