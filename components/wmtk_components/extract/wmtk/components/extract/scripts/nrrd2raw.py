import numpy as np
import nrrd

def read_nrrd_data(filename):
    data, header = nrrd.read(filename)
    return data

def write_array_data(filename, data):
    # 获取数组的形状信息
    dim1, dim2, dim3 = data.shape

    # 将形状信息和数据以字节形式写入文件
    with open(filename, 'wb') as f:
        # 将三个维度的大小写入文件
        f.write(dim1.to_bytes(4, byteorder='little'))
        f.write(dim2.to_bytes(4, byteorder='little'))
        f.write(dim3.to_bytes(4, byteorder='little'))

        # 将数组数据以字节形式写入文件
        for i in range(data.shape[0]):
            for j in range(data.shape[1]):
                for k in range(data.shape[2]):
                    f.write(data[i][j][k].to_bytes(4, byteorder='little'))

# 读取NRRD文件的数组数据
nrrd_data = read_nrrd_data('D:/Desktop/script/voxelman_head_healed.nrrd')

# 将数组数据写入新文件
write_array_data('D:/Desktop/script/head_data.raw', nrrd_data)
