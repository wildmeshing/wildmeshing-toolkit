import numpy as np
import cv2
import os
import nrrd

file_name = 'D:/Desktop/Workfiles/Code/marchingwindow/data/egg/kinder_healed.nrrd'
dst_folder_name = 'D:/Desktop/Workfiles/Code/marchingwindow/data/egg/outputfile'

# read nrrd and output raw image files
def read_nrrd(file_path):
    data, header = nrrd.read(file_path)
    return data

data = read_nrrd(file_name)

os.makedirs(dst_folder_name, exist_ok=True)

dim_z = data.shape[2]

for file_index in range(0, dim_z):
    array = data[:,:,file_index].astype(int)

    with open(f"{dst_folder_name}/{file_index:04d}.svdata", "wb") as f_out:
        sb = (int(array.shape[0])).to_bytes(4, byteorder='little')
        f_out.write(sb)
        sb = (int(array.shape[1])).to_bytes(4, byteorder='little')
        f_out.write(sb)
        sb = (4).to_bytes(4, byteorder='little')
        f_out.write(sb)

        for i in range(array.shape[0]):
            for j in range(array.shape[1]):
                sb = int(array[i, j])
                sb = sb.to_bytes(4, byteorder='little')
                f_out.write(sb)