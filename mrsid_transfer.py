import os
from osgeo import gdal

def convert_mrsid_to_tiff_batch(input_dir, output_dir):
    
    files = [f for f in os.listdir(input_dir) if f.endswith('.sid')]
    indexx = files.index("HAB2.sid")
    files = files[indexx+1:]
    
    if not files:
        print("没有找到任何.sid文件")
        return
    
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    for file in files:
        input_sid = os.path.join(input_dir, file)
        output_tiff = os.path.join(output_dir, os.path.splitext(file)[0] + '.tif')
        
        dataset = gdal.Open(input_sid)
        if not dataset:
            print(f"无法打开文件: {input_sid}")
            continue
        
        gdal.Translate(output_tiff, dataset)
        print(f"转换成功: {input_sid} -> {output_tiff}")


input_directory = "Toronto_Picture2011-2021\\2012"  # 替换为你的MrSID文件目录
output_directory = "Trans_output\\2012" # 替换为你想要的输出TIFF文件目录

convert_mrsid_to_tiff_batch(input_directory, output_directory)

