import os
from qgis.core import QgsApplication, QgsRasterLayer, QgsProcessingFeedback
from qgis import processing

# 启动QGIS应用程序（对于独立Python脚本，QGIS环境变量必须正确配置）
QgsApplication.setPrefixPath("/Applications/QGIS.app/Contents/MacOS", True)
qgs = QgsApplication([], False)
qgs.initQgis()

# 输入目录和输出目录
input_dir = "/Users/clairemeng/Desktop/mrsid_to_tiff/Toronto_Picture2011-2021/2012"  # 替换为你的MrSID文件所在目录
output_dir = "/Users/clairemeng/Desktop/mrsid_to_tiff/Trans_output/2012"  # 替换为你想要的输出TIFF文件目录

# 创建输出目录（如果不存在）
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# 批量转换函数
def batch_convert_mrsid_to_tiff(input_dir, output_dir):
    # 获取目录中所有的.sid文件
    files = [f for f in os.listdir(input_dir) if f.endswith('.sid')]
    
    # 遍历每个文件并进行转换
    for file in files:
        input_path = os.path.join(input_dir, file)
        output_path = os.path.join(output_dir, os.path.splitext(file)[0] + '.tif')
        
        # 创建QGIS栅格图层
        raster_layer = QgsRasterLayer(input_path, file)
        
        if not raster_layer.isValid():
            print(f"无法加载文件: {input_path}")
            continue
        
        # 使用QGIS的gdal:translate工具进行格式转换
        params = {
            'INPUT': raster_layer,
            'TARGET_CRS': None,
            'NODATA': None,
            'COPY_SUBDATASETS': False,
            'OPTIONS': '',
            'EXTRA': '',
            'OUTPUT': output_path
        }
        
        processing.run("gdal:translate", params, QgsProcessingFeedback())
        print(f"成功转换: {input_path} -> {output_path}")

# 调用批量转换函数
batch_convert_mrsid_to_tiff(input_dir, output_dir)

# 关闭QGIS应用程序
qgs.exitQgis()
