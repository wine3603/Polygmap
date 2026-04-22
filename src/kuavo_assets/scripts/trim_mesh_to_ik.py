import xml.etree.ElementTree as ET

def process_urdf(input_file, output_file, version):
    # 解析XML文件
    tree = ET.parse(input_file)
    root = tree.getroot()
    
    # 需要保留的link和joint前缀
    keep_prefixes = ['base', 'zarm']
    
    # 找出所有需要删除的elements
    to_remove = []
    for elem in root:
        if elem.tag in ['link', 'joint']:
            name = elem.get('name', '')
            if not any(name.startswith(prefix) for prefix in keep_prefixes):
                to_remove.append(elem)
    
    # 删除不需要的elements
    for elem in to_remove:
        root.remove(elem)
    
    # 修改mesh路径
    for mesh in root.findall(".//mesh"):
        filename = mesh.get('filename', '')
        if 'package://kuavo_assets/models/biped_s'+str(version) in filename:
            new_filename = filename.replace('package://kuavo_assets/models/biped_s'+str(version), '../..')
            mesh.set('filename', new_filename)
    
    # 写入新文件
    tree.write(output_file, encoding='utf-8', xml_declaration=True)

# 使用脚本
path = '../models/'

versions = [40, 41, 42, 43, 45, 46]
for version in versions:
    robot_version = 'biped_s' + str(version)
    input_file = path + robot_version + '/urdf/' + robot_version + '.urdf'
    output_file = path + robot_version + '/urdf/drake/biped_v3_arm.urdf'
    process_urdf(input_file, output_file, version)
