import os

from ament_index_python.packages import get_package_share_directory

desc_dir = get_package_share_directory('iir_base')

path = os.path.join(desc_dir, 'urdf', 'iirbot.urdf.xacro')

print(path)
