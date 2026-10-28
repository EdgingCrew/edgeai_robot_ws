import os

from ament_index_python import get_package_share_directory
import yaml
    

def load_robots():
    path = os.path.join(get_package_share_directory('edgeai_robot_description'), 'config', 'robots.yaml')
    with open(path) as f:
        return yaml.safe_load(f)['robots']