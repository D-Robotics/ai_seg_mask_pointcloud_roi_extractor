# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import sys
import yaml
from collections import OrderedDict
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python import get_package_share_directory


current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)
sys.path.append(current_dir)

NODE_NAME = "seg_mask"

def _to_plain_dict(obj):
    """Recursively convert OrderedDict to plain dict for safe YAML serialization."""
    if isinstance(obj, OrderedDict):
        return {k: _to_plain_dict(v) for k, v in obj.items()}
    if isinstance(obj, dict):
        return {k: _to_plain_dict(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [_to_plain_dict(v) for v in obj]
    return obj

class AutoLaunchArguments:
    def __init__(self, descriptions_dir, visual_params_path):
        """
        Initialize the AutoLaunchArguments class.

        :@param descriptions_dir: Directory containing YAML files with parameter descriptions.
        :@param visual_params_path: Path to the YAML file with visual parameters.
        """
        self.descriptions_dir = descriptions_dir
        self.visual_params_path = visual_params_path
        self.declare_para = []
        self.config_para = []

        self.params_dict = OrderedDict()

        self.params_dict[NODE_NAME] = {'ros__parameters': {}}
        self._load_all_parameters()
        
    
    def _load_all_parameters(self):
        """
        Load all parameters from YAML files in the descriptions directory.
        """
        if not os.path.exists(self.descriptions_dir):
            raise FileNotFoundError(f"Directory {self.descriptions_dir} does not exist, please check the path.")
        
        yaml_files = []
        for filename in os.listdir(self.descriptions_dir):
            if filename.endswith('.yaml'):
                yaml_files.append(os.path.join(self.descriptions_dir, filename))

        if len(yaml_files) == 0:
            raise FileNotFoundError(f"Directory {self.descriptions_dir} does not contain any yaml files, please check the path.")
        
        
        self._process_yaml_file(yaml_files)
    
    def _process_yaml_file(self, yaml_files):
        """
        Process parameters from a single YAML file.
        """
        for file_path in yaml_files:
            try:
                with open(file_path, 'r', encoding='utf-8') as file:
                    yaml_data = yaml.safe_load(file)
                
                if not isinstance(yaml_data, dict) or NODE_NAME not in yaml_data:
                    print(f"Warning: Invalid YAML structure in {file_path}")
                    return
                
                if 'ros__parameters' not in yaml_data[NODE_NAME]:
                    print(f"Warning: No ros__parameters found in {file_path}")
                    return
                
                parameters = yaml_data[NODE_NAME]['ros__parameters']
                
                for param_name, param_data in parameters.items():
                    if param_name in ['description', 'description_en', 'type', 'default_value', 'range', 'unit']:
                        continue
                    
                    if not isinstance(param_data, dict):
                        continue
                    
                    default_value = param_data.get('default_value', '0')
                    description = param_data.get('description_en', f'Parameter {param_name}')
                    
                    # DeclareLaunchArgument
                    self.declare_para.append(
                        DeclareLaunchArgument(
                            param_name,
                            default_value = default_value,
                            description = description,
                        )
                    )
                    
                    # LaunchConfiguration
                    self.config_para.append({
                        param_name: LaunchConfiguration(param_name)
                    })

                    # Add to params_dict and write to params.yaml
                    if isinstance(param_data, dict) and 'default_value' in param_data:
                        default_value = param_data['default_value']
                        param_type = param_data.get('type', '').lower()
                        
                        if param_type == 'bool':
                            # yaml.safe_load may parse unquoted true/false as Python bool
                            if isinstance(default_value, bool):
                                value = default_value
                            else:
                                value = str(default_value).lower() == 'true'
                        elif param_type == 'int':
                            value = int(default_value)
                        elif param_type == 'double' or param_type == 'float':
                            value = float(default_value)
                        elif param_type == 'string':
                            value = str(default_value)
                        else:
                            raise ValueError(f"Unknown type {param_type} for parameter {param_name}")
                        
                        self.params_dict[NODE_NAME]['ros__parameters'][param_name] = value
                
            except Exception as e:
                print(f"Error processing file {file_path}: {e}")
            
        os.makedirs(os.path.dirname(self.visual_params_path), exist_ok=True)
        
        with open(self.visual_params_path, 'w', encoding='utf-8') as file:
            # Convert OrderedDict to plain dict to avoid Python-specific YAML tags
            # (e.g. !!python/object/apply:collections.OrderedDict) that ROS 2 can't parse.
            plain_dict = _to_plain_dict(self.params_dict)
            yaml.safe_dump(plain_dict, file, default_flow_style=False, allow_unicode=True, sort_keys=False)
        
        print(f"Successfully generated params.yaml at {self.visual_params_path}")
    
    def get_declare_arguments(self):
        """
        Get all DeclareLaunchArgument actions.
        """
        return self.declare_para
    
    def get_config_parameters(self):
        """
        Get all LaunchConfiguration actions.
        """
        return self.config_para
    
    def find_parameter(self, param_name):
        """
        Find a parameter by name.
        """
        for para_dict in self.config_para:
            if param_name in para_dict:
                return para_dict[param_name]
        return None

