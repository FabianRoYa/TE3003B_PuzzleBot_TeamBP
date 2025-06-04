from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.substitutions import FindPackageShare
import yaml

# hay que empatar con el YAML de parámetros
def load_parameters(context, *args, **kwargs):
    # Obtener la ruta del archivo YAML de parámetros
    params_file = PathJoinSubstitution([
        FindPackageShare('puzzlebot_ros'),
        'config',
        LaunchConfiguration('camera_params_file').perform(context)
    ]).perform(context)
    
    # Cargar parámetros desde el archivo YAML
    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)
    
    # Separar parámetros por nodo
    vision_params = params.get('vision_node', {})
    camera_params = params.get('camera_node', {})
    
    return [
        Node(
            package='puzzlebot_ros',
            executable='vision',
            name='vision_node',
            parameters=[vision_params],
            output='screen'
        ),
        Node(
            package='puzzlebot_ros',
            executable='eye_opener',
            name='camera_node',
            parameters=[camera_params],
            output='screen'
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        # Argumento para especificar el archivo de parámetros
        DeclareLaunchArgument(
            'camera_params_file',
            default_value='camera_params.yaml',
            description='Nombre del archivo YAML con parámetros de cámara'
        ),
        
        # Función para cargar parámetros y crear nodos
        OpaqueFunction(function=load_parameters)
    ])