from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.substitutions import FindPackageShare
import yaml

def extract_camera_params(calib_data):
    """Extrae y reformatea los parámetros de calibración de la estructura YAML"""
    params = {}
    
    # Para matriz de cámara
    if 'camera_matrix' in calib_data:
        cam_matrix = calib_data['camera_matrix']
        if 'data' in cam_matrix and len(cam_matrix['data']) == 9:
            params['camera_matrix'] = cam_matrix['data']
    
    # Para coeficientes de distorsión
    if 'distortion_coefficients' in calib_data:
        dist_coeffs = calib_data['distortion_coefficients']
        if 'data' in dist_coeffs and len(dist_coeffs['data']) >= 5:
            # Tomar solo los primeros 5 coeficientes
            params['distortion_coefficients'] = dist_coeffs['data'][:5]
    
    # Dimensiones de imagen
    params['image_width'] = calib_data.get('image_width', 320)
    params['image_height'] = calib_data.get('image_height', 240)
    
    return params

def load_parameters(context, *args, **kwargs):
    # Obtener la ruta del archivo YAML de parámetros
    params_file = PathJoinSubstitution([
        FindPackageShare('blackpearls_nav2_puzzlebot'),
        'config',
        LaunchConfiguration('camera_params_file').perform(context)
    ]).perform(context)
    
    # Cargar parámetros desde el archivo YAML
    with open(params_file, 'r') as f:
        calib_data = yaml.safe_load(f)
    
    # Extraer parámetros de visión
    vision_params = extract_camera_params(calib_data)
    vision_params['marker_length'] = float(LaunchConfiguration('marker_length').perform(context))
    
    # Parámetros para el nodo de cámara
    camera_params = {
        'image_width': vision_params['image_width'],
        'image_height': vision_params['image_height'],
        'camera_topic': LaunchConfiguration('camera_topic').perform(context)
    }
    
    return [
        Node(
            package='blackpearls_nav2_puzzlebot',
            executable='vision',
            name='vision_node',
            parameters=[vision_params],
            output='screen'
        ),
        Node(
            package='blackpearls_nav2_puzzlebot',
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
            default_value='camera_calibration.yaml',
            description='Nombre del archivo YAML con parámetros de cámara'
        ),
        
        # Parámetro para el tamaño del marcador ArUco
        DeclareLaunchArgument(
            'marker_length',
            default_value='0.14',
            description='Tamaño del marcador ArUco en metros'
        ),
        
        # Parámetro para el topic de la cámara
        DeclareLaunchArgument(
            'camera_topic',
            default_value='camera',
            description='Nombre del topic para la imagen de la cámara'
        ),
        
        # Función para cargar parámetros y crear nodos
        OpaqueFunction(function=load_parameters)
    ])