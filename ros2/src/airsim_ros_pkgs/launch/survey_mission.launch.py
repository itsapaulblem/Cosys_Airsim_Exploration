#!/usr/bin/env python3
"""
Survey Mission Launch File - Area Survey and Mapping Operations
Optimized for systematic area coverage, mapping, and data collection

Survey Types:
- Agricultural monitoring (crop analysis, irrigation assessment)
- Environmental surveying (terrain mapping, vegetation analysis)
- Infrastructure inspection (pipeline monitoring, facility assessment)
- Reconnaissance and surveillance operations

Features:
- Grid-based systematic coverage for maximum efficiency
- Configurable overlap for photogrammetry and mapping
- Multi-altitude surveying for different data resolution
- Data logging and geo-referencing capabilities

Ultra-Clean Architecture: /VehicleName with specialized survey parameters
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

def generate_launch_description():
    """Generate launch description for area survey missions"""
    
    # === CORE ARGUMENTS ===
    host_ip_arg = DeclareLaunchArgument(
        'host_ip',
        default_value='172.28.240.1',
        description='IP address of AirSim server'
    )
    
    host_port_arg = DeclareLaunchArgument(
        'host_port',
        default_value='41451',
        description='Port of AirSim server'
    )
    
    # === SURVEY TYPE CONFIGURATION ===
    survey_type_arg = DeclareLaunchArgument(
        'survey_type',
        default_value='grid_survey',
        choices=['grid_survey', 'agricultural_survey', 'infrastructure_survey', 'environmental_survey'],
        description='Type of survey mission to execute'
    )
    
    survey_pattern_arg = DeclareLaunchArgument(
        'survey_pattern',
        default_value='grid',
        choices=['grid', 'lawnmower', 'spiral'],
        description='Survey pattern optimized for systematic coverage'
    )
    
    survey_vehicle_arg = DeclareLaunchArgument(
        'survey_vehicle',
        default_value='Droan1',
        description='Primary survey vehicle'
    )
    
    # === SURVEY AREA DEFINITION ===
    survey_area_corners_arg = DeclareLaunchArgument(
        'survey_area_corners',
        default_value='0,0 200,0 200,150 0,150',
        description='Survey area corners in NED coordinates (space-separated x,y pairs)'
    )
    
    # Alternative: GPS-based survey area
    gps_survey_area_arg = DeclareLaunchArgument(
        'gps_survey_area',
        default_value='',
        description='GPS survey area (space-separated lat,lon pairs, overrides NED coordinates if provided)'
    )
    
    # === SURVEY PARAMETERS ===
    survey_altitude_arg = DeclareLaunchArgument(
        'survey_altitude',
        default_value='50.0',
        description='Survey altitude in meters (higher for broader coverage)'
    )
    
    survey_speed_arg = DeclareLaunchArgument(
        'survey_speed',
        default_value='8.0',
        description='Survey speed in m/s (optimized for data collection)'
    )
    
    line_spacing_arg = DeclareLaunchArgument(
        'line_spacing',
        default_value='25.0',
        description='Distance between survey lines in meters'
    )
    
    # === COVERAGE AND OVERLAP SETTINGS ===
    lateral_overlap_arg = DeclareLaunchArgument(
        'lateral_overlap',
        default_value='60.0',
        description='Lateral overlap percentage between flight lines (for photogrammetry)'
    )
    
    forward_overlap_arg = DeclareLaunchArgument(
        'forward_overlap',
        default_value='80.0',
        description='Forward overlap percentage along flight lines'
    )
    
    # === DATA COLLECTION SETTINGS ===
    photo_interval_arg = DeclareLaunchArgument(
        'photo_interval',
        default_value='2.0',
        description='Time interval between photos in seconds'
    )
    
    enable_mapping_arg = DeclareLaunchArgument(
        'enable_mapping',
        default_value='true',
        choices=['true', 'false'],
        description='Enable mapping and photogrammetry data collection'
    )
    
    enable_multispectral_arg = DeclareLaunchArgument(
        'enable_multispectral',
        default_value='false',
        choices=['true', 'false'],
        description='Enable multispectral imaging (if available)'
    )
    
    # === MULTI-ALTITUDE SURVEY ===
    enable_multi_altitude_arg = DeclareLaunchArgument(
        'enable_multi_altitude',
        default_value='false',
        choices=['true', 'false'],
        description='Enable multi-altitude survey for different resolutions'
    )
    
    altitude_levels_arg = DeclareLaunchArgument(
        'altitude_levels',
        default_value='30.0,50.0,80.0',
        description='Comma-separated altitude levels for multi-altitude survey'
    )
    
    # === SURVEY-SPECIFIC PARAMETERS ===
    
    # Agricultural survey parameters
    crop_type_arg = DeclareLaunchArgument(
        'crop_type',
        default_value='general',
        choices=['general', 'corn', 'wheat', 'soy', 'vineyard', 'orchard'],
        description='Crop type for agricultural surveys (affects flight parameters)'
    )
    
    growth_stage_arg = DeclareLaunchArgument(
        'growth_stage',
        default_value='mid_season',
        choices=['early', 'mid_season', 'mature', 'harvest'],
        description='Growth stage for agricultural analysis'
    )
    
    # Infrastructure survey parameters
    infrastructure_type_arg = DeclareLaunchArgument(
        'infrastructure_type',
        default_value='general',
        choices=['general', 'pipeline', 'powerline', 'road', 'facility'],
        description='Infrastructure type for specialized survey patterns'
    )
    
    inspection_detail_arg = DeclareLaunchArgument(
        'inspection_detail',
        default_value='standard',
        choices=['overview', 'standard', 'detailed'],
        description='Level of inspection detail (affects altitude and speed)'
    )
    
    # === MISSION TIMING AND SAFETY ===
    mission_timeout_arg = DeclareLaunchArgument(
        'mission_timeout',
        default_value='2400',  # 40 minutes
        description='Survey mission timeout in seconds'
    )
    
    auto_start_delay_arg = DeclareLaunchArgument(
        'auto_start_delay',
        default_value='15.0',
        description='Delay before auto-starting survey (seconds, 0 to disable)'
    )
    
    weather_check_arg = DeclareLaunchArgument(
        'weather_check',
        default_value='true',
        choices=['true', 'false'],
        description='Enable weather suitability check before survey'
    )
    
    # Get launch configuration values
    host_ip = LaunchConfiguration('host_ip')
    host_port = LaunchConfiguration('host_port')
    survey_type = LaunchConfiguration('survey_type')
    survey_vehicle = LaunchConfiguration('survey_vehicle')
    
    # === SURVEY VEHICLE NODE ===
    survey_vehicle_node = Node(
        package='airsim_ros_pkgs',
        executable='mission_multirotor_node',
        name=[survey_vehicle],  # Ultra-clean naming
        namespace='',
        output='screen',
        parameters=[{
            'host_ip': host_ip,
            'host_port': host_port,
            'vehicle_name': survey_vehicle,
            'mission_type': 'survey',
            'survey_altitude': LaunchConfiguration('survey_altitude'),
            'survey_speed': LaunchConfiguration('survey_speed'),
            'line_spacing': LaunchConfiguration('line_spacing'),
            'lateral_overlap': LaunchConfiguration('lateral_overlap'),
            'forward_overlap': LaunchConfiguration('forward_overlap'),
            'photo_interval': LaunchConfiguration('photo_interval'),
            'enable_mapping': LaunchConfiguration('enable_mapping'),
            'enable_multispectral': LaunchConfiguration('enable_multispectral'),
            'mission_timeout_seconds': LaunchConfiguration('mission_timeout'),
            'survey_pattern': LaunchConfiguration('survey_pattern'),
            'crop_type': LaunchConfiguration('crop_type'),
            'infrastructure_type': LaunchConfiguration('infrastructure_type'),
            'inspection_detail': LaunchConfiguration('inspection_detail')
        }],
        arguments=['--ros-args', '--log-level', 'INFO'],
        remappings=[
            ('mission/status', [survey_vehicle, '/survey/status']),
            ('survey/data', [survey_vehicle, '/survey/data']),
            ('survey/photos', [survey_vehicle, '/survey/photos'])
        ]
    )
    
    # === SURVEY COORDINATION NODE ===
    survey_coordination_node = Node(
        package='airsim_ros_pkgs',
        executable='mission_coordination_node',
        name='survey_coordinator',
        namespace='',
        output='screen',
        parameters=[{
            'mission_type': 'survey',
            'survey_type': survey_type,
            'primary_vehicle': survey_vehicle,
            'survey_pattern': LaunchConfiguration('survey_pattern'),
            'enable_data_logging': LaunchConfiguration('enable_mapping'),
            'mission_timeout_seconds': LaunchConfiguration('mission_timeout')
        }],
        arguments=['--ros-args', '--log-level', 'INFO']
    )
    
    # === GRID SURVEY CLIENT ===
    grid_survey_client = TimerAction(
        period=LaunchConfiguration('auto_start_delay'),
        actions=[
            Node(
                package='airsim_ros_pkgs',
                executable='test_mission_client.py',
                name='grid_survey_client',
                output='screen',
                parameters=[{
                    'target_vehicle': survey_vehicle,
                    'mission_type': 'survey',
                    'survey_pattern': LaunchConfiguration('survey_pattern'),
                    'survey_area': LaunchConfiguration('survey_area_corners'),
                    'survey_altitude': LaunchConfiguration('survey_altitude'),
                    'survey_speed': LaunchConfiguration('survey_speed'),
                    'line_spacing': LaunchConfiguration('line_spacing'),
                    'lateral_overlap': LaunchConfiguration('lateral_overlap'),
                    'photo_interval': LaunchConfiguration('photo_interval')
                }],
                arguments=['--ros-args', '--log-level', 'INFO'],
                condition=IfCondition(PythonExpression([
                    \"'\", survey_type, \"' == 'grid_survey' and \",
                    \"float('\", LaunchConfiguration('auto_start_delay'), \"') > 0\"
                ]))
            )
        ],
        condition=IfCondition(PythonExpression([
            \"'\", survey_type, \"' == 'grid_survey'\"
        ]))
    )
    
    # === AGRICULTURAL SURVEY CLIENT ===
    agricultural_survey_client = TimerAction(
        period=LaunchConfiguration('auto_start_delay'),
        actions=[
            Node(
                package='airsim_ros_pkgs',
                executable='demo_mission_planner.py',
                name='agricultural_survey_client',
                output='screen',
                parameters=[{
                    'vehicle': survey_vehicle,
                    'mission_type': 'agricultural_survey',
                    'crop_type': LaunchConfiguration('crop_type'),
                    'growth_stage': LaunchConfiguration('growth_stage'),
                    'survey_altitude': LaunchConfiguration('survey_altitude'),
                    'line_spacing': LaunchConfiguration('line_spacing'),
                    'enable_multispectral': LaunchConfiguration('enable_multispectral'),
                    'photo_interval': LaunchConfiguration('photo_interval')
                }],
                arguments=['--ros-args', '--log-level', 'INFO'],
                condition=IfCondition(PythonExpression([
                    \"'\", survey_type, \"' == 'agricultural_survey' and \",
                    \"float('\", LaunchConfiguration('auto_start_delay'), \"') > 0\"
                ]))
            )
        ],
        condition=IfCondition(PythonExpression([
            \"'\", survey_type, \"' == 'agricultural_survey'\"
        ]))
    )
    
    # === INFRASTRUCTURE SURVEY CLIENT ===
    infrastructure_survey_client = TimerAction(
        period=LaunchConfiguration('auto_start_delay'),
        actions=[
            Node(
                package='airsim_ros_pkgs',
                executable='demo_mission_planner.py',
                name='infrastructure_survey_client',
                output='screen',
                parameters=[{
                    'vehicle': survey_vehicle,
                    'mission_type': 'infrastructure_survey',
                    'infrastructure_type': LaunchConfiguration('infrastructure_type'),
                    'inspection_detail': LaunchConfiguration('inspection_detail'),
                    'survey_altitude': LaunchConfiguration('survey_altitude'),
                    'survey_speed': LaunchConfiguration('survey_speed'),
                    'line_spacing': LaunchConfiguration('line_spacing'),
                    'photo_interval': LaunchConfiguration('photo_interval')
                }],
                arguments=['--ros-args', '--log-level', 'INFO'],
                condition=IfCondition(PythonExpression([
                    \"'\", survey_type, \"' == 'infrastructure_survey' and \",
                    \"float('\", LaunchConfiguration('auto_start_delay'), \"') > 0\"
                ]))
            )
        ],
        condition=IfCondition(PythonExpression([
            \"'\", survey_type, \"' == 'infrastructure_survey'\"
        ]))
    )
    
    # === ENVIRONMENTAL SURVEY CLIENT ===
    environmental_survey_client = TimerAction(
        period=LaunchConfiguration('auto_start_delay'),
        actions=[
            Node(
                package='airsim_ros_pkgs',
                executable='demo_mission_planner.py',
                name='environmental_survey_client',
                output='screen',
                parameters=[{
                    'vehicle': survey_vehicle,
                    'mission_type': 'environmental_survey',
                    'survey_pattern': LaunchConfiguration('survey_pattern'),
                    'survey_altitude': LaunchConfiguration('survey_altitude'),
                    'survey_speed': LaunchConfiguration('survey_speed'),
                    'line_spacing': LaunchConfiguration('line_spacing'),
                    'enable_mapping': LaunchConfiguration('enable_mapping'),
                    'enable_multi_altitude': LaunchConfiguration('enable_multi_altitude'),
                    'altitude_levels': LaunchConfiguration('altitude_levels'),
                    'photo_interval': LaunchConfiguration('photo_interval')
                }],
                arguments=['--ros-args', '--log-level', 'INFO'],
                condition=IfCondition(PythonExpression([
                    \"'\", survey_type, \"' == 'environmental_survey' and \",
                    \"float('\", LaunchConfiguration('auto_start_delay'), \"') > 0\"
                ]))
            )
        ],
        condition=IfCondition(PythonExpression([
            \"'\", survey_type, \"' == 'environmental_survey'\"
        ]))
    )
    
    # === DATA LOGGING AND ANALYSIS NODE ===
    survey_data_logger = Node(
        package='airsim_ros_pkgs',
        executable='simple_mission_coordination_demo.py',
        name='survey_data_logger',
        output='screen',
        parameters=[{
            'log_survey_data': LaunchConfiguration('enable_mapping'),
            'survey_vehicle': survey_vehicle,
            'output_format': 'geotiff',
            'enable_photogrammetry': LaunchConfiguration('enable_mapping'),
            'lateral_overlap': LaunchConfiguration('lateral_overlap'),
            'forward_overlap': LaunchConfiguration('forward_overlap')
        }],
        arguments=['--ros-args', '--log-level', 'INFO'],
        condition=IfCondition(LaunchConfiguration('enable_mapping'))
    )
    
    return LaunchDescription([
        # Launch arguments
        host_ip_arg,
        host_port_arg,
        survey_type_arg,
        survey_pattern_arg,
        survey_vehicle_arg,
        survey_area_corners_arg,
        gps_survey_area_arg,
        survey_altitude_arg,
        survey_speed_arg,
        line_spacing_arg,
        lateral_overlap_arg,
        forward_overlap_arg,
        photo_interval_arg,
        enable_mapping_arg,
        enable_multispectral_arg,
        enable_multi_altitude_arg,
        altitude_levels_arg,
        crop_type_arg,
        growth_stage_arg,
        infrastructure_type_arg,
        inspection_detail_arg,
        mission_timeout_arg,
        auto_start_delay_arg,
        weather_check_arg,
        
        # Survey vehicle node
        survey_vehicle_node,
        
        # Survey coordination
        survey_coordination_node,
        
        # Survey execution clients
        grid_survey_client,
        agricultural_survey_client,
        infrastructure_survey_client,
        environmental_survey_client,
        
        # Data logging and analysis
        survey_data_logger,
    ])