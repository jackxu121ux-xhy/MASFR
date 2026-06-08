from launch import LaunchDescription
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart
from launch.actions import RegisterEventHandler, LogInfo

def generate_launch_description():
    
    crawl_control = Node(
        package='multimodal_motion',
        executable='node_crawling'
    )

    px4_control = Node(
        package='multimodal_motion',
        executable='node_px4'
    )

    manipulation = Node(
        package='multimodal_motion',
        executable='manipulation'
    )

    launch1 = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=crawl_control,
            on_start=[
                LogInfo(msg='Crawling node has started, PX4 node is starting!'),
                px4_control
            ]
        )
    )

    launch2 = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=crawl_control,
            on_start=[
                LogInfo(msg='PX4 node has started, manipulation node is starting!'),
                manipulation
            ]
        )
    )

    return LaunchDescription([
        crawl_control,
        launch1,
        launch2
    ])