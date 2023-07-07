import subprocess


def launch_rviz(rviz_config):
    """Launches RViz with the given configuration.

    Args:
        rviz_config (str): Path to the RViz configuration file.

    Returns:
        subprocess.Popen: The RViz subprocess.
    """
    process = subprocess.Popen(["ros2", "run", "rviz2", "rviz2", "-d", rviz_config])
    return process

def launch_bridge():
    process = subprocess.Popen(["ros2", "launch", "rosbridge_server", "rosbridge_websocket_launch.xml" ])
    return process

def start_node(node_script_path):
    """Starts a node script as a separate process.

    Args:
        node_script_path (str): Path to the node script file.

    Returns:
        subprocess.Popen: The node script subprocess.
    """
    process = subprocess.Popen(['python3', node_script_path])
    return process


def main():
    """Main function to launch RViz and start node scripts.
    
    Launches RViz with a given configuration and starts two node scripts.
    It waits for these processes to complete. If KeyboardInterrupt is caught,
    it terminates the subprocesses and waits for them to finish.

    """
    # rviz_process = launch_rviz('./visualizer/map.rviz')
    multimedia_collection = start_node('./multimedia/videoSync.py')
    bridge = launch_bridge()

    try:
        # rviz_process.wait()
        multimedia_collection.wait()
        bridge.wait()
    except KeyboardInterrupt:
        # rviz_process.terminate()
        multimedia_collection.terminate()
        bridge.terminate()
        # rviz_process.wait()
        multimedia_collection.wait()
        bridge.wait()


if __name__ == '__main__':
    main()
