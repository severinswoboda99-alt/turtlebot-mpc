import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import matplotlib.pyplot as plt
import numpy as np

# Read data from rosbag2
def read_bag(bag_path, topic_name, msg_type_str):
    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path,
        storage_id='mcap'
    )
    converter_options = rosbag2_py.ConverterOptions('', '')

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    msg_type = get_message(msg_type_str)

    data = []
    while reader.has_next():
        topic, raw_data, t = reader.read_next()
        if topic == topic_name:
            msg = deserialize_message(raw_data, msg_type)
            data.append((t, msg))
    return data

# Extract planned path data, only last message
def extract_planned(path_msgs):
    if not path_msgs:
        return [], []

    _, msg = path_msgs[-1]

    xs, ys = [], []
    for pose in msg.poses:
        xs.append(pose.pose.position.x)
        ys.append(pose.pose.position.y)

    return xs, ys

# Extract travelled path data, only last message
def extract_travelled(travelled_msgs):
    if not travelled_msgs:
        return [], [], []

    # Take ONLY the last message (contains full trajectory)
    _, msg = travelled_msgs[-1]

    xs, ys = [], []
    for pose_stamped in msg.poses:
        xs.append(pose_stamped.pose.position.x)
        ys.append(pose_stamped.pose.position.y)

    return xs, ys, []

# Compute deviation between target and travelled trajectory
def compute_error(px, py, tx, ty):
    errors = []
    planned = np.array(list(zip(px, py)))
    for x, y in zip(tx, ty):
        dists = np.linalg.norm(planned - np.array([x, y]), axis=1)
        errors.append(np.min(dists))
    return errors

# Plotting Execution
if __name__ == "__main__":
    bag_path = "/home/swo/turtlebot-mpc/src/mpc-tracker/rosbag2/rosbag2_2026_04_09-15_51_18"  # rosbag2 folder
    planned = read_bag(bag_path, "/path", "nav_msgs/msg/Path")
    travelled = read_bag(bag_path, "/travelled_path", "nav_msgs/msg/Path")

    px, py = extract_planned(planned)
    tx, ty, _ = extract_travelled(travelled)

    errors = compute_error(px, py, tx, ty)

    # Plot trajectories
    plt.figure()
    plt.plot(px, py, linestyle='--', label='Planned Path', color='g')
    plt.plot(tx, ty, label='Executed Path', color='b')
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.legend()
    plt.axis('equal')
    plt.title("Trajectory Comparison")
    plt.savefig("trajectory.pdf")
    plt.show()

    # Plot error
    plt.figure()
    plt.plot(errors, color='r')
    plt.xlabel("Time index")
    plt.ylabel("Deviation [m]")
    plt.title("Tracking Error")
    plt.savefig("error_plot.pdf")
    plt.show()