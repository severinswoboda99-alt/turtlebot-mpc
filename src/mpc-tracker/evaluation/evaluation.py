import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import matplotlib.pyplot as plt
import numpy as np
import math

def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def angle_diff(a, b):
    d = a - b
    return math.atan2(math.sin(d), math.cos(d))

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
        return [], [], []

    _, msg = path_msgs[-1]

    xs, ys, yaws = [], [], []
    for pose in msg.poses:
        xs.append(pose.pose.position.x)
        ys.append(pose.pose.position.y)
        yaws.append(quaternion_to_yaw(pose.pose.orientation))

    return xs, ys, yaws

# Extract traveled path data, only last message
def extract_traveled(traveled_msgs, movement_threshold=0.01):
    if not traveled_msgs:
        return [], [], []

    _, msg = traveled_msgs[-1]

    xs, ys, yaws = [], [], []

    prev_x = None
    prev_y = None
    start_idx = 0

    for i, pose_stamped in enumerate(msg.poses):
        x = pose_stamped.pose.position.x
        y = pose_stamped.pose.position.y

        if prev_x is not None:
            dist = math.hypot(x - prev_x, y - prev_y)
            if dist > movement_threshold:
                start_idx = i
                break

        prev_x = x
        prev_y = y

    for pose_stamped in msg.poses[start_idx:]:
        xs.append(pose_stamped.pose.position.x)
        ys.append(pose_stamped.pose.position.y)
        yaws.append(quaternion_to_yaw(pose_stamped.pose.orientation))

    return np.array(xs), np.array(ys), np.array(yaws)

# Extract computing time data
def extract_loop_time(loop_msgs):
    times = []
    for _, msg in loop_msgs:
        times.append(msg.data)
    return np.array(times)

# Extract inputs
def extract_cmd_vel(cmd_vel_msgs, threshold=1e-6):
    if not cmd_vel_msgs:
        return [], [], []

    times, v_actuals, w_actuals = [], [], []
    for t, msg in cmd_vel_msgs:
        v = msg.twist.linear.x
        w = msg.twist.angular.z
        if abs(v) > threshold or abs(w) > threshold:
            times.append(t)
            v_actuals.append(v)
            w_actuals.append(w)

    return np.array(times), np.array(v_actuals), np.array(w_actuals)

# Compute deviation between target and traveled trajectory
def compute_pos_error(px, py, tx, ty):
    pos_errors = []
    planned = np.array(list(zip(px, py)))
    for x, y in zip(tx, ty):
        dists = np.linalg.norm(planned - np.array([x, y]), axis=1)
        pos_errors.append(np.min(dists))
    return pos_errors

def compute_heading_error(px, py, pyaw, tx, ty, tyaw):
    errors = []
    planned_xy = np.array(list(zip(px, py)))

    for x, y, yaw_t in zip(tx, ty, tyaw):
        dists = np.linalg.norm(planned_xy - np.array([x, y]), axis=1)
        idx = np.argmin(dists)

        yaw_p = pyaw[idx]
        err = angle_diff(yaw_t, yaw_p)
        errors.append(err)

    return np.array(errors)

def compute_control_effort(v, w):
    dv = np.diff(v)
    dw = np.diff(w)
    return dv, dw

# Plotting Execution
plt.rcParams['font.size'] = '12'

if __name__ == "__main__":
    bag_path = "/home/swo/turtlebot-mpc/src/mpc-tracker/rosbag2/rosbag2_2026_04_17-11_10_40"  # rosbag2 folder
    planned = read_bag(bag_path, "/path", "nav_msgs/msg/Path")
    traveled = read_bag(bag_path, "/traveled_path", "nav_msgs/msg/Path")
    loop_time = read_bag(bag_path, "/loop_time", "std_msgs/msg/Float64")
    cmd_vel = read_bag(bag_path, "/cmd_vel", "geometry_msgs/msg/TwistStamped")
    
    px, py, pyaw = extract_planned(planned)
    tx, ty, tyaw = extract_traveled(traveled)
    t_l = extract_loop_time(loop_time)
    t_cmd, v_cmd, w_cmd = extract_cmd_vel(cmd_vel)
    
    # Computing Time per Loop
    filtered = t_l[t_l < np.percentile(t_l, 99)]
    avg_time = np.mean(filtered)
    std_time = np.std(filtered)
    max_time = np.max(filtered)
    min_time = np.min(filtered)

    print(f"Average solve time: {avg_time:.3f} ms")
    print(f"Std dev: {std_time:.3f} ms")
    print(f"Min: {min_time:.3f} ms")
    print(f"Max: {max_time:.3f} ms")

    # Position Error
    pos_errors = compute_pos_error(px, py, tx, ty)
    
    avg_pos_error = np.mean(pos_errors)
    std_pos_error = np.std(pos_errors)
    max_pos_error = np.max(pos_errors)
    
    print(f"Average pos error: {avg_pos_error:.3f} m")
    print(f"Std dev: {std_pos_error:.3f} m")
    print(f"Max: {max_pos_error:.3f} m")
    
    # Heading Error
    heading_errors = compute_heading_error(px, py, pyaw, tx, ty, tyaw)
    heading_errors_deg = np.degrees(heading_errors)
    
    avg_heading_error = np.mean(heading_errors_deg)
    std_heading_error = np.std(heading_errors_deg)
    max_heading_error = np.max(heading_errors_deg)
    
    print(f"Average heading error: {avg_heading_error:.3f} deg")
    print(f"Std dev: {std_heading_error:.3f} deg")
    print(f"Max: {max_heading_error:.3f} deg")
    
    # Control Effort
    print("First 5 v_actual:", v_cmd[:5])
    print("First 5 w_actual:", w_cmd[:5])
    dv, dw = compute_control_effort(v_cmd, w_cmd)

    print(f"Average Δv: {np.mean(dv):.3f} m/s²")
    print(f"Std dev: {np.std(dv):.3f} m/s²")
    print(f"Max: {np.max(dv):.3f} m/s²")

    print(f"Average Δw: {np.mean(dw):.3f} rad/s²")
    print(f"Std dev: {np.std(dw):.3f} rad/s²")
    print(f"Max: {np.max(dw):.3f} rad/s²")

    # Plot planned path
    # plt.figure()
    # plt.plot(px, py, linestyle='--', label='Planned Path', color='b')
    # plt.xlabel("X [m]")
    # plt.ylabel("Y [m]")
    # plt.legend()
    # plt.axis('equal')
    # plt.title("Path 3")
    # plt.grid(linewidth = 0.5)
    # plt.savefig("planned_3.pdf")
    # plt.show()
    
    # Plot trajectories
    # plt.figure()
    # plt.plot(px, py, linestyle='--', label='Planned Path', color='b')
    # plt.plot(tx, ty, label='Robot Trajectory', color='crimson')
    # plt.xlabel("X [m]")
    # plt.ylabel("Y [m]")
    # plt.legend()
    # plt.axis('equal')
    # plt.title("Trajectory-Path Comparison")
    # plt.grid(linewidth = 0.5)
    # plt.savefig("trajectory_comp.pdf")
    # plt.show()
    
    # # Plot computing loop time
    # plt.figure()
    # plt.plot(filtered, color='turquoise')
    # plt.axhline(y=avg_time, color='r', linestyle='-')
    # plt.xlabel("Time index")
    # plt.ylabel("Time [ms]")
    # plt.title("Computing Time")
    # plt.savefig("loop_time_plot.pdf")
    # plt.show()
    
    # # Histogram loop time
    # plt.figure()
    # plt.hist(filtered, bins=30, color='lightseagreen')
    # plt.xlabel("Time [ms]")
    # plt.ylabel("Frequency")
    # plt.title("Computing Time Distribution")
    # plt.grid()
    # plt.savefig("loop_time_hist.pdf")
    # plt.show()
    
    # # Plot position error
    # plt.figure()
    # plt.plot(pos_errors, color='blueviolet')
    # plt.axhline(y=avg_pos_error, color='r', linestyle='-')
    # plt.xlabel("Time index")
    # plt.ylabel("Deviation [m]")
    # plt.title("Position Error")
    # plt.savefig("pos_error_plot.pdf")
    # plt.show()
    
    # # Histogram position error
    # plt.figure()
    # plt.hist(pos_errors, bins=30, color='indigo')
    # plt.xlabel("Deviation [m]")
    # plt.ylabel("Frequency")
    # plt.title("Position Error Distribution")
    # plt.grid()
    # plt.savefig("pos_error_hist.pdf")
    # plt.show()
    
    # # Plot heading error
    # plt.figure()
    # plt.plot(heading_errors_deg, color='slateblue')
    # plt.axhline(y=avg_heading_error, color='r', linestyle='-')
    # plt.xlabel("Time index")
    # plt.ylabel("Deviation [deg]")
    # plt.title("Heading Error")
    # plt.grid()
    # plt.savefig("heading_error_plot.pdf")
    # plt.show()
    
    # # Histogram heading error
    # plt.figure()
    # plt.hist(heading_errors_deg, bins=30, color='darkslateblue')
    # plt.xlabel("Deviation [deg]")
    # plt.ylabel("Frequency")
    # plt.title("Heading Error Distribution")
    # plt.grid()
    # plt.savefig("heading_error_hist.pdf")
    # plt.show()
    
    # Plot control effort
    plt.figure()
    plt.plot(t_cmd[1:], dv, label='Δv', color='blue')
    plt.plot(t_cmd[1:], dw, label='Δw', color='red')
    plt.axhline(y=0, color='k', linestyle='--')
    plt.xlabel("Time [s]")
    plt.ylabel("Control Effort")
    plt.title("Control Effort (Input Deltas)")
    plt.legend()
    plt.grid()
    plt.savefig("control_effort.pdf")
    plt.show()