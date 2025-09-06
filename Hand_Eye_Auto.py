import numpy as np
import cv2
from opcua import Client, ua
from scipy.spatial.transform import Rotation as R
import os
import glob
import time

# --- OPC UA Server (PLC) ---
url = "opc.tcp://192.168.10.10:4840"
client = Client(url)
start = False


def calculate_reprojection_error(world_points_array, image_points_array, K, dist, rvecs, tvecs):
    """
    Calculate the reprojection error after camera calibration.
    """
    total_error = 0
    for i in range(len(world_points_array)):
        image_points_projected, _ = cv2.projectPoints(world_points_array[i], rvecs[i], tvecs[i], K, dist)
        error = np.linalg.norm(image_points_array[i] - image_points_projected.reshape(-1, 2), axis=1)
        total_error += np.sum(error)

        mean_error = np.mean(error)
        print(f"Average reprojection error for pose {i + 1}: {mean_error}")

    mean_error = total_error / len(world_points_array) / len(world_points_array[0])
    print(f"\nOverall average reprojection error: {mean_error}")
    return total_error, mean_error


def get_robot_position():
    """
    Reads the robot position and orientation from the OPC UA server,
    applies decimal/sign corrections, and returns translation & rotation.
    """
    client.connect()

    # Read robot position and rotation values from PLC (example structure)
    node = client.get_node('ns=4;i=172')
    x = node.get_value()
    node = client.get_node('ns=4;i=173')
    x_dec = node.get_value()
    node = client.get_node('ns=4;i=166')
    x_signal = node.get_value()

    node = client.get_node('ns=4;i=174')
    y = node.get_value()
    node = client.get_node('ns=4;i=175')
    y_dec = node.get_value()
    node = client.get_node('ns=4;i=167')
    y_signal = node.get_value()

    node = client.get_node('ns=4;i=176')
    z = node.get_value()
    node = client.get_node('ns=4;i=177')
    z_dec = node.get_value()
    node = client.get_node('ns=4;i=168')
    z_signal = node.get_value()

    node = client.get_node('ns=4;i=178')
    w = node.get_value()
    node = client.get_node('ns=4;i=179')
    w_dec = node.get_value()
    node = client.get_node('ns=4;i=169')
    w_signal = node.get_value()

    node = client.get_node('ns=4;i=180')
    p = node.get_value()
    node = client.get_node('ns=4;i=181')
    p_dec = node.get_value()
    node = client.get_node('ns=4;i=170')
    p_signal = node.get_value()

    node = client.get_node('ns=4;i=182')
    r = node.get_value()
    node = client.get_node('ns=4;i=183')
    r_dec = node.get_value()
    node = client.get_node('ns=4;i=171')
    r_signal = node.get_value()

    client.disconnect()

    # Apply decimal correction
    x += x_dec / 1000
    y += y_dec / 1000
    z += z_dec / 1000
    w += w_dec / 1000
    p += p_dec / 1000
    r += r_dec / 1000

    # Apply sign correction
    if x_signal:
        x = -x
    if y_signal:
        y = -y
    if z_signal:
        z = -z
    if w_signal:
        w = -w
    if p_signal:
        p = -p
    if r_signal:
        r = -r

    robot_translation = np.array([x, y, z], dtype=np.float32)
    robot_rotation = np.array([w, p, r], dtype=np.float32)

    print("Robot translation:", robot_translation)
    print("Robot rotation (Euler angles):", robot_rotation)

    # Convert Euler angles (XYZ) to rotation matrix
    rotation_matrix = euler_to_rotation_matrix(robot_rotation)

    # Acknowledge position registered
    client.connect()
    node = client.get_node("ns=4;i=161")
    node.set_value(ua.DataValue(ua.Variant(True, ua.VariantType.Boolean)))
    client.disconnect()

    return robot_translation, rotation_matrix


def euler_to_rotation_matrix(angles_deg):
    """
    Converts Euler angles (XYZ) in degrees to a 3x3 rotation matrix.
    """
    angles_rad = np.radians(angles_deg)
    rotation_matrix = R.from_euler('xyz', angles_rad).as_matrix()
    return rotation_matrix


# --- Initialization ---
image_folder = "C:\\Users\\NunoAlves\\Desktop\\Results\\"
image_extensions = ("*.png", "*.jpg", "*.jpeg", "*.bmp", "*.tiff")

grid_size = (23, 13)   # Calibration grid squares (cols, rows)
square_size = 10.0     # Square size in mm
camera_resolution = (800, 600)

# Center of the grid
center_x = (grid_size[0] - 1) * square_size / 2
center_y = (grid_size[1] - 1) * square_size / 2

# Storage lists
object_points = []
image_points = []
robot_translation_vectors = []
robot_rotation_vectors = []

# Remove old images
for ext in image_extensions:
    for image in glob.glob(os.path.join(image_folder, ext)):
        os.remove(image)
print("All old images removed.")

# Signal PLC to start calibration
client.connect()
client.get_node("ns=4;i=104").set_value(ua.DataValue(ua.Variant(True, ua.VariantType.Boolean)))
client.disconnect()

client.connect()
client.get_node("ns=4;i=227").set_value(ua.DataValue(ua.Variant(False, ua.VariantType.Boolean)))
client.disconnect()

# --- Robot position collection ---
for i in range(9):
    while not start:
        client.connect()
        start = client.get_node("ns=4;i=86").get_value()
        client.disconnect()
    start = False

    # Record robot position
    robot_translation, robot_rotation = get_robot_position()
    robot_rotation_vectors.append(robot_rotation)
    robot_translation_vectors.append(robot_translation)

    print("\nCalibration step:", i + 1)

    # Acknowledge robot position saved
    client.connect()
    client.get_node("ns=4;i=85").set_value(ua.DataValue(ua.Variant(True, ua.VariantType.Boolean)))
    client.disconnect()

# Wait for images to be saved
time.sleep(2)

# Load calibration images
image_files = []
for ext in image_extensions:
    image_files.extend(glob.glob(os.path.join(image_folder, ext)))

image_files.sort(key=os.path.getmtime)
image_files = image_files[:9]

# Process calibration images
for image_file in image_files:
    image = cv2.imread(image_file)
    if image is None:
        print(f"Error loading image: {image_file}")
        continue

    ret, corners = cv2.findChessboardCorners(image, grid_size, None)

    if ret:
        image_points.append(corners.reshape(-1, 2))

        # Generate world points centered at grid
        world_points = np.zeros((grid_size[0] * grid_size[1], 3), np.float32)
        for i in range(grid_size[1]):
            for j in range(grid_size[0]):
                x = j * square_size - center_x
                y = center_y - i * square_size
                world_points[i * grid_size[0] + j] = [x, y, 0]

        object_points.append(world_points)

# --- Camera calibration ---
ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(object_points, image_points, camera_resolution, None, None)

total_error, mean_error = calculate_reprojection_error(object_points, image_points, K, dist, rvecs, tvecs)

print("\nIntrinsic matrix:\n", K)
print("\nDistortion coefficients:\n", dist)

# --- Hand-eye calibration ---
R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
    robot_rotation_vectors, robot_translation_vectors, rvecs, tvecs, cv2.CALIB_HAND_EYE_TSAI
)

r_cam2gripper, _ = cv2.Rodrigues(R_cam2gripper)
r_cam2gripper_deg = np.degrees(r_cam2gripper)
dist_mm = np.linalg.norm(t_cam2gripper)

print(f"r_cam2gripper (deg): {r_cam2gripper_deg}")
print(f"t_cam2gripper: {t_cam2gripper}")
print(f"Camera-tool distance: {dist_mm:.2f} mm")

# Example world-to-base transformation
R_world2cam, _ = cv2.Rodrigues(rvecs[0])
R_gripper2base = robot_rotation_vectors[0]
t_world2cam = tvecs[0].reshape(3, 1)
t_cam2gripper = t_cam2gripper.reshape(3, 1)
t_gripper2base = robot_translation_vectors[0].reshape(3, 1)

P_world = np.array([[0], [0], [0]])
P_cam = R_world2cam @ P_world + t_world2cam
P_gripper = R_cam2gripper @ P_cam + t_cam2gripper
P_base = R_gripper2base @ P_gripper + t_gripper2base

print("Point in robot base frame:\n", P_base.flatten())

# --- Send calibration results to PLC ---
client.connect()

# Translation
client.get_node("ns=4;i=163").set_value(ua.DataValue(ua.Variant(t_cam2gripper[0], ua.VariantType.Float)))
client.get_node("ns=4;i=164").set_value(ua.DataValue(ua.Variant(t_cam2gripper[1], ua.VariantType.Float)))
client.get_node("ns=4;i=165").set_value(ua.DataValue(ua.Variant(t_cam2gripper[2], ua.VariantType.Float)))

# Rotation (degrees)
client.get_node("ns=4;i=229").set_value(ua.DataValue(ua.Variant(r_cam2gripper_deg[0], ua.VariantType.Float)))
client.get_node("ns=4;i=230").set_value(ua.DataValue(ua.Variant(r_cam2gripper_deg[1], ua.VariantType.Float)))
client.get_node("ns=4;i=231").set_value(ua.DataValue(ua.Variant(r_cam2gripper_deg[2], ua.VariantType.Float)))

client.disconnect()

# Signal calibration done
client.connect()
client.get_node("ns=4;i=226").set_value(ua.DataValue(ua.Variant(True, ua.VariantType.Boolean)))
client.disconnect()

print("\nCalibration Done!\n")