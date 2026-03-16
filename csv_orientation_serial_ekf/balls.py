import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import serial


# Faces defined by vertex indices (order matters)
faces = [
    [0, 1, 3, 2],  # bottom (-z)
    [4, 5, 7, 6],  # top (+z)
    [0, 1, 5, 4],  # front (-y)   depends on view; we treat consistently
    [2, 3, 7, 6],  # back (+y)
    [0, 2, 6, 4],  # left (-x)
    [1, 3, 7, 5]   # right (+x)
]

def plot_cube(roll, pitch, ax=None, side=1.0, color='cyan', alpha=0.5, edgecolor='k'):
    if ax is None:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

    # Vertices of a cube centered at (0,0,0) with side length `side`
    d = side / 2
    vertices = np.array([[ x*0.7, y*1.4, z*0.2]
                         for x in (-d, d)
                         for y in (-d, d)
                         for z in (-d, d)])

    # Rotation matrices
    # Roll  : rotation about x‑axis
    R_roll = np.array([[1, 0, 0],
                       [0, np.cos(roll), -np.sin(roll)],
                       [0, np.sin(roll), np.cos(roll)]])
    # Pitch : rotation about y‑axis
    R_pitch = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                        [0, 1, 0],
                        [-np.sin(pitch), 0, np.cos(pitch)]])
    # Combined rotation: first roll, then pitch
    R = R_pitch @ R_roll

    # Apply rotation to all vertices
    vertices_rotated = vertices @ R.T   # equivalent to (R @ vertices.T).T

    # Build the rotated faces
    rotated_faces = [[vertices_rotated[idx] for idx in face] for face in faces]

    # Create 3D polygon collection
    cube = Poly3DCollection(rotated_faces,
                            facecolors=color,
                            edgecolors=edgecolor,
                            alpha=alpha,
                            linewidths=1)
    ax.add_collection3d(cube)

    # Set axes limits large enough to contain the rotated cube in all orientations
    margin = side * 0.2
    max_range = side * np.sqrt(3) / 2 + margin
    ax.set_xlim(-max_range, max_range)
    ax.set_ylim(-max_range, max_range)
    ax.set_zlim(-max_range, max_range)

    # Make axes aspect ratio equal
    ax.set_box_aspect([1, 1, 1])

    # Label axes
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    return ax, cube, vertices, faces

def animate_cube_serial(port='COM3', baudrate=9600, side=1.0, color='cyan', alpha=0.5, edgecolor='k', interval=50):
    # Open serial port
    try:
        ser = serial.Serial(port, baudrate, timeout=0)  # non‑blocking
        print(f"Opened serial port {port} at {baudrate} baud.")
    except Exception as e:
        print(f"Error opening serial port: {e}")
        return

    # Set up the figure and axes
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')

    # Initial cube (roll=pitch=0)
    ax, cube, vertices, faces = plot_cube(0, 0, ax, side, color, alpha, edgecolor)

    # Store last valid angles in degrees (for display)
    last_roll_deg = 0.0
    last_pitch_deg = 0.0

    # Text annotation to show current angles
    angle_text = ax.text2D(0.05, 0.95, "", transform=ax.transAxes, fontsize=12,
                           verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    # Function to close serial port when figure is closed
    def on_close(event):
        if ser.is_open:
            ser.close()
            print("Serial port closed.")
    fig.canvas.mpl_connect('close_event', on_close)

    # Update function for animation
    def update(frame):
        nonlocal last_roll_deg, last_pitch_deg

        # Try to read a line from serial
        if ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8').strip()
                if line:
                    # Parse comma‑separated values
                    parts = line.split(',')
                    if len(parts) >= 2:
                        roll_deg = float(parts[0])
                        pitch_deg = float(parts[1])
                        # Update last known values
                        last_roll_deg = roll_deg
                        last_pitch_deg = pitch_deg
            except Exception as e:
                print(f"Error parsing serial data: {e}")

        # Convert degrees to radians for rotation matrices
        roll = np.radians(last_roll_deg)
        pitch = np.radians(last_pitch_deg)

        # Rotation matrices
        R_roll = np.array([[1, 0, 0],
                           [0, np.cos(roll), -np.sin(roll)],
                           [0, np.sin(roll), np.cos(roll)]])
        R_pitch = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                            [0, 1, 0],
                            [-np.sin(pitch), 0, np.cos(pitch)]])
        R = R_pitch @ R_roll

        # Apply rotation
        vertices_rotated = vertices @ R.T
        new_segments = [[vertices_rotated[idx] for idx in face] for face in faces]
        cube.set_verts(new_segments)

        # Update the angle display
        angle_text.set_text(f"Roll: {last_roll_deg:.1f}°\nPitch: {last_pitch_deg:.1f}°")

        return cube, angle_text

    # Create animation
    anim = FuncAnimation(fig, update, interval=interval, blit=False, cache_frame_data=False)
    plt.show()
    return anim

if __name__ == '__main__':
    animate_cube_serial(port='COM11', baudrate=9600, side=2.0, color='orange', alpha=0.6)
