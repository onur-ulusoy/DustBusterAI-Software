import sys
import numpy as np
import cv2
import matplotlib.pyplot as plt

def load_pgm_image(file_path):
    return cv2.imread(file_path, cv2.IMREAD_GRAYSCALE)

def discretize_walkable_area(image, sample_rate):
    walkable_area = np.where((image > 5) & (image < 255), 1, 0)
    sampled_area = walkable_area[::sample_rate, ::sample_rate]
    return np.where(sampled_area == 1)

def plot_walkable_area(coords, image_shape, sample_rate, resolution, origin):
    y, x = coords
    x_scaled = x * sample_rate * resolution + origin[0]
    y_scaled = origin[1] + image_shape[0] * resolution - y * sample_rate * resolution

    fig, ax = plt.subplots()
    ax.scatter(x_scaled, y_scaled, c='red', marker='.')
    ax.set_xlim(origin[0], origin[0] + image_shape[1] * resolution)
    ax.set_ylim(origin[1], origin[1] + image_shape[0] * resolution)
    ax.set_aspect('equal', adjustable='box')

    plt.show()

def print_points(coords, image_shape, sample_rate, resolution, origin):
    y, x = coords
    points = np.vstack((x * sample_rate * resolution + origin[0], origin[1] + image_shape[0] * resolution - y * sample_rate * resolution)).T
    for point in points:
        print(f"({point[0]}, {point[1]})")

def return_points(coords, image_shape, sample_rate, resolution, origin):
    y, x = coords
    points = np.vstack((x * sample_rate * resolution + origin[0], origin[1] + image_shape[0] * resolution - y * sample_rate * resolution)).T
    for point in points:
        point[0] = round(point[0], 2)
        point[1] = round(point[1], 2)
    return points

if __name__ == "__main__":
    if len(sys.argv) != 5:
        print(f"Usage: {sys.argv[0]} <PGM_FILE> <SAMPLE_RATE> <ORIGIN_X> <ORIGIN_Y>")
        sys.exit(1)

    pgm_file = sys.argv[1]
    sample_rate = int(sys.argv[2])
    origin_x = float(sys.argv[3])
    origin_y = float(sys.argv[4])

    image = load_pgm_image(pgm_file)
    coords = discretize_walkable_area(image, sample_rate)
    resolution = 0.05
    origin = (origin_x, origin_y)

    plot_walkable_area(coords, image.shape, sample_rate, resolution, origin)
    print_points(coords, image.shape, sample_rate, resolution, origin)
    points = return_points(coords, image.shape, sample_rate, resolution, origin)

    for point in points:
        print(f"({point[0]}, {point[1]})")