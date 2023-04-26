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

def plot_walkable_area(coords, sample_rate):
    y, x = coords
    x_scaled = x * sample_rate
    y_scaled = y * sample_rate
    plt.scatter(x_scaled, y_scaled, c='red', marker='.')
    plt.gca().invert_yaxis()
    plt.gca().set_aspect('equal', adjustable='box')
    plt.show()

def print_points(coords, sample_rate):
    y, x = coords
    points = np.vstack((x * sample_rate, y * sample_rate)).T
    for point in points:
        print(f"({point[0]}, {point[1]})")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <PGM_FILE> <SAMPLE_RATE>")
        sys.exit(1)

    pgm_file = sys.argv[1]
    sample_rate = int(sys.argv[2])

    image = load_pgm_image(pgm_file)
    coords = discretize_walkable_area(image, sample_rate)

    plot_walkable_area(coords, sample_rate)
    print_points(coords, sample_rate)
