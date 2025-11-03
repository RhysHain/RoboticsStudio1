#!/usr/bin/env python3
"""
Generate a line (lawnmower) search pattern for a drone.

Each line in the output file contains:
    x y z
separated by spaces.

The pattern begins at the specified start point and sweeps
the defined area in a back-and-forth pattern.
"""

import numpy as np
import os

def generate_search_pattern(start_x: float,
                            start_y: float,
                            z: float,
                            x_min: float,
                            x_max: float,
                            y_min: float,
                            y_max: float,
                            spacing: float,
                            filename: str = "line_search_pattern.txt"):
    """
    Generates a line search pattern and writes it to a text file.

    Args:
        start_x (float): Starting x coordinate
        start_y (float): Starting y coordinate
        z (float): Constant altitude
        x_min (float): Minimum x boundary of the search area
        x_max (float): Maximum x boundary of the search area
        y_min (float): Minimum y boundary of the search area
        y_max (float): Maximum y boundary of the search area
        spacing (float): Distance between each sweep line
        filename (str): Output text file name
    """
    # Generate grid ranges
    y_values = np.arange(y_min, y_max + spacing, spacing)
    x_values = np.linspace(x_min, x_max, int((x_max - x_min) / spacing) + 1)

    coords = [(start_x, start_y, z)]

    # Find the nearest y line to the start
    nearest_y_idx = np.argmin(np.abs(y_values - start_y))
    y_values = y_values[nearest_y_idx:]

    # Start with the first sweep from the starting x,y
    direction = 1
    first_line = [x for x in x_values if x >= start_x]
    for x in first_line:
        coords.append((x, start_y, z))

    # Continue with alternating direction lines (lawnmower)
    for y in y_values[1:]:
        if direction == 1:
            for x in reversed(x_values):
                coords.append((x, y, z))
        else:
            for x in x_values:
                coords.append((x, y, z))
        direction *= -1

    # Write to file
    with open(filename, "w") as f:
        for x, y, z in coords:
            f.write(f"{x:.2f} {y:.2f} {z:.2f}\n")

    print(f"✅ Search pattern saved to '{filename}' ({len(coords)} points).")


if __name__ == "__main__":
    # Example usage — edit these values as needed

    output_dir = os.path.join(
        "Code",
        "config"
    )

    # Ensure the output directory exists
    os.makedirs(output_dir, exist_ok=True)

    ##
    # @brief Full path of the generated SDF file.
    ##
    output_file = os.path.join(output_dir, "searchPatternPoints.txt")
    generate_search_pattern(
        start_x=0,
        start_y=0,
        z=6,
        x_min=-50,
        x_max=50,
        y_min=-50,
        y_max=50,
        spacing=5,
        filename=output_file
    )
