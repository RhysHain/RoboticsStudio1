#!/usr/bin/env python3
##
# @file generate_world.py
# @brief Generates a complete SDF world file with randomly placed trees and a red box.
#
# This script creates an SDF world file named `generated_world.sdf` located in:
# `/Simulation Files/41068_ignition_bringup_with_drone/41068_ignition_bringup/worlds`
# relative to the script location.
#
# The script generates:
# - A forest floor grid of 25 ground tiles.
# - A configurable number of trees placed randomly in the world.
# - A single red box model placed randomly in the same range.
#
# The output file is fully compatible with Ignition Gazebo / Gazebo Fortress or later.
#
# @note Adjust the `num_pine_trees`, `min_coord`, and `max_coord` parameters
#       to change the number and distribution of trees.
#
# @author Rhys
# @date 2025-10-31
##

import random
import os

##
# @brief Generates a complete SDF world file with randomised trees and a red box.
#
# The generated file includes the base environment, forest floor, randomly placed
# oak trees, and one red box model. The number of trees and coordinate range
# can be easily configured via parameters at the top of the script.
##
def main():
    ##
    # @brief Number of trees to generate in the world.
    ##
    num_pine_trees = 50

    ##
    # @brief Minimum and maximum coordinate range for random placement.
    ##
    min_coord = -60
    max_coord = 60

    ##
    # @brief Default Z-axis and rotation values for the generated models.
    ##
    z_tree = 0
    roll = pitch = yaw = 0
    z_red_box = 0.5

    ##
    # @brief Output directory for the generated world file.
    # The path is relative to the script location.
    ##
    output_dir = os.path.join(
        "Simulation Files",
        "41068_ignition_bringup_with_drone",
        "41068_ignition_bringup",
        "worlds"
    )

    # Ensure the output directory exists
    os.makedirs(output_dir, exist_ok=True)

    ##
    # @brief Full path of the generated SDF file.
    ##
    output_file = os.path.join(output_dir, "generated_world.sdf")

    # ---------------------------
    # Build the SDF header section
    # ---------------------------
    sdf_header = """<?xml version="1.0"?>
<sdf version='1.8'>
  <world name="new_world">
    <!-- Basic world setup  -->
    <plugin name='ignition::gazebo::systems::Physics' filename='ignition-gazebo-physics-system' />
    <plugin name='ignition::gazebo::systems::UserCommands' filename='ignition-gazebo-user-commands-system' />
    <plugin name='ignition::gazebo::systems::SceneBroadcaster' filename='ignition-gazebo-scene-broadcaster-system' />
    <plugin name='ignition::gazebo::systems::Contact' filename='ignition-gazebo-contact-system' />

    <!-- Lighting -->
    <light name="sun" type="directional">
      <cast_shadows>0</cast_shadows>
      <pose>0 0 100 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.8 0.8 0.8 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
      <intensity>5</intensity>
    </light>

    <!-- Physics & environment -->
    <gravity>0 0 -9.81</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <physics name="default_physics" type="ignored">
      <max_step_size>0.01</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>100</real_time_update_rate>
    </physics>
    <scene>
      <ambient>1 1 1 1</ambient>
      <background>0.6 0.8 1.0 1</background>
      <shadows>0</shadows>
      <grid>0</grid>
    </scene>

    <!-- Georeference -->
    <spherical_coordinates>
      <latitude_deg>0.0</latitude_deg>
      <longitude_deg>0.0</longitude_deg>
      <elevation>10.0</elevation>
      <heading_deg>0</heading_deg>
      <surface_model>EARTH_WGS84</surface_model>
    </spherical_coordinates>

    <!-- Large ground planes with forest floor texture -->
    <model name="forest_floor">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <link name="base"/>
"""

    # ---------------------------
    # Generate the forest floor grid
    # ---------------------------
    forest_plane_includes = ""
    for x in [-50, -25, 0, 25, 50]:
        for y in [-50, -25, 0, 25, 50]:
            forest_plane_includes += f"      <include>\n"
            forest_plane_includes += f"        <uri>model://forest_plane</uri>\n"
            forest_plane_includes += f"        <name>forest_plane_x{x}_y{y}</name>\n"
            forest_plane_includes += f"        <pose>{x} {y} 0 0 0 0</pose>\n"
            forest_plane_includes += f"      </include>\n"

    forest_plane_includes += "    </model>\n\n"

    # ---------------------------
    # Generate the pine forest models
    # ---------------------------
    pine_forest = "    <model name=\"pine_forest\">\n"
    pine_forest += "      <static>true</static>\n"
    pine_forest += "      <pose>0 0 0 0 0 0</pose>\n"
    pine_forest += "      <link name=\"base\"/>\n"

    ##
    # @brief Loop to create each tree include block with randomised positions.
    ##
    for i in range(num_pine_trees):
        x = round(random.uniform(min_coord, max_coord), 1)
        y = round(random.uniform(min_coord, max_coord), 1)
        pine_forest += f"      <include>\n"
        pine_forest += f"        <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/oak%20tree</uri>\n"
        pine_forest += f"        <name>oak_tree_{i}</name>\n"
        pine_forest += f"        <pose>{x} {y} {z_tree} {roll} {pitch} {yaw}</pose>\n"
        pine_forest += f"      </include>\n"

    pine_forest += "    </model>\n\n"

    # ---------------------------
    # Generate a random red box model
    # ---------------------------
    red_box_x = round(random.uniform(min_coord, max_coord), 1)
    red_box_y = round(random.uniform(min_coord, max_coord), 1)

    red_box = f"""    <!-- Red box (visual only, no collision) -->
    <model name="red_box">
      <pose>{red_box_x} {red_box_y} {z_red_box} 0 0 0</pose>
      <static>true</static>

      <link name="visual_only_link">
        <visual name="visual">
          <geometry>
            <box><size>0.4 0.4 1.8</size></box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
            <emissive>0 0 0 1</emissive>
          </material>
        </visual>
      </link>
    </model>
"""

    # ---------------------------
    # Footer to close the world
    # ---------------------------
    sdf_footer = "  </world>\n</sdf>\n"

    # Combine all parts into a complete SDF
    sdf_complete = sdf_header + forest_plane_includes + pine_forest + red_box + sdf_footer

    # Write to file
    with open(output_file, "w") as f:
        f.write(sdf_complete)

    print(f"SDF file '{output_file}' generated with {num_pine_trees} trees and random red_box position.")


##
# @brief Entry point of the script.
##
if __name__ == "__main__":
    main()
