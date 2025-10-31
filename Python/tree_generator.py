import random

# Parameters
num_trees = 1000  # number of trees to generate
z = 0
roll = pitch = yaw = 0
min_coord = -60
max_coord = 60
output_file = "tree_positions.xml"

with open(output_file, "w") as f:
    for i in range(num_trees):
        x = round(random.uniform(min_coord, max_coord), 1)
        y = round(random.uniform(min_coord, max_coord), 1)
        block = f"""<include>
  <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/oak%20tree</uri>
  <name>oak_tree_{i}</name>
  <pose>{x} {y} {z} {roll} {pitch} {yaw}</pose>
</include>

"""
        f.write(block)

print(f"{num_trees} tree positions written to {output_file}")
