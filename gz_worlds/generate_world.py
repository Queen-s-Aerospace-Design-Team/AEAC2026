"""
Generate a world sdf file, can change "box" to be randomly sized walls for testing.
"""

import xml.etree.ElementTree as ET
from xml.dom import minidom

# Create root
sdf = ET.Element("sdf", version="1.9")

# Create world
world = ET.SubElement(sdf, "world", name="walls")

# Physics
physics = ET.SubElement(world, "physics", type="ode")
ET.SubElement(physics, "max_step_size").text = "0.004"
ET.SubElement(physics, "real_time_factor").text = "1.0"
ET.SubElement(physics, "real_time_update_rate").text = "250"

# Gravity & magnetic field
ET.SubElement(world, "gravity").text = "0 0 -9.8"
ET.SubElement(world, "magnetic_field").text = "6e-06 2.3e-05 -4.2e-05"
ET.SubElement(world, "atmosphere", type="adiabatic")

# Scene
scene = ET.SubElement(world, "scene")
ET.SubElement(scene, "grid").text = "false"
ET.SubElement(scene, "ambient").text = "0.4 0.4 0.4 1"
ET.SubElement(scene, "background").text = "0.7 0.7 0.7 1"
ET.SubElement(scene, "shadows").text = "true"

# Ground plane
ground = ET.SubElement(world, "model", name="ground_plane")
ET.SubElement(ground, "static").text = "true"
link = ET.SubElement(ground, "link", name="link")

# Collision
collision = ET.SubElement(link, "collision", name="collision")
geometry = ET.SubElement(collision, "geometry")
plane = ET.SubElement(geometry, "plane")
ET.SubElement(plane, "normal").text = "0 0 1"
ET.SubElement(plane, "size").text = "1 1"

surface = ET.SubElement(collision, "surface")
ET.SubElement(ET.SubElement(surface, "friction"), "ode")
ET.SubElement(surface, "bounce")
ET.SubElement(surface, "contact")

# Visual
visual = ET.SubElement(link, "visual", name="visual")
geometry_v = ET.SubElement(visual, "geometry")
plane_v = ET.SubElement(geometry_v, "plane")
ET.SubElement(plane_v, "normal").text = "0 0 1"
ET.SubElement(plane_v, "size").text = "100 100"

material = ET.SubElement(visual, "material")
ET.SubElement(material, "ambient").text = "0.8 0.8 0.8 1"
ET.SubElement(material, "diffuse").text = "0.8 0.8 0.8 1"
ET.SubElement(material, "specular").text = "0.8 0.8 0.8 1"

ET.SubElement(link, "pose").text = "0 0 0 0 -0 0"

inertial = ET.SubElement(link, "inertial")
ET.SubElement(inertial, "pose").text = "0 0 0 0 -0 0"
ET.SubElement(inertial, "mass").text = "1"
inertia = ET.SubElement(inertial, "inertia")
ET.SubElement(inertia, "ixx").text = "1"
ET.SubElement(inertia, "ixy").text = "0"
ET.SubElement(inertia, "ixz").text = "0"
ET.SubElement(inertia, "iyy").text = "1"
ET.SubElement(inertia, "iyz").text = "0"
ET.SubElement(inertia, "izz").text = "1"

ET.SubElement(link, "enable_wind").text = "false"
ET.SubElement(ground, "pose").text = "0 0 0 0 -0 0"
ET.SubElement(ground, "self_collide").text = "false"

# Function to create box models
def create_box(world, name, pose, size):
    model = ET.SubElement(world, "model", name=name)
    ET.SubElement(model, "pose").text = pose
    ET.SubElement(model, "static").text = "true"
    link = ET.SubElement(model, "link", name=f"link{name[-1]}")

    collision = ET.SubElement(link, "collision", name="collision")
    geometry = ET.SubElement(collision, "geometry")
    box = ET.SubElement(geometry, "box")
    ET.SubElement(box, "size").text = size

    visual = ET.SubElement(link, "visual", name="visual")
    geometry_v = ET.SubElement(visual, "geometry")
    box_v = ET.SubElement(geometry_v, "box")
    ET.SubElement(box_v, "size").text = size

    material = ET.SubElement(visual, "material")
    ET.SubElement(material, "ambient").text = "0.3 0.3 0.3 1"
    ET.SubElement(material, "diffuse").text = "0.7 0.7 0.7 1"
    ET.SubElement(material, "specular").text = "1 1 1 1"

# Add boxes
create_box(world, "box1", "5 0 7.5 0 0 0", "1 20 15")
create_box(world, "box2", "-3 5 7.5 0 0 0", "10 1 15")
create_box(world, "box3", "13 -10 7.5 0 0 0", "17 1 15")
create_box(world, "box4", "12 0 7.5 0 0 0", "1 20 15")

# Convert ElementTree to a string (Otherwise it will be one line)
rough_string = ET.tostring(sdf, 'utf-8')

# Use minidom to add lines
reparsed = minidom.parseString(rough_string)
pretty_xml = reparsed.toprettyxml(indent="  ")  # 2 spaces for indentation

# Write to file
with open("aeac.sdf", "w", encoding="utf-8") as f:
    f.write(pretty_xml)

print("SDF file 'aeac.sdf' created successfully.")