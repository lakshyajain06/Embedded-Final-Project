"""Generate two STL parts in millimeters using manifold3d and NumPy."""
from pathlib import Path
import struct

from manifold3d import Manifold
import numpy as np


def box(x, y, z, dx, dy, dz):
    return Manifold.cube((dx, dy, dz)).translate((x, y, z))


def export_stl(name, solid):
    assert solid.status().name == "NoError" and len(solid.decompose()) == 1
    mesh = solid.to_mesh()
    with Path(__file__).with_name(name).open("wb") as output:
        output.write(b"SynapSense enclosure".ljust(80, b"\0"))
        output.write(struct.pack("<I", len(mesh.tri_verts)))
        for indices in mesh.tri_verts:
            triangle = mesh.vert_properties[indices, :3]
            normal = np.cross(triangle[1] - triangle[0], triangle[2] - triangle[0])
            normal /= np.linalg.norm(normal)
            output.write(struct.pack("<12fH", *normal, *triangle.flatten(), 0))
    print(name, len(mesh.tri_verts), "triangles", solid.bounding_box())


# 34 x 28 mm shell. The board faces left, with the battery beneath it.
base = box(-17, -14, 0, 34, 28, 14) - box(-15.4, -12.4, 1.6, 30.8, 24.8, 13)
for shelf_x in (-14.4, 2.2):
    base += box(shelf_x, -13, 7.1, 2.5, 26, 1.4)
# USB opening extends to the rim so no bridge blocks the plug.
base -= box(-18, -7, 8.4, 3.6, 14, 6)

# Opposing 22 mm watch-strap gaps. The 2 mm bores take spring-bar tips.
for lug_x, pin_x in ((-24, -21), (16, 21)):
    for lug_y in (-14, 11):
        base += box(lug_x, lug_y, 0, 8, 3, 6.5)
        hole = Manifold.cylinder(3.4, 1, circular_segments=24)
        base -= hole.rotate((90, 0, 0)).translate((pin_x, lug_y + 3.2, 3.8))

# Print top face down. The tabs sit inside the two long walls.
lid = box(-17, -14, 0, 34, 28, 1.6)
for tab_y in (-12.15, 10.65):
    lid += box(-13, tab_y, 1.4, 26, 1.5, 1.4)
# Open viewing slot for the board's LEDs.
lid -= box(-15, -9, -0.1, 7, 18, 2)
export_stl("base.stl", base)
export_stl("lid.stl", lid)
