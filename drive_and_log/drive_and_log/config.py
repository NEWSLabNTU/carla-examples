from carla import Transform, Location, Rotation

#  WORLD = "Town03"
#  NPC1_ROUTE = [
#      # Transform(Location(-3.247418, 23.639963, 0.1), Rotation(0.0, 0.84, 0.0)),
#      # Transform(Location(1.474368, -22.95079, 0.1), Rotation(0.0, -174, 0.0)),
#      Transform(
#          Location(13.695746, 18.811819, 0.1), Rotation(-0.000031, -43.884972, -0.007568)
#      ),
#      Transform(
#          Location(22.825905, 1.144211, 0.1), Rotation(0.003324, -89.318871, -0.000676)
#      ),
#      Transform(
#          Location(18.419220, -13.330012, 0.1),
#          Rotation(-0.000854, -129.323746, -0.000246),
#      ),
#      Transform(
#          Location(-13.913259, -18.567612, 0.1), Rotation(-0.000031, 140.858994, 0.021522)
#      ),
#      Transform(
#          Location(-23.580221, -2.984560, 0.1), Rotation(-0.000122, 99.207008, -0.014651)
#      ),
#      Transform(
#          Location(-20.525547, 12.254155, 0.1), Rotation(-0.000305, 57.465370, -0.036132)
#      ),
#  ]
#  NPC2_ROUTE = [
#      Transform(Location(165.969940, -194.172440, 0.1), Rotation(0.0, 1.005202, 0.0)),
#      Transform(Location(232.168045, -21.722141, 0.1), Rotation(0.0, 92.112457, 0.0)),
#  ]
#  NPC3_ROUTE = [
#      Transform(Location(-16.424643, -135, 0.1), Rotation(0.0, 1.1394, 0.0)),
#      Transform(Location(7.562178, -181.43866, 0.1), Rotation(0.0, -88.1558, 0.0)),
#      Transform(Location(-46.9482, -205.7256, 0.1), Rotation(0.0, -177.528, 0.0)),
#      Transform(Location(-84.9675, -154.1095, 0.1), Rotation(0.0, 91.6498, 0.0)),
#  ]

WORLD = "Town10HD"
NPC3_ROUTE = [
    Transform(
        Location(x=-52.133560, y=-40.180298, z=0.482400),
        Rotation(pitch=0.000000, yaw=90.432304, roll=0.000000),
    ),
    Transform(
        Location(x=-81.243576, y=12.964413, z=0.000293),
        Rotation(pitch=-0.000369, yaw=-179.756287, roll=-0.000031),
    ),
    Transform(
        Location(x=-52.330223, y=-9.467075, z=0.000294),
        Rotation(pitch=-0.000061, yaw=89.905533, roll=0.000000),
    ),
    Transform(
        Location(x=-103.806343, y=52.093533, z=0.000292),
        Rotation(pitch=-0.004945, yaw=-88.720642, roll=-0.000214),
    ),
    Transform(
        Location(x=-72.047188, y=27.981750, z=0.000293),
        Rotation(pitch=-0.000396, yaw=0.196186, roll=0.000000),
    ),
    Transform(
        Location(x=-72.612579, y=128.264389, z=0.001414),
        Rotation(pitch=-0.004337, yaw=-164.806107, roll=-0.474091),
    ),
]
