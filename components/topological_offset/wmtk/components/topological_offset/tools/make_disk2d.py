#!/usr/bin/env python3
"""Write a MSH 4.1 ASCII mesh of a DISK domain with a concentric disk tagged tag_0.

The simplest possible 2D offset input: everything is radially symmetric, so the exact answer
is an annulus and the region area is pi*(R+d)^2 with no boundary effects to argue about.

    make_disk2d.py [rings] [R_inner] [R_domain] [out.msh]
"""
import math, sys

M = int(sys.argv[1]) if len(sys.argv) > 1 else 60      # radial rings
RIN = float(sys.argv[2]) if len(sys.argv) > 2 else 0.20
RDOM = float(sys.argv[3]) if len(sys.argv) > 3 else 1.00
out = sys.argv[4] if len(sys.argv) > 4 else "disk2d.msh"

NT = 6 * M   # angular divisions, chosen so cells stay near-equilateral at the rim

nodes = [(0.0, 0.0)]                       # centre
ring = []                                  # ring[k] = node tags on ring k+1
for k in range(1, M + 1):
    r = RDOM * k / M
    tags = []
    for a in range(NT):
        th = 2.0 * math.pi * a / NT
        nodes.append((r * math.cos(th), r * math.sin(th)))
        tags.append(len(nodes))            # 1-based
    ring.append(tags)

tris = []
for a in range(NT):                        # centre fan
    tris.append((1, ring[0][a], ring[0][(a + 1) % NT]))
for k in range(M - 1):                     # quad strips, split consistently
    lo, hi = ring[k], ring[k + 1]
    for a in range(NT):
        b = (a + 1) % NT
        tris.append((lo[a], hi[a], hi[b]))
        tris.append((lo[a], hi[b], lo[b]))

def inside(t):
    cx = sum(nodes[i - 1][0] for i in t) / 3.0
    cy = sum(nodes[i - 1][1] for i in t) / 3.0
    return cx * cx + cy * cy < RIN * RIN

disk = [t for t in tris if inside(t)]
amb = [t for t in tris if not inside(t)]

L = ["$MeshFormat\n4.1 0 8\n$EndMeshFormat",
     '$PhysicalNames\n2\n2 1 "ambient"\n2 2 "tag_0"\n$EndPhysicalNames',
     "$Entities\n0 0 2 0",
     f"1 {-RDOM} {-RDOM} 0 {RDOM} {RDOM} 0 1 1 0",
     f"2 {-RDOM} {-RDOM} 0 {RDOM} {RDOM} 0 1 2 0",
     "$EndEntities",
     f"$Nodes\n1 {len(nodes)} 1 {len(nodes)}",
     f"2 1 0 {len(nodes)}"]
L += [str(i + 1) for i in range(len(nodes))]
L += [f"{x!r} {y!r} 0" for x, y in nodes]
L.append("$EndNodes")
n = len(amb) + len(disk)
L.append(f"$Elements\n2 {n} 1 {n}")
tag = 1
L.append(f"2 1 2 {len(amb)}")
for t in amb:
    L.append(f"{tag} {t[0]} {t[1]} {t[2]}"); tag += 1
L.append(f"2 2 2 {len(disk)}")
for t in disk:
    L.append(f"{tag} {t[0]} {t[1]} {t[2]}"); tag += 1
L.append("$EndElements")
open(out, "w").write("\n".join(L) + "\n")
print(f"{out}: disk domain R={RDOM}, inner disk R={RIN} -- "
      f"{len(nodes)} nodes, {n} tris ({len(disk)} inner, {len(amb)} ambient)")
