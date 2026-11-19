# Override field coordinates

Source: Game Manual v2.1, Appendix A (pp. 129-145: Field Element Locations p137, Tape Lines p138, Scoring Object Locations p139, AprilTags p143, perimeter p140-141, element drawings p132-136).

## Frame and conventions
- **Origin**: inside bottom-left corner of the walls, **audience view** (the view in the manual drawings).
- **+X** points right (toward the blue Alliance Station). **+Y** points up the drawing (toward the "0 deg" wall). **Heading 0 = +Y**, which matches LemLib (0 = +y, clockwise positive). Red Alliance Station is on the left (x < 0), blue on the right (x > 140.41).
- Units are inches. The inside wall-to-wall distance is 140.41" (portable perimeter) or 140.50" (metal perimeter). The numbers below use 140.41". Mid-field is 70.20".
- Confidence: **[D]** dimensioned in the manual, **[S]** derived by symmetry or the tile grid, **[A]** approximate, read off the low-resolution drawing. Check [A] items against the official STEP model before using them for odometry-critical moves.
- The field has 180-degree rotational symmetry: `(x, y) -> (140.41 - x, 140.41 - y)` maps red items to blue items.

## Grid lines [D]
Tile-edge lines (the dimension lines in the drawings) are at 23.11, 46.66, 70.20, 93.75, 117.30 on both axes. Most elements sit on these intersections.

## Goals [D] (centre positions)
Heights: Alliance Goal 3.25", neutral quadrant Goal 5.77", centre Goal 8.77". Octagon about 5.6" across the base (4.7" body).

| Goal | Colour | x | y | Quadrant | AprilTag ID |
|---|---|---|---|---|---|
| Centre | neutral (tall) | 70.20 | 70.20 | Midfield | 0 |
| Red A | red | 23.11 | 46.66 | Red 1 (left) | 2 |
| Red B | red | 46.66 | 23.11 | Red 2 (bottom) | 3 |
| Blue A | blue | 117.30 | 93.75 | Blue 1 (right) | 2 |
| Blue B | blue | 93.75 | 117.30 | Blue 2 (top) | 3 |
| N1 | neutral (short) | 23.11 | 93.75 | Red 1 (left) | 1 |
| N2 | neutral (short) | 93.75 | 23.11 | Red 2 (bottom) | 4 |
| N3 | neutral (short) | 117.30 | 46.66 | Blue 1 (right) | 1 |
| N4 | neutral (short) | 46.66 | 117.30 | Blue 2 (top) | 4 |

The IDs repeat per alliance side (1-4 on the red half, 1-4 on the blue half; `0` is unique), so also use the tag's colour or position to tell them apart. Each Goal has several tags (one per face).

## Quadrants [S]
Each quadrant is a triangle between the perimeter and the Midfield diamond, split by the diagonal tape lines from the four corners.
- **Red 1** (left): Goals Red A, N1; Toggle on left wall.
- **Red 2** (bottom): Goals Red B, N2; Toggle on bottom wall.
- **Blue 1** (right): Goals Blue A, N3; Toggle on right wall.
- **Blue 2** (top): Goals Blue B, N4; Toggle on top wall.

## Toggles [D]
Four Toggles, one centred on each wall, mounted on the perimeter.
- Centres: left (0, 70.20), bottom (70.20, 0), right (140.41, 70.20), top (70.20, 140.41).
- Length 25.99" (660.2 mm). Axle height 13.20" above the floor. Triangular, 2.03" across.
- Start: yellow facing up and into the field, alliance colour facing out of the field (neutral).
- A Toggle counts as set only when a face lies flat against its mounts.

## Midfield and Autonomous Line
- Midfield is the diamond whose corners are on the tile grid: left (46.66, 70.20), right (93.75, 70.20), bottom (70.20, 46.66), top (70.20, 93.75) [S] (tape centre lines).
- Autonomous Line: double white tape runs along the two diagonals from corner to corner and around the diamond. The tape pairs are about 2.5" apart (63.5 mm, [D]).
- 28 Scoring Objects start on the Autonomous Line and count as neutral (manual p38).

## Loaders and Load Zones
- 4 Loaders, 2 per alliance, mounted on the **left and right walls** near the corners [D from p137]:
  - Red (left wall x = 0): centres at y = 11.44 and y = 128.97.
  - Blue (right wall x = 140.41): centres at y = 11.44 and y = 128.97.
- Loader footprint: 3.74" deep (into the field) x 4.02" wide. Opening on top is 3.39" dia, 14.37" tall resting, 22.97" raised. The Loader is lifted by Drive Team to insert objects; Robots remove them from the bottom opening.
- Load Zones: L-shaped tape around each Loader. Approximate region [A]: x in 0 to about 11.7 and y in 0 to 23.11 (bottom corners) or 117.30 to 140.41 (top corners), mirrored on the right wall. Tape outer edge sits at a tile centre (about 11.7") on one leg and on the grid line (23.11 / 117.30) on the other.

## Alliance Stations and Match Loads [A]
- Red station is outside the left wall, blue outside the right wall; Match Loads are held by the Drive Team outside the field and fed into the Loaders by hand. Skills counts are in game-basics.md.
- Preload positions (red: 2 near the station corners; blue mirrored). In Skills only the red station is used.

## Starting Scoring Object layout (manual p139, V5 H2H; VEX U figures p98 and p101) [D unless noted]
We play VEX U. The VEX U field (Figures VEXU-1, VURS1) uses the same crosses, wall clusters and diamond pins below, but **only the centre Goal starts with a Pin (one yellow/yellow)** and the GPS code strip is installed. Its Match Loads are 10 Cups, 10 colour/yellow Pins and 3 yellow/yellow Pins per alliance; Skills uses 4 red/yellow, 6 blue/yellow and 10 Cups. Both Skills Robots start on the red (left) side.
Coordinates are object centres. `+-3.16` is half a Pin/Cup width plus offset.

### Pin crosses (each: 4 horizontal Pins at N/E/S/W around the grid point; the arms touch the centre)
| Cross centre | Arm centres |
|---|---|
| (23.11, 117.30) [S] | (19.95, 117.30), (26.27, 117.30), (23.11, 114.14), (23.11, 120.46) |
| (46.66, 93.75) | (43.50, 93.75), (49.82, 93.75), (46.66, 90.59), (46.66, 96.91) |
| (93.75, 46.66) | (90.59, 46.66), (96.91, 46.66), (93.75, 43.50), (93.75, 49.82) |
| (117.30, 23.11) [S] | (114.14, 23.11), (120.46, 23.11), (117.30, 19.95), (117.30, 26.27) |

Each cross has two red/yellow and two blue/yellow Pins (8 + 8 over the field). Exact colour order per arm: see Figure FO-2 (manual p23) [A].

### Wall clusters (each: Cup, yellow Pin, Cup, three in a row parallel to the wall)
- Top wall, y = 138.83: x = 43.50 / 46.66 / 49.82 and x = 90.59 / 93.75 / 96.91.
- Bottom wall, y = 1.58: same x values.
- Left wall, x = 1.58: y = 43.50 / 46.66 / 49.82 and y = 90.59 / 93.75 / 96.91.
- Right wall, x = 138.83: same y values.

The middle item of each cluster is a yellow/yellow Pin; the two outer items are Cups [A on exact orientation].

### Other Pins [A]
- 4 red/blue Pins at the Midfield diamond corners: (46.66, 70.20), (93.75, 70.20), (70.20, 46.66), (70.20, 93.75). Colour up per Figure FO-2.
- Remaining yellow/yellow Pins were not fully placed; extra yellow Pins appear near (23.11, 23.11) and (117.30, 117.30) in the p139 drawing [A].
- Pin and Cup counts differ between V5 and VEX U (extra Match Load Pins), so do not rely on the V5 inventory; use Figures VEXU-1 and VURS1.

## Object and element dimensions [D]
| Item | Size |
|---|---|
| Pin | 6.50" tall, 3.16" max dia (flange), 2.35" and 1.40" ends |
| Cup | 6.48" tall, 3.16" top dia, 2.32" base dia |
| Alliance Goal | 3.25" tall, 5.61" base |
| Neutral quadrant Goal | 5.77" tall |
| Centre Goal | 8.77" tall |
| Toggle | 25.99" long, 2.03" face, axle 13.20" high |
| Loader | 3.74" deep, 4.02" wide, 14.37" tall (22.97" raised) |
| Wall | 11.54" tall (portable) / 11.50" (metal), 2.00" / 1.27" thick |

## Converting to the repo's local auton frame
The auton code uses a local frame: start pose = (0, 0) heading 0, +y forward, +x right. If the robot starts at field pose `(sx, sy, theta)` (theta clockwise from +Y, degrees), convert a field point `(fx, fy)` with:

```
dx = fx - sx;  dy = fy - sy
lx =  dx * cos(theta) - dy * sin(theta)
ly =  dx * sin(theta) + dy * cos(theta)
```

Forward is (sin theta, cos theta) and right is (cos theta, -sin theta) in field axes, so the formula above is a dot product with each. Example: a robot at (12, 40) facing +X (theta = 90) sees the Red A Goal at (23.11, 46.66) at local (-6.66, 11.11): 11.11" ahead, 6.66" to the left. `score()` adds its own `scoreShiftX/Y` origin shift on top of this.

## Known limits of this document
- Object colours and orientations inside the crosses, the diamond-corner Pins and the corner yellow Pins were read from low-resolution figures and may differ in detail.
- Load Zone tape extents are inferred from the tape-detail drawing.
- Real fields differ by a fraction of an inch (T5).
