# VEX U (VURC) Override 2026-27: game basics for programmers

We compete in **VEX U** (collegiate), not standard V5RC. Source: VEX V5 Override Game Manual v2.1, Section 6 (pp. 95-102) plus the base rules it modifies (Sections 1-3). Where Section 6 and the base rules conflict, **Section 6 wins**. Rule IDs are given so you can look things up: `VU*` are VEX U rules, others (SC, SG, GG, R) are base rules. Coordinates are in [field-coordinates.md](field-coordinates.md).

Our repo builds the **15" robot** (target `15_bot_v_1`). Each Match, a Team brings two Robots: one up to 24x24x24" and one up to 15x15x15" at the start (VUR1).

## Field
Same Override field as V5 (12'x12' foam tiles, about 140.4" inside the walls, 9 Goals, 4 Loaders, 4 Toggles). VEX U setup differences (Figure VEXU-1, p98):
- The **VEX GPS code strip is installed** on the field walls.
- **Only the Midfield (centre) Goal starts with a Pin**, a yellow/yellow Pin Placed in it. All other Goals start empty.
- Match Loads (off-field): Red gets 10 Cups, 10 red/yellow Pins and 3 yellow/yellow Pins. Blue gets 10 Cups, 10 blue/yellow Pins and 3 yellow/yellow Pins.
- Tape: **Autonomous Line** (diagonals plus the diamond edges) and **Midfield** (the centre diamond). 4 **Quadrants** (Red 1 left, Red 2 bottom, Blue 2 top, Blue 1 right) with 2 Goals and 1 Toggle each. 4 corner **Load Zones**.

## Match formats (VEX U)
| Match | Auton | Driver | Robots |
|---|---|---|---|
| Head-to-Head (1 Team vs 1 Team, VUT1) | **0:30** (VUT4) | **1:30** (VUT5) | 2 per Team (one 24", one 15") |
| Driving Skills | none | 1:00 | up to 2 (VURS2) |
| Autonomous Coding Skills | 1:00 | none | up to 2 (VURS2) |

- Auton can end early only if both Teams and the Head Referee agree and the option was announced at the event meeting (VUT4b).
- Red places one Robot first, then both blue Robots, then the second red Robot (VUG2).
- Skills Matches are otherwise as in base Section 3.

## Scoring (evaluated after the Match, 5 s settle, SC1)
| Item | Points |
|---|---|
| Scored alliance-colour Pin Half | 5 |
| Scored yellow Pin Half (must be Owned) | 10 |
| Robot ending at least partly in Midfield (SC6) | 8 |
| Autonomous Bonus (most points at end of auton) | 12 (6 each on a tie) |

- **Placed** (SC2): a Pin is Placed if nested in a Goal, or in a Cup nested with another Placed Pin. Each Goal / Cup half holds at most one Pin Half.
- **Scored** (SC3): each visible half of a Placed Pin scores by colour (red for red, blue for blue).
- **Toggle set** (SC4): 3 discrete states (yellow / red / blue), fully seated, touching no Robot. Otherwise it counts as neutral yellow.
- **Owned yellow** (SC5): in a quadrant Goal, the alliance whose colour the Toggle is set to. In the Midfield Goal, the alliance with more Robots in Midfield at the end.
- **Autonomous Bonus differs in VEX U (VUG5)**: auton scoring **includes** Midfield yellow ownership (the alliance with more Robots in Midfield at auton end owns those halves) and the 8 points for a Robot ending in Midfield. In V5 these are excluded.
- **Autonomous Win Point (VUG6)**, no Violations in auton, all of:
  1. At least **12** Pin Halves Scored for your alliance.
  2. At least **4** Goals each holding at least 2 of your Scored Pin Halves.
  3. Neither Robot contacting the Field Perimeter.
  4. At least one Robot at least partly in Midfield.
  (Pin Halves and Goals in Quadrants on the opposing side of the Autonomous Line do not count for 1 and 2.)

## Robot Skills specifics (VURS1-3 over RSC2-RSC5)
- Both Robots may be used, and **both must start in legal red-alliance starting positions**. Each starts with one red/yellow Pin as its Preload (SG5). All other SG1 start conditions apply.
- Match Loads off-field: **4 red/yellow Pins, 6 blue/yellow Pins, 10 Cups** (Figure VURS1, p101). They enter only through the **red** Loaders, at any time.
- All Goals start empty in V5 Skills. Check Figure VURS1 for the VEX U Skills setup (the Midfield yellow Pin may differ).
- Robots move freely after the start (no Autonomous Line limit in Skills).
- Red halves score only in a red Quadrant or Midfield, blue only in a blue Quadrant or Midfield, yellow only if Owned. Ownership in Skills follows base RSC rules (Midfield by a Robot ending in it, quadrants by a matching Toggle).
- GPS strip is on the field. Optional Skills Stop Time as in base Section 3.

## Robot limits that matter for code
- **Start size** (VUR1): 24x24x24" robot and 15x15x15" robot. Each must be fully stopped and touching only its Preload (SG1; VUG1 swaps the size).
- **Moving electronics are allowed at the start** (VUG3): cooling fans, spinning LIDAR and similar Additional Electronics may run, but must not move the Robot or any mechanism and must not touch Scoring Objects or other Robots.
- **Expansion** (SG2 / SG3, unchanged): never larger than 24" x 24" footprint or 50" tall at any time. For the 15" robot this is a software limit too: arms, claws and pivots that swing out must stay inside 24" x 24" at every moment.
- **Possession** (SG6): at most 1 Pin and 1 Cup at once.
- **Electronics** (VUR10-13): exactly one V5 Brain and at least one V5 Radio per Robot; unlimited V5 Smart Motors (11 W and 5.5 W only; the 55 W drivetrain limit does **not** apply); **Sensors, External Processors and Additional Electronics (e.g. Raspberry Pi, LIDAR) are allowed** if they connect to the Brain through its external ports and never drive motors or solenoids directly. Robot-to-robot wireless is allowed only through V5 Brains and Radios. Commercial Electromechanical Assemblies are not legal (VUR13).
- Fabricated parts (3D printing, machining) are allowed under VUR3-7. Not relevant to code but explains why mechanisms differ from V5.

## Autonomous rules
- No human input during auton (VUT4a, GG12).
- **Loaders are open in auton (VUG4)**: Drive Team Members may introduce Match Loads through their alliance-colour Loaders, and operate them, during both auton and driver control. Both Cups and Pins may be introduced. In V5 this is driver-control only.
- Do not contact tiles, Scoring Objects or Field Elements on the opponent's side of the Autonomous Line (SG7). Objects touching the Autonomous Line are neutral.
- Midfield and Autonomous-Line objects are contested; expect the opposing Robots there (SG8).
- Any Violation in auton gives the opponent the Autonomous Bonus and forfeits your AWP (GG13). If both Teams commit one, no Bonus is awarded.

## Protected things (don't touch)
- Opposing alliance-coloured Goals and the Pins in them (SG9). Neutral Goals: do not remove Placed objects (SG10).
- Load Zones during driver control: do not block or stay in the opponent's, and do not touch the opponent's Loaders (SG13). Not applicable in auton. (VUG7, a VEX U Load Zone rule, was added and then removed again; it is not in v2.1.)
- Midfield Goal: cannot Place objects during Endgame (SG12).
- No grabbing or hooking onto Field Elements (GG9); no Robot completely outside the Field; no holding a Robot longer than a 3-count (GG17).
- Do not intentionally remove Scoring Objects from the Field (SG4).

## Sensors and field hints
- Goals carry AprilTags: the centre Goal is ID 0 and quadrant Goals use IDs 1-4 (see [field-coordinates.md](field-coordinates.md)). The GPS code strip allows the V5 GPS sensor for absolute pose in both H2H and Skills.
- Field components vary slightly event to event. Leave margin on odometry-critical moves.

## Manual page map
VEX U Section 6: pp. 95-110 (field setup p97-98, VUG p99-100, Skills p100-101, Tournament p101-102, Robot p102-110). Scoring p27-32, Specific Game Rules p33-44, Robot Skills p58-61, Field specs Appendix A p129-145.
