# How Small Size League teams plan robot motion

This survey found **70 team entries with a documented method**. A row names an implementation or prototype the authors describe and dates the evidence. Several entries share code or research lineages. Results range from simulation tests to competition use. Teams can change planners between seasons.

**Coverage.** Screened 368 indexed SSL team descriptions spanning 2009–2025 and 28 PDFs linked by the official 2026 team list. Added earlier team publications for Cornell, FU-Fighters and RoboRoos. The archive is incomplete, especially before 2009. The league publishes descriptions and match results, with no all-time certification of collision-free planning. [League paper archive](https://ssl.robocup.org/team-description-papers/) · [2026 entries and papers](https://ssl.robocup.org/robocups/robocup-2026/robocup-2026-teams/) · [Paper index](https://tdpsearch.com)

## What applies to RoboTurk

**A route answers where. A timed trajectory also answers when.** Two teammates can follow intersecting routes safely if they reach the crossing at different times. TIGERs and ER-Force search trajectories the robot can accelerate and brake along. RobôCIn reports why this mattered in matches, including collisions when its robots could not follow sharp turns in a shortest path. [TIGERs publications](https://www.tigers-mannheim.de/publications/tdps/) · [RobôCIn §3.3](https://arxiv.org/abs/2307.10018)

**Next experiment for our 5v5 stack.** Have each team share time-stamped intended trajectories among its own players. Let the ball retriever plan first when support paths cross its approach. For opponents, predict motion from observed positions and enlarge clearance as prediction uncertainty grows. Compare deadlocks, teammate contacts, opponent contacts and recovery time on the same crowded starts. This is a proposed experiment. The touchline demo uses the existing reactive steering plus a board-aware recovery maneuver.

## Team-by-team evidence

RRT grows a tree toward sampled positions. ERRT biases that search using prior waypoints. A* and Dijkstra search graphs. Bang-bang trajectories accelerate and brake subject to limits. Velocity-obstacle methods select a velocity outside predicted collision regions.

| Team or joint entry | How the robots navigate | Primary source |
|---|---|---|
| ACES | Smoothed RRT paths, then added RRT* to improve paths. | [2015 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2015__ACES.pdf) |
| AIS | Tested potential fields and several waypoint-smoothed RRT variants, including RRT*. | [2018 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2018__AIS.pdf) |
| Austrian Cubes | Combined potential-field navigation with delay compensation and trajectory generation. | [2010 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2010__Austrian_Cubes.pdf) |
| B-Smart | Combined RRT path generation with reactive collision avoidance after the behavior layer. | [2010 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2010__B-Smart.pdf) |
| Botnia Dragon Knights | Used extended RRT (ERRT) for waypoints and trapezoidal speed profiles to follow them. | [2010 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2010__Botnia_Dragon_Knights.pdf) |
| BRocks | Grew a randomized search tree from the current state toward the target. | [2016 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2016__BRocks.pdf) |
| CMDragons (CMµs lineage) | Used ERRT for waypoints, then a dynamics-aware safety search to choose a safe acceleration. The navigation skill served multiple tactics. | [2013 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2013__CMDragons.pdf) · [safety paper](https://cse-robotics.engr.tamu.edu/dshell/cs689/papers/bruce06safe.pdf) · [2018 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2018__CMus.pdf) |
| Cornell Big Red | Replanned each frame. When a trajectory hit a circular obstacle, generated routes through via-points on either side and selected the better one. | [2000 team paper §3.3](https://www.cs.cornell.edu/boom/2005/ProjectArchive/robocup/documentation/robocup/2000/RoboCup_2000_Springer_Paper.pdf) |
| Cyrus | Guided an artificial potential field with a prior RRT path to help escape local minima. | [2016 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2016__Cyrus.pdf) |
| Delft Mercurians | Implemented local model predictive control with disk-separation and velocity constraints in a holonomic motion model. | [2024 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2024__Delft_Mercurians.pdf) |
| Eagle Knights | Rebuilt a geometric exploring tree every cycle. Tested another RRT implementation as a replacement. | [2013 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2013__Eagle_Knights.pdf) |
| Eagle Knights-Robobulls | The joint entry used an exploring tree with circular players and rectangular goals, then generated spline trajectories. | [2010 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2010__Eagle_Knights-Robobulls.pdf) |
| EMEnents | Implemented Tangent Bug for a static scene as a first planner. | [2013 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2013__EMEnents.pdf) |
| ER-Force | Searched feasible acceleration-limited trajectories, varying duration and acceleration direction, and checked them for collisions. | [2020 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2020__ER-Force.pdf) |
| First Order Robotics | Evaluated locally reachable velocities with a dynamic-window planner and used a bisector planner to escape traps. | [2025 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2025__First_Order.pdf) |
| FU-Fighters | Searched a cost grid. Obstacle-adjacent edges had high cost. A* selected the least-cost route. | [Team description](https://dcmlr.inf.fu-berlin.de/rojas/index.html%3Fp%3D464.html) |
| FURGbot | Provided a navigation service to the skill layer using ERRT with a k-d tree for the search. | [2014 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2014__FURGbot.pdf) |
| ibis | Replaced A* with RVO2 for robot encounters, preprocessing the preferred velocity around static obstacles. | [2026 ETDP §3.2](https://ssl.robocup.org/wp-content/uploads/2026/04/2026_ETDP_ibis.pdf) |
| Immortals | Fed ERRT waypoints to motion control, then passed the requested acceleration through Dynamic Safety Search. | [2015 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2015__Immortals.pdf) |
| IRSS Deluxe | Tested potential fields and RRT-family planners with cached waypoints and smoothing. The later AIS paper comes from the same university. | [2014 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2014__IRSS_Deluxe.pdf) |
| ITAndroids | Added forbidden polygons to a visibility graph to model restricted zones and obstructions. | [2025 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2025__ITAndroids.pdf) |
| KgpKubs | Implemented RRT and kinodynamic variants and investigated RRT-Connect and real-time RRT*. | [2018 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2018__KgpKubs.pdf) |
| KIKS | Compared RRT with Optimal Reciprocal Collision Avoidance (ORCA), which selects a velocity using pairwise constraints. | [2016 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2016__KIKS.pdf) |
| KN2C | Ran A* with six candidate neighbors around each robot. Its Voronoi calculation addressed attacker positioning separately. | [2015 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2015__KN2C.pdf) |
| Luhbots | Steered around obstacles using circular fields, a reactive extension of artificial potential fields. | [2023 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2023__Luhbots.pdf) |
| MCT Susano Logics | Replaced unstable repeated RRT plans with tangents around expanded opponent collision circles. | [2019 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2019__MCT_Susano_Logics.pdf) |
| MIT RoboTeam | Prototyped RRT-X. The tree rewires as obstacles move and supplies updated waypoints. The paper describes simulation development. | [2020 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2020__MIT_RoboTeam.pdf) |
| MRL | Used RRT as the navigation search tree, accepting extensions that cleared obstacles. | [2013 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2013__MRL.pdf) |
| NAMeC | Compared a deterministic search that advances toward the goal or branches around an obstruction with Informed RRT*. | [2025 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2025__NAMeC.pdf) |
| NEUIslanders | Used potential fields and modified the field computation to improve its speed. The paper also describes RRT. | [2017 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2017__NEUIslanders.pdf) |
| NUST Falcons | Used velocity obstacles based on relative motion to avoid predicted encounters. | [2011 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2011__NUST_Falcons.pdf) |
| ODENS | Inflated obstacles, alternated direct segments with following their bounding boxes, then tested visibility to shorten the route. | [2010 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2010__ODENS.pdf) |
| OMID | Used RRT and refined the implementation for recurring search in the soccer scene. | [2019 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2019__OMID.pdf) |
| OP-Amp | Connected RRT waypoints with splines to obtain smoother motion. | [2019 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2019__OP-Amp.pdf) |
| Parsian | Adapted its extended RRT planner for a changing environment. | [2014 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2014__Parsian.pdf) |
| Plasma-Z | Summed target attraction and obstacle repulsion, grouping adjacent obstructions and shaping the force pattern to escape traps. | [2009 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2009__Plasma-Z.pdf) |
| RFC Cambridge | Checked the straight route first, limited RRT search radius and ran multiple short expansions per update. | [2013 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2013__RFC_Cambridge.pdf) |
| Ri-one | Scaled avoidance distances with robot speed and checked nearby robots before sending motion commands. | [2022 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2022__Ri-one.pdf) |
| RoboBulls | Queued clear straight segments around blockers and resolved close head-on encounters in a separate movement module. | [2015 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2015__RoboBulls.pdf) |
| RoboCupJunior SSL Team (Faabs subgroup) | The joint entry describes Faabs using tangent-based RRT-inspired routing and planning the approach behind the ball. This refers to the subgroup’s existing system. | [2026 TDP §4.2](https://ssl.robocup.org/wp-content/uploads/2026/03/2026_TDP_RoboCupJunior-SSL-Team.pdf) |
| RoboDragons | Added circular obstacle constraints to a model predictive trajectory tracker with speed and acceleration limits. | [2022 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2022__RoboDragons.pdf) |
| RoboFEI | Used A* over a discretized field populated from the observed game objects. | [2018 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2018__RoboFEI.pdf) |
| RoboIME | Used ERRT with a direct-line shortcut. Tested retaining and rewiring the tree between updates using RRT*-family techniques. | [2023 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2023__RoboIME.pdf) |
| RoboJackets | Used RRT in robot control and exposed the same planner in an interactive debugger. | [2014 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2014__RoboJackets.pdf) |
| RoboRoos | Combined a goal-attraction map with an obstacle-repulsion map in polar coordinates. The best heading drove the onboard motion controller. | [Team paper §6.2](https://citeseerx.ist.psu.edu/document?doi=c34d54524d615bc3008175fac9dbd1e27ea1351a&repid=rep1&type=pdf) |
| RoboTeam Twente | Used bang-bang trajectory segments, checked moving obstacles and searched intermediate trajectories when the direct route conflicted. | [2023 ETDP](https://ssl.robocup.org/wp-content/uploads/2023/02/2023_ETDP_RoboTeam_Twente.pdf) |
| RoboTurk | Used artificial potential fields and grouped obstacles to reduce oscillation and local trapping. | [2011 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2011__RoboTurk.pdf) |
| RobôCIn | Replaced its visibility graph with ER-Force bang-bang trajectory search and added TIGERs-style intermediate samples to explore around obstructions. | [2023 ETDP §3.3](https://arxiv.org/abs/2307.10018) |
| SeeGoals | Generated RRT waypoints and selected a forward waypoint to track with pure pursuit. | [2026 TDP §6.1](https://ssl.robocup.org/wp-content/uploads/2026/03/2026_TDP_SeeGoals.pdf) |
| Skuba | Used RRT as the default and a faster subgoal planner for fast moves, then generated velocity commands along the chosen route. | [2012 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2012__Skuba.pdf) |
| SRBL | Described an extended RRT navigation module that grows collision-checked branches toward the target. | [2025 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2025__SRBL.pdf) |
| SRC | Interpolated RRT waypoints with quasi-uniform B-splines for continuity. | [2022 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2022__SRC.pdf) |
| SSH | Built a graph of candidate points, connected neighbors and used Dijkstra to select a route. | [2015 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2015__SSH.pdf) |
| Stanford Robotics Club | Described individual RRT plans in task-priority order and an alternative simultaneous planning mode. | [2014 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2014__Stanford_Robotics_Club.pdf) |
| STOx’s | Tried a straight segment, recursively inserted lateral subgoals at blockers, then compared routes around opposite sides. | [2014 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2014__STOxs.pdf) |
| Sysmic Robotics | Replaced RRT with Fast Path Planning after sharp waypoint changes caused control problems. | [2025 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2025__Sysmic_Robotics.pdf) |
| The A-Team | Implemented the STOx recursive subgoal planner during competition. | [2024 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2024__The_A-Team.pdf) |
| The Bots | Implemented trajectory sampling based on TIGERs Mannheim's approach. | [2025 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2025__The_Bots.pdf) |
| TIGERs Mannheim | Generated candidate timed trajectories through intermediate positions and tested them against obstacles. This replaced the earlier RRT. | [2019 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2019__TIGERs_Mannheim.pdf) |
| TRAPS | Used A* for slow robots and restricted areas, with a local planner for moving obstacles and an immediate escape check. | [2025 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2025__TRAPS.pdf) |
| Tritons-RCSC | Adjusted obstacle footprints with speed to account for motion between updates. Its 2023 paper also reports prior use of Jump Point Search. | [2024 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2024__Tritons-RCSC.pdf) · [2023 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2023__Tritons-RCSC.pdf) |
| TurtleRabbit | Used a probabilistic roadmap in 2024. Its 2026 paper reports a switch to Voronoi-based navigation. | [2025 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2025__TurtleRabbit.pdf) · [2026 TDP](https://ssl.robocup.org/wp-content/uploads/2026/04/2026_TDP_TurtleRabbit.pdf) |
| UaiSoccer | Represented possible routes as a graph and used Dijkstra to compute the shortest path. | [2014 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2014__UaiSoccer.pdf) |
| UBC Thunderbots | Planned global routes with an edge-level sparse visibility graph (ENLSVG) and ran Hybrid Reciprocal Velocity Obstacles (HRVO) onboard. | [2024 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2024__UBC_Thunderbots.pdf) |
| ULtron | Combined A* pathfinding with artificial potential fields for collision avoidance and PID tracking. | [2017 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2017__ULtron.pdf) |
| UMass MinuteBots | Searched an eight-connected grid and applied Dynamic Safety Search to final commands. Also experimented with repairing future teammate path conflicts. | [2018 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2018__UMass_MinuteBots.pdf) |
| UNSAbots | Used A* with distance to the target as its heuristic. | [2014 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2014__UNSAbots.pdf) |
| URoboRus | Built paths between endpoints among circular obstacles and executed them through its MoveToAvoidance function. | [2020 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2020__URoboRus.pdf) |
| Warthog Robotics | Used potential fields and PID control. Evaluated Informed RRT* and a visibility graph as alternatives. | [2025 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2025__Warthog_Robotics.pdf) |
| ZJUNlict | Ran a global RRT, then computed trapezoidal velocity profiles through the resulting path points. | [2019 team paper](https://tdpsearch.com/pdfs/soccer_smallsize__2019__ZJUNlict.pdf) |

## Other entries screened

These historical index labels had a planner mention, a proposal, or insufficient method detail in the material screened. An omission here says nothing about what a team actually ran.

AMC, Anorak, Bochica, Compilation Error, Field Rangers, Fort Collins Football Club, GreenTea, ICRS-FC, INPUT, Khainui, OrcaBOT, Owaribito-CU, Project Neon, RoboFighties, RoboPET, SPbUnited, TritonBots FC.

For example, ICRS-FC proposed RRT*, and RoboPET proposed RRT plus Dynamic Safety Search. GreenTea describes a planner/follower interface. INPUT uses a potential field to choose pass destinations, a different problem. The 2026 papers for NBT, NRG and RoboUnited also leave the routing algorithm unresolved in this survey. [ICRS-FC](https://tdpsearch.com/pdfs/soccer_smallsize__2024__ICRS-FC.pdf) · [RoboPET](https://tdpsearch.com/pdfs/soccer_smallsize__2010__RoboPET.pdf) · [GreenTea](https://tdpsearch.com/pdfs/soccer_smallsize__2023__GreenTea.pdf) · [INPUT](https://tdpsearch.com/pdfs/soccer_smallsize__2022__INPUT.pdf)

Earlier 5dpo documentation describes planning for robots and the ball and executing queued trajectories onboard. The search method is unspecified. [5dpo team paper](https://www.ida.liu.se/ext/robocup/small/5dpo/paper.pso.pdf)
