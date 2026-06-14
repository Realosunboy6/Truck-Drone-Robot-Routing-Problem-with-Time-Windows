# Strict Math Model with Ordered Multi-Customer Sortie Improvement

This file documents `drone_robot_truck_team_model_ordered_sorties_strict_math_improved.py`.

## Paper Parameter Scenario
This runner uses `data_processed/tdrp_tw_literature_params/<case>`, which stores the VRP-DR paper Table 3 values: truck speed 45 km/h, drone speed 75 km/h, robot speed 25 km/h, drone payload 25 kg, robot payload 20 kg, drone distance limit 20 km, and robot distance limit 15 km.
To keep the MILP solvable, ordered sortie generation is capped at 3 customers per sortie and the best 10 routes per launch/recovery node pair.

## Main Correction
The model follows the math-model decision structure: truck arc variables `x`, drone sortie variables `y`, and robot sortie variables `z` decide the platform assignment.
Customer assignment is not pre-filtered by eligibility columns.
Assignment is determined by `truck_service[j] + drone_service[j] + robot_service[j] == 1`, while capacity, endurance, synchronization, and timing constraints control feasibility.

## Ordered-Route Improvement
The drone and robot variables use complete ordered sortie paths instead of single-customer sortie variables.
A selected sortie can represent a path such as `6 -> 3 -> 8 -> 4`, so the route sequence is directly recoverable from the decision variable.

## Retained Model Structure
- Customer service exactly once.
- Truck, drone, and robot capacity constraints (hard).
- Drone and robot endurance enforced per sortie (hard): candidate generation excludes routes beyond the distance limit, and per-sortie constraints restate it in the MILP.
- Each physical drone or robot performs at most one sortie, so no inter-sortie sequencing or onboard platform tracking is required. Fleet sizes in the processed cases exceed the number of useful sorties, so this does not restrict the solution space.
- Flexible truck-platform synchronization for drone/robot launch and recovery.
- Drone launch variable `h_launch[v,d,s]` identifies which truck launches each selected drone sortie.
- Drone recovery variable `h_recover[v,d,s]` identifies which truck recovers each selected drone sortie.
- Distinct drones interacting with each truck are limited by `MAX_DRONES_PER_TRUCK` using aggregate truck-drone pairing constraints.
- Drone launches from the depot are limited by `DRONES_CARRIED_AT_DEPOT`; launches from any truck stop are limited by `MAX_DRONES_PER_TRUCK`.
- Drone recoveries at truck stops and the end depot are limited by `MAX_DRONES_PER_TRUCK`.
- Robot launch variable `g_launch[v,r,s]` identifies which truck launches each selected robot sortie.
- Robot recovery variable `g_recover[v,r,s]` identifies which truck recovers each selected robot sortie.
- Distinct robots interacting with each truck are limited by `MAX_ROBOTS_PER_TRUCK` using aggregate truck-robot pairing constraints.
- Robot launches from the depot are limited by `ROBOTS_CARRIED_AT_DEPOT`; launches from any truck stop are limited by `MAX_ROBOTS_PER_TRUCK`.
- Robot recoveries at truck stops and the end depot are limited by `MAX_ROBOTS_PER_TRUCK`.
- Ordered sortie timing linked to truck launch and recovery times.
- Time-window penalty constraints (soft).
- Route-duration penalty (soft): truck depot departure is fixed at time 0, and each truck's end-depot arrival may exceed `T_max` only at a penalty. The global excess equals the maximum per-truck excess.
- Operating cost = variable travel cost + fixed activation costs (truck 30 per used truck, drone 10 per selected sortie, robot 8 per selected sortie; Malik et al., VRP-DR, arXiv:2505.23584, Table 3). Fixed costs are written in parameters.json as truck_fixed_cost / drone_fixed_cost / robot_fixed_cost; the same values remain model defaults as a safeguard.
- Operating cost plus the two soft penalties form the objective. Capacity and endurance are hard constraints with no penalty terms.
- Symmetry breaking: platform k+1 may fly a sortie only if platform k does, valid because platforms are identical and limited to one sortie each.
- Lossless fleet trimming: the model instantiates at most MAX_DRONES_PER_TRUCK x NUM_TRUCKS drones and MAX_ROBOTS_PER_TRUCK x NUM_TRUCKS robots, because the distinct-platform caps make any additional identical platforms provably unusable. This shrinks the MILP without changing the solution space.
- Optional truck-only baseline: setting DRT_TRUCK_ONLY=1 fixes all drone and robot sortie variables to zero.
- Makespan is computed after solving from the reported truck and platform finish times. It is not a decision variable and is not part of the objective, matching the PDF formulation (Z plus route-duration and time-window penalty terms).
- The `Truck Physical Timing` sheet compares each model arrival time with the earliest physically required time (travel chain, time windows, platform recovery), making any remaining harmless slack in arrival variables explicit.

## Added Operational Routing Completeness Constraints
- Binary `used_truck[v]`.
- Depot departure equals `used_truck[v]`.
- Depot return equals `used_truck[v]`.
- Truck flow conservation at customer nodes.
- Binary `visit[v,i]` linked to inbound truck arcs.
- Truck service relation `truck_service[i] == sum_v visit[v,i]`.
- Truck-use activation constraints.
- Used trucks must serve at least one customer.
- Truck-indexed timing propagation includes truck service at the departure customer before traveling to the next node.
- Truck time-window constraints linked to visits.
- Truck route-duration diagnostics per truck.
- MTZ subtour-elimination constraints for customer-customer arcs.

## Ordered Sortie Generation
- Each feasible drone/robot sortie is an ordered customer sequence between a launch node and a recovery node.
- Candidate generation keeps at most 10 high-coverage, low-cost ordered routes per launch/recovery node pair.
- A sequence is generated only if its total demand is within platform capacity.
- A sequence is generated only if its full route distance is within platform endurance.
- The full route cost and duration use every consecutive arc in the ordered path.
- The selected route timing propagates through the ordered customer sequence and includes customer service time.
- Output sheets distinguish precomputed travel duration from service duration, active duration, elapsed schedule time, waiting time, and recovery slack.
- Output sheets also report selected sortie travel distance by summing the platform-specific distance on every route leg.

## Synchronization Interpretation
The model uses flexible truck-platform synchronization: an ordered sortie from `i` to `k` has one launch truck and one recovery truck.
If `h_launch[v,d,s] = 1`, drone `d` may launch only after truck `v` arrives at launch node `i`.
If `h_recover[v,d,s] = 1`, drone `d` must recover no later than truck `v` arrives at recovery node `k`.
The same interpretation applies to robot variables `g_launch[v,r,s]` and `g_recover[v,r,s]`.

## Big-M
Chosen M = `10000.0`.

## Strict Model Baseline
- Objective: 6.53333333336
- Truck arcs: 10
- Drone sorties: 0
- Robot sorties: 0
- Truck-served customers: 8

## Ordered-Sortie Result
- Solve status: time limit exceeded
- Objective: 3024.4777778006564
- Truck arcs: 9
- Drone sorties: 0
- Robot sorties: 0
- Feasible drone sequences generated: 730
- Feasible robot sequences generated: 730
- Reported makespan: 12.927777777800657
- Route duration excess: 2.9277777778006566
- Total penalty: 2927.7777778006566

## Professor's Route-Confirmation Concern
The selected sortie sheet prints the complete ordered path, so a route like `6 -> 3 -> 8 -> 4` is confirmed by one selected variable rather than inferred from multiple independent single-customer sorties.
