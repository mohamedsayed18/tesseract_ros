# Puzzle Piece auxiliary axes example

The puzzle piece examples show a small collaborative robot manipulating a puzzle piece to debur the edges with a grinder which has two axes of rotation.

![Puzzle piece](https://github.com/ros-industrial-consortium/trajopt_ros/blob/master/gh_pages/_static/example_gifs/puzzle_piece_with_positioner.gif)

```cpp
tesseract_common::VectorIsometry3d tool_poses = makePuzzleToolPoses();
```

`makePuzzleToolPoses`: reads the target poses of the puzzle piece from the
`/config/puzzle_bent.csv` and return a list of the target poses.

```cpp
// Create manipulator information for program
ManipulatorInfo mi;
mi.manipulator = "manipulator_aux";
mi.working_frame = "part";
mi.tcp = ToolCenterPoint("grinder_frame", true);  // true - indicates this is an external TCP
```

We define the manipulator information for our problem, 
"manipulator_aux": This is the name of the group as defined in the srdf file
"working_frame": Is the reference frame the points will be refereed to it.
"tcp": Is the tool center point we want to plan our motion on it.

```cpp
// Create cartesian waypoint
Waypoint wp = CartesianWaypoint(tool_poses[0]);
PlanInstruction plan_instruction(wp, PlanInstructionType::START, "CARTESIAN");
plan_instruction.setDescription("from_start_plan");
program.setStartInstruction(plan_instruction);
```

```cpp
// Create Program
CompositeInstruction program("DEFAULT", CompositeInstructionOrder::ORDERED, mi);
```

We create the program, we define its the order of waypoints execution. It can be one of the following:

* ORDERED: Must go in forward
* UNORDERED: Any order is allowed
* ORDERED_AND_REVERABLE: Can go forward or reverse the order

The last parameter is the manipulator we created above `mi`

Now, Let's add the instructions to the program.
The first pose is set as the start poses.

```cpp
// Create cartesian waypoint
Waypoint wp = CartesianWaypoint(tool_poses[0]);
PlanInstruction plan_instruction(wp, PlanInstructionType::START, "CARTESIAN");
plan_instruction.setDescription("from_start_plan");
program.setStartInstruction(plan_instruction);
```

We create a waypoint then make this waypoint an instruction and add it to the program.

To create a waypoint we have many ways to do so. One of them is using the `CartesianWaypoint`

Then we decide how to move to this waypoint, We having the following options:

* LINEAR: Straight line motion
* FREESPACE: Move to the point freely
* CIRCULAR: Circular motion to the point
* START: This point is the start point of your program

The `plan_instruction` takes the waypoint, the type of instruction and string which acts as a label for the instruction. We can also add a description to the instruction using `setDescription`.

For the start instruction we use the `setStartInstruction`

```cpp
for (std::size_t i = 1; i < tool_poses.size(); ++i)
{
    Waypoint wp = CartesianWaypoint(tool_poses[i]);
    PlanInstruction plan_instruction(wp, PlanInstructionType::LINEAR, "CARTESIAN");
    plan_instruction.setDescription("waypoint_" + std::to_string(i));
    program.push_back(plan_instruction);
}
```

A linear motion is set between the rest of the points.

```cpp
// Create Process Planning Server
ProcessPlanningServer planning_server(std::make_shared<ROSProcessEnvironmentCache>(monitor_), 5);
planning_server.loadDefaultProcessPlanners();
```

We create a server object which will take the program as request and return the response.
The "loadDefaultProcessPlanners" method load the default process planners, But we can load specific plannners.

```cpp
const std::string new_planner_name = "TRAJOPT_NO_POST_CHECK";
tesseract_planning::TrajOptTaskflowParams params;
params.enable_post_contact_discrete_check = false;
params.enable_post_contact_continuous_check = false;
planning_server.registerProcessPlanner(new_planner_name,
                                        std::make_unique<tesseract_planning::TrajOptTaskflow>(params));
```

A trajopt taskflow is used without collision checking by setting
`enable_post_contact_discrete_check` & `enable_post_contact_continuous_check` to `false`.

Next we will create TrajOpt Profiles.

```cpp
// Create TrajOpt Profile
auto trajopt_plan_profile = std::make_shared<tesseract_planning::TrajOptDefaultPlanProfile>();
trajopt_plan_profile->cartesian_coeff = Eigen::VectorXd::Constant(6, 1, 5);
trajopt_plan_profile->cartesian_coeff(3) = 2;
trajopt_plan_profile->cartesian_coeff(4) = 2;
trajopt_plan_profile->cartesian_coeff(5) = 0;
```

In the `TrajOptDefaultPlanProfile`, the `cartesian_coeff` is a vector of size six.
The first three correspond to translation and the last three to rotation.
The `trajopt_plan_profile->cartesian_coeff(5) = 0;` indicates it is free to rotate around the z-axis.
If the value is greater than zero it is considered constrained and the coefficient represents a
weight/scale applied to the error and gradient.

```cpp
auto trajopt_composite_profile = std::make_shared<tesseract_planning::TrajOptDefaultCompositeProfile>();
trajopt_composite_profile->collision_constraint_config.enabled = false;
trajopt_composite_profile->collision_cost_config.enabled = true;
trajopt_composite_profile->collision_cost_config.safety_margin = 0.025;
trajopt_composite_profile->collision_cost_config.type = trajopt::CollisionEvaluatorType::SINGLE_TIMESTEP;
trajopt_composite_profile->collision_cost_config.coeff = 1;
```

In the `TrajOptDefaultCompositeProfile`, we can set the following parameters:

`collision_constraint_config`: to decide to apply constraints on the collision

`collision_cost_config`: to give costs for the collision.

`collision_cost_config.safety_margin`: Define the safety margin for collision

`collision_cost_config.type`: It can take the following values:

* SINGLE_TIMESTEP: TODO, description.
* DISCRETE_CONTINUOUS: TODO, description.
* CAST_CONTINUOUS: TODO, description.

```cpp
auto trajopt_solver_profile = std::make_shared<tesseract_planning::TrajOptDefaultSolverProfile>();
trajopt_solver_profile->convex_solver = sco::ModelType::OSQP;
trajopt_solver_profile->opt_info.max_iter = 200;
trajopt_solver_profile->opt_info.min_approx_improve = 1e-3;
trajopt_solver_profile->opt_info.min_trust_box_size = 1e-3;
```

In `TrajOptDefaultSolverProfile` we specify the type of the convex solver to use it can be:
GUROBI, OSQP, QPOASES, BPMPD, AUTO_SOLVER

`max_iter`: the maximum number of iterations to find a solution.

`min_approx_improve`: the minimum improvement that for the solution after each iteration.

`min_trust_box_size`: TODO, describe

```cpp
// Add profile to Dictionary
planning_server.getProfiles()->addProfile<tesseract_planning::TrajOptPlanProfile>("CARTESIAN",          trajopt_plan_profile);
planning_server.getProfiles()->addProfile<tesseract_planning::TrajOptCompositeProfile>("DEFAULT",
                                                                                        trajopt_composite_profile);
planning_server.getProfiles()->addProfile<tesseract_planning::TrajOptSolverProfile>("DEFAULT",
                                                                                    trajopt_solver_profile);
```

We add the profiles created above to our planning server.

```cpp
// Create Process Planning Request
ProcessPlanningRequest request;
request.name = new_planner_name;
request.instructions = Instruction(program);
```

We create a request object and add the program to its instruction.

```cpp
ProcessPlanningFuture response = planning_server.run(request);
```

we use the run method providing the request as a parameter to get the response.
