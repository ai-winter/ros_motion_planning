/*
 * RVO.h
 * RVO2 Library
 *
 * SPDX-FileCopyrightText: 2008 University of North Carolina at Chapel Hill
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <https://gamma.cs.unc.edu/RVO2/>
 */

#ifndef RVO_RVO_H_
#define RVO_RVO_H_

/**
 * @file  RVO.h
 * @brief Includes all public headers in the library.
 */

/**
 * @namespace RVO
 * @brief     Contains all classes, functions, and constants used in the
 *            library.
 */

/* IWYU pragma: begin_exports */
// #include "Export.h"
#include "Line.h"
#include "RVOSimulator.h"
#include "Vector2.h"
/* IWYU pragma: end_exports */

/**

@mainpage  RVO2 Library Documentation
@author    Jur van den Berg
@author    <a href="https://www-users.cs.umn.edu/~sjguy/">Stephen J. Guy</a>
@author    <a href="https://www.jamiesnape.io/">Jamie Snape</a>
@author    <a href="https://www.cs.umd.edu/people/lin/">Ming C. Lin</a>
@author    <a href="https://www.cs.umd.edu/people/dmanocha/">Dinesh Manocha</a>
@copyright 2008 <a href="https://www.unc.edu/">University of North Carolina at
           Chapel Hill</a>

We present a formal approach to reciprocal collision avoidance, where multiple
independent mobile robots or agents need to avoid collisions with each other
without communication among agents while moving in a common workspace. Our
formulation, <a href="https://gamma.cs.unc.edu/ORCA/">optimal reciprocal
collision avoidance</a> (ORCA), provides sufficient conditions for
collision-free motion by letting each agent take half of the responsibility of
avoiding pairwise collisions. Selecting the optimal action for each agent is
reduced to solving a low-dimensional linear program, and we prove that the
resulting motions are smooth. We test our optimal reciprocal collision avoidance
approach on several dense and complex simulation scenarios workspaces involving
thousands of agents, and compute collision-free actions for all of them in only
a few milliseconds.

<b>RVO2 Library</b> is an
<a href="https://www.apache.org/licenses/LICENSE-2.0">open-source</a> C++98
implementation of our algorithm in two dimensions. It has a simple API for
third-party applications. The user specifies static obstacles, agents, and the
preferred velocities of the agents. The simulation is performed step-by-step via
a simple call to the library. The simulation is fully accessible and manipulable
during runtime. The library exploits multiple processors if they are available
using <a href="https://www.openmp.org/">OpenMP</a> for efficient parallelization
of the simulation.

The creation of <b>RVO2 Library</b> was supported by the
<a href="https://www.arl.army.mil/who-we-are/directorates/aro/">United States
Army Research Office</a> (ARO) under Contract W911NF‑04‑1‑0088, by the
<a href="https://www.nsf.gov">National Science Foundation</a> (NSF) under Award
0636208, Award 0917040, and Award 0904990, by the
<a href="https://www.darpa.mil">Defense Advanced Research Projects Agency</a>
(DARPA) and the United States Army Research, Development, and Engineering
Command (RDECOM) under Contract WR91CRB‑08‑C‑0137, and by
<a href="https://www.intel.com/">Intel Corporation</a>. Any opinions, findings,
and conclusions or recommendations expressed herein are those of the authors and
do not necessarily reflect the views of ARO, NSF, DARPA, RDECOM, or Intel.

Please follow the following steps to install and use <b>RVO2 Library</b>.

@li @subpage what_is_new_in_rvo2_library
@li @subpage building_rvo2_library
@li @subpage using_rvo2_library
@li @subpage parameter_overview

See the documentation of the RVO::RVOSimulator class for an exhaustive list of
public functions of <b>RVO2 Library</b>.

<b>RVO2 Library</b>, accompanying example code, and this documentation is
released under the following @subpage terms_and_conditions
"terms and conditions".

@page what_is_new_in_rvo2_library What Is New in RVO2 Library

@section local_collision_avoidance Local Collision Avoidance

The main difference between <b>RVO2 Library</b> and %RVO Library 1.x is the
local collision avoidance technique used. <b>RVO2 Library</b> uses <a
href="http://gamma.cs.unc.edu/ORCA/">optimal reciprocal collision avoidance</a>
(ORCA), whereas %RVO Library 1.x uses <a href="http://gamma.cs.unc.edu/RVO/">
reciprocal velocity obstacles</a> (%RVO). For legacy reasons, and since both
techniques are based on the same principles of reciprocal collision avoidance
and relative velocity, we did not change the name of the library.

A major consequence of the change of local collision avoidance technique is that
the simulation has become much faster in <b>RVO2 Library</b>. optimal reciprocal
collision avoidance defines velocity constraints with respect to other agents as
half-planes, and an optimal velocity is efficiently found using two-dimensional
linear programming. In contrast, %RVO Library 1.x uses random sampling to find a
good velocity. Also, the behavior of the agents is smoother in <b>RVO2
Library</b>. It is proven mathematically that optimal reciprocal collision
avoidance lets the velocity of agents evolve continuously over time, whereas
%RVO Library 1.x occasionally showed oscillations and reciprocal dances.
Furthermore, optimal reciprocal collision avoidance provides stronger guarantees
with respect to collision avoidance.

@section global_path_planning Global Path Planning

Local collision avoidance as provided by <b>RVO2 Library</b> should in principle
be accompanied by global path planning that determines the preferred velocity of
each agent in each time step of the simulation. %RVO Library 1.x has a built-in
roadmap infrastructure to guide agents around obstacles to fixed goals. However,
besides roadmaps, other techniques for global planning, such as navigation
fields, cell decompositions, etc. exist. Therefore, <b>RVO2 Library</b> does not
provide global planning infrastructure anymore. Instead, it is the
responsibility of the external application to set the preferred velocity of each
agent ahead of each time step of the simulation. This makes the library more
flexible to use in varying application domains. In one of the example
applications that comes with <b>RVO2 Library</b>, we show how a roadmap similar
to %RVO Library 1.x is used externally to guide the global navigation of the
agents. As a consequence of this change, <b>RVO2 Library</b> does not have a
concept of a &quot;goal position&quot; or &quot;preferred speed&quot; for each
agent, but only relies on the preferred velocities of the agents set by the
external application.

@section structure_of_rvo2_library Structure of RVO2 Library

The structure of <b>RVO2 Library</b> is similar to that of %RVO Library 1.x.
Users familiar with %RVO Library 1.x should find little trouble in using <b>RVO2
Library</b>. However, <b>RVO2 Library</b> is not backwards compatible with %RVO
Library 1.x. The main reason for this is that the optimal reciprocal collision
avoidance technique requires different (and fewer) parameters to be set than
%RVO. Also, the way obstacles are represented is different. In %RVO Library 1.x,
obstacles are represented by an arbitrary collection of line segments. In
<b>RVO2 Library</b>, obstacles are non-intersecting polygons, specified by lists
of vertices in counterclockwise order. Further, in %RVO Library 1.x agents
cannot be added to the simulation after the simulation is initialized. In
<b>RVO2 Library</b> this restriction is removed. Obstacles still need to be
processed before the simulation starts, though. Lastly, in %RVO Library 1.x an
instance of the simulator is a singleton. This restriction is removed in <b>RVO2
Library</b>.

@section smaller_changes Smaller Changes

With <b>RVO2 Library</b>, we have adopted the philosophy that anything that is
not part of the core local collision avoidance technique is to be stripped from
the library. Therefore, besides the roadmap infrastructure, we have also removed
acceleration constraints of agents, orientation of agents, and the unused
&quot;class&quot; of agents. Each of these can be implemented external of the
library if needed. We did maintain a k-D tree infrastructure for efficiently
finding other agents and obstacles nearby each agent.

Also, <b>RVO2 Library</b> allows accessing information about the simulation,
such as the neighbors and the collision-avoidance constraints of each agent,
that is hidden from the user in %RVO Library 1.x.

@page building_rvo2_library Building RVO2 Library

@section cmake CMake

@code{.sh}
git clone https://github.com/snape/RVO2.git
cd RVO2
mkdir _build
cmake -B _build -S . --install-prefix /path/to/install
cmake --build _build
cmake --install _build
ctest _build
@endcode

@section bazel Bazel

@code{.sh}
git clone https://github.com/snape/RVO2.git
cd RVO2
bazel test ...
@endcode

@page using_rvo2_library Using RVO2 Library

@section structure Structure

A program performing an <b>RVO2 Library</b> simulation has the following global
structure.

@code{.cc}
#include <RVO.h>

#include <vector>

std::vector<RVO::Vector2> goals;

int main() {
  // Create a new simulator instance.
  RVO::RVOSimulator *simulator = new RVO::RVOSimulator();

  // Set up the scenario.
 setupScenario(simulator);

  // Perform (and manipulate) the simulation.
  do {
    updateVisualization(simulator);
    setPreferredVelocities(simulator);
    simulator->doStep();
  } while (!reachedGoal(simulator));

  delete simulator;
}
@endcode

In order to use <b>RVO2 Library</b>, the user needs to include RVO.h. The first
step is then to create an instance of RVO::RVOSimulator. Then, the process
consists of two stages. The first stage is specifying the simulation scenario
and its parameters. In the above example program, this is done in the method
setupScenario(...), which we will discuss below. The second stage is the actual
performing of the simulation.

In the above example program, simulation steps are taken until all the agents
have reached some predefined goals. Prior to each simulation step, we set the
preferred velocity for each agent, i.e., the velocity the agent would have taken
if there were no other agents around, in the method setPreferredVelocities(...).
The simulator computes the actual velocities of the agents and attempts to
follow the preferred velocities as closely as possible while guaranteeing
collision avoidance at the same time. During the simulation, the user may want
to retrieve information from the simulation for instance to visualize the
simulation. In the above example program, this is done in the method
updateVisualization(...), which we will discuss below. It is also possible to
manipulate the simulation during the simulation, for instance by changing
positions, radii, velocities, etc. of the agents.

@section setting_up_the_simulation_scenario Setting up the Simulation Scenario

A scenario that is to be simulated can be set up as follows. A scenario consists
of two types of objects: agents and obstacles. Each of them can be manually
specified. Agents may be added anytime before or during the simulation.
Obstacles, however, need to be defined prior to the simulation, and
RVO::RVOSimulator::processObstacles() need to be called in order for the
obstacles to be accounted for in the simulation. The user may also want to
define goal positions of the agents, or a roadmap to guide the agents around
obstacles. This is not done in <b>RVO2 Library</b>, but needs to be taken care
of in the user's external application.

The following example creates a scenario with four agents exchanging positions
around a rectangular obstacle in the middle.

@code{.cc}
#include <RVO.h>

#include <cstddef>

namespace {
void setupScenario(RVO::RVOSimulator *simulator) {
  // Specify global time step of the simulation.
  simulator->setTimeStep(0.25F);

  // Specify default parameters for agents that are subsequently added.
  simulator->setAgentDefaults(15.0F, 10U, 10.0F, 5.0F, 2.0F, 2.0F);

  // Add agents, specifying their start position.
  simulator->addAgent(RVO::Vector2(-50.0F, -50.0F));
  simulator->addAgent(RVO::Vector2(50.0F, -50.0F));
  simulator->addAgent(RVO::Vector2(50.0F, 50.0F));
  simulator->addAgent(RVO::Vector2(-50.0F, 50.0F));

  // Create goals. The simulator is unaware of these.
  for (std::size_t i = 0U; i < simulator->getNumAgents(); ++i) {
    goals.push_back(-simulator->getAgentPosition(i));
  }

  // Add polygonal obstacle(s), specifying vertices in counterclockwise order.
  std::vector<RVO::Vector2> vertices;
  vertices.push_back(RVO::Vector2(-7.0F, -20.0F));
  vertices.push_back(RVO::Vector2(7.0F, -20.0F));
  vertices.push_back(RVO::Vector2(7.0F, 20.0F));
  vertices.push_back(RVO::Vector2(-7.0F, 20.0F));

  simulator->addObstacle(vertices);

  // Process obstacles so that they are accounted for in the simulation.
  simulator->processObstacles();
}
}  // namespace
@endcode

See the documentation on RVO::RVOSimulator for a full overview of the
functionality to specify scenarios.

@section retrieving_information_from_the_simulation Retrieving Information from
                                                    the Simulation

During the simulation, the user can extract information from the simulation for
instance for visualization purposes, or to determine termination conditions of
the simulation. In the example program above, visualization is done in the
updateVisualization(...) method. Below we give an example that simply writes the
positions of each agent in each time step to the standard output. The
termination condition is checked in the reachedGoal(...) method. Here we give an
example that returns true if all agents are within one radius of their goals.

@code{.cc}
#include <RVO.h>

#include <cstddef>
#include <iostream>

namespace {
void updateVisualization(RVO::RVOSimulator *simulator) {
  // Output the current global time.
  std::cout << simulator->getGlobalTime() << " ";

  // Output the position for all the agents.
  for (std::size_t i = 0U; i < simulator->getNumAgents(); ++i) {
    std::cout << simulator->getAgentPosition(i) << " ";
  }

  std::cout << std::endl;
}
}  // namespace
@endcode

@code{.cc}
#include <RVO.h>

#include <cstddef>

bool reachedGoal(RVO::RVOSimulator *simulator) {
  // Check whether all agents have arrived at their goals.
  for (std::size_t i = 0U; i < simulator->getNumAgents(); ++i) {
    if (absSq(goals[i] - simulator->getAgentPosition(i)) >
        simulator->getAgentRadius(i) * simulator->getAgentRadius(i)) {
      // Agent is further away from its goal than one radius.
      return false;
    }
  }

  return true;
}
@endcode

Using similar functions as the ones used in this example, the user can access
information about other parameters of the agents, as well as the global
parameters, and the obstacles. See the documentation of the class
RVO::RVOSimulator for an exhaustive list of public functions for retrieving
simulation information.

@section manipulating_the_simulation Manipulating the Simulation

During the simulation, the user can manipulate the simulation, for instance by
changing the global parameters, or changing the parameters of the agents
(potentially causing abrupt different behavior). It is also possible to give the
agents a new position, which make them jump through the scene. New agents can be
added to the simulation at any time, but it is not allowed to add obstacles to
the simulation after they have been processed by calling
RVO::RVOSimulator::processObstacles(). Also, it is impossible to change the
position of the vertices of the obstacles.

See the documentation of the class RVO::RVOSimulator for an exhaustive list of
public functions for manipulating the simulation.

To provide global guidance to the agents, the preferred velocities of the agents
can be changed ahead of each simulation step. In the above example program, this
happens in the method setPreferredVelocities(...). Here we give an example that
simply sets the preferred velocity to the unit vector towards the agent's goal
for each agent (i.e., the preferred speed is 1.0). Note that this may not give
convincing results with respect to global navigation around the obstacles. For
this a roadmap or other global planning techniques may be used (see one of the
@ref example_programs "example programs" that accompanies <b>RVO2 Library</b>).

@code{.cc}
#include <RVO.h>

#include <cstddef>

namespace {
void setPreferredVelocities(RVO::RVOSimulator *simulator) {
  // Set the preferred velocity for each agent.
  for (std::size_t i = 0U; i < simulator->getNumAgents(); ++i) {
    if (absSq(goals[i] - simulator->getAgentPosition(i)) <
        simulator->getAgentRadius(i) * simulator->getAgentRadius(i) ) {
      // Agent is within one radius of its goal, set preferred velocity to zero.
      simulator->setAgentPrefVelocity(i, RVO::Vector2(0.0F, 0.0F));
    } else {
      // Agent is far away from its goal, set preferred velocity as unit vector
      // towards agent's goal.
      simulator->setAgentPrefVelocity(i,
                                normalize(goals[i] -
simulator->getAgentPosition(i)));
    }
  }
}
}  // namespace
@endcode

@section example_programs Example Programs

<b>RVO2 Library</b> is accompanied by three example programs, which can be found
in the <tt>$RVO_ROOT/examples</tt> directory. The examples are named Blocks,
Circle, and Roadmap, and contain the following demonstration scenarios:

<table border="0" cellpadding="3" width="100%">
<tr>
<td valign="top" width="100"><b>Blocks</b></td>
<td valign="top">A scenario in which 100 agents, split in four groups initially
positioned in each of four corners of the environment, move to the other side of
the environment through a narrow passage generated by four obstacles. There is
no roadmap to guide the agents around the obstacles.</td>
</tr>
<tr>
<td valign="top" width="100"><b>Circle</b></td>
<td valign="top">A scenario in which 250 agents, initially positioned evenly
distributed on a circle, move to the antipodal position on the circle. There are
no obstacles.</td>
</tr>
<tr>
<td valign="top" width="100"><b>Roadmap</b></td>
<td valign="top">The same scenario as <b>Blocks</b>, but now the preferred
velocities of the agents are determined using a roadmap guiding the agents
around the obstacles.</td>
</tr>
</table>

@page parameter_overview Parameter Overview

@section global_parameters Global Parameters

<table border="0" cellpadding="3" width="100%">
<tr>
<td valign="top" width="150"><strong>Parameter</strong></td>
<td valign="top" width="150"><strong>Type (unit)</strong></td>
<td valign="top"><strong>Meaning</strong></td>
</tr>
<tr>
<td valign="top">timeStep</td>
<td valign="top">float (time)</td>
<td valign="top">The time step of the simulation. Must be positive.</td>
</tr>
</table>

@section agent_parameters Agent Parameters

<table border="0" cellpadding="3" width="100%">
<tr>
<td valign="top" width="150"><strong>Parameter</strong></td>
<td valign="top" width="150"><strong>Type (unit)</strong></td>
<td valign="top"><strong>Meaning</strong></td>
</tr>
<tr>
<td valign="top">maxNeighbors</td>
<td valign="top">std::size_t</td>
<td valign="top">The maximum number of other agents the agent takes into account
in the navigation. The larger this number, the longer the running time of the
simulation. If the number is too low, the simulation will not be safe.</td>
</tr>
<tr>
<td valign="top">maxSpeed</td>
<td valign="top">float (distance/time)</td>
<td valign="top">The maximum speed of the agent. Must be non-negative.</td>
</tr>
<tr>
<td valign="top">neighborDist</td>
<td valign="top">float (distance)</td>
<td valign="top">The maximum distance center-point to center-point to other
agents the agent takes into account in the navigation. The larger this number,
the longer the running time of the simulation. If the number is too low, the
simulation will not be safe. Must be non-negative.</td>
</tr>
<tr>
<td valign="top" width="150">position</td>
<td valign="top" width="150">RVO::Vector2 (distance, distance)</td>
<td valign="top">The current position of the agent.</td>
</tr>
<tr>
<td valign="top" width="150">prefVelocity</td>
<td valign="top" width="150">RVO::Vector2 (distance/time, distance/time)</td>
<td valign="top">The current preferred velocity of the agent. This is the
velocity the agent would take if no other agents or obstacles were around. The
simulator computes an actual velocity for the agent that follows the preferred
velocity as closely as possible, but at the same time guarantees collision
avoidance.</td>
</tr>
<tr>
<td valign="top">radius</td>
<td valign="top">float (distance)</td>
<td valign="top">The radius of the agent. Must be non-negative.</td>
</tr>
<tr>
<td valign="top" width="150">timeHorizon</td>
<td valign="top" width="150">float (time)</td>
<td valign="top">The minimal amount of time for which the agent's velocities
that are computed by the simulation are safe with respect to other agents. The
larger this number, the sooner this agent will respond to the presence of other
agents, but the less freedom the agent has in choosing its velocities. Must be
positive.</td>
</tr>
<tr>
<td valign="top">timeHorizonObst</td>
<td valign="top">float (time)</td>
<td valign="top">The minimal amount of time for which the agent's velocities
that are computed by the simulation are safe with respect to obstacles. The
larger this number, the sooner this agent will respond to the presence of
obstacles, but the less freedom the agent has in choosing its velocities. Must
be positive.</td>
</tr>
<tr>
<td valign="top" width="150">velocity</td>
<td valign="top" width="150">RVO::Vector2 (distance/time, distance/time)</td>
<td valign="top">The (current) velocity of the agent.</td>
</tr>
</table>

@page terms_and_conditions Terms and Conditions

@section source_code_license Source Code License

Source code is licensed under the
<a href="https://www.apache.org/licenses/LICENSE-2.0">Apache License,
Version 2.0</a>.

@verbatim
RVO2 Library

SPDX-FileCopyrightText: 2008 University of North Carolina at Chapel Hill
SPDX-License-Identifier: Apache-2.0

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    https://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
@endverbatim

@section documentation_license Documentation Licence

Documentation is licensed under the
<a href="https://creativecommons.org/licenses/by-sa/4.0/">Creative Commons
Attribution-ShareAlike 4.0 International (CC-BY-SA-4.0) Public License</a>.

@verbatim
RVO2 Library

SPDX-FileCopyrightText: 2008 University of North Carolina at Chapel Hill
SPDX-License-Identifier: CC-BY-SA-4.0

Creative Commons Attribution-ShareAlike 4.0 International Public License

You are free to:

* Share -- copy and redistribute the material in any medium or format

* ShareAlike -- If you remix, transform, or build upon the material, you must
  distribute your contributions under the same license as the original

* Adapt -- remix, transform, and build upon the material for any purpose, even
  commercially.

The licensor cannot revoke these freedoms as long as you follow the license
terms.

Under the following terms:

* Attribution -- You must give appropriate credit, provide a link to the
  license, and indicate if changes were made. You may do so in any reasonable
  manner, but not in any way that suggests the licensor endorses you or your
  use.

* No additional restrictions -- You may not apply legal terms or technological
  measures that legally restrict others from doing anything the license
  permits.

Notices:

* You do not have to comply with the license for elements of the material in
  the public domain or where your use is permitted by an applicable exception
  or limitation.

* No warranties are given. The license may not give you all of the permissions
  necessary for your intended use. For example, other rights such as publicity,
  privacy, or moral rights may limit how you use the material.
@endverbatim

@section other_licenses Other Licenses

Selected files are licensed under the
<a href="https://creativecommons.org/publicdomain/zero/1.0/">CC0 1.0 Universal
(CC0 1.0) Public Domain Dedication</a> or the
<a href="https://creativecommons.org/licenses/by/4.0/">Creative Commons
Attribution 4.0 International (CC-BY-4.0) Public License</a>.

@section trademarks Trademarks

<b>RVO2 Library</b> is a trademark of the University of North Carolina at Chapel
Hill. OpenMP is a trademark of the OpenMP Architecture Review Board, registered
in the United States and other countries. Intel is a trademark of Intel
Corporation, registered in the United States and other countries. Apache is a
trademark of the Apache Software Foundation. Creative Commons, CC, and CCO are
trademarks of Creative Commons Corporation, registered in the United States and
other countries. Other names may be trademarks of their respective owners.

 */

#endif /* RVO_RVO_H_ */
