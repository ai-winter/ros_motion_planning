# Important Updates

| Date | Update |
| :--: | ------ |
| 2023.1.13 | cost of motion nodes is set to `NEUTRAL_COST`, which is unequal to that of heuristics, so there is no difference between A* and Dijkstra. This bug has been solved in A* C++ v1.1 |
| 2023.1.18 | update RRT C++ v1.1, adding heuristic judgement when generating random nodes |
| 2023.2.25 | update PID C++ v1.1, making desired theta the weighted combination of theta error and theta on the trajectory |
| 2023.3.16 | support dynamic simulation environment, user can add pedestrians by modifing `pedestrian_config.yaml` |