# TFD plan solver

## TFD instalation

1. `mkdir ~/tfd_planning`
2. `cd ~/tfd_planning`
3. follow the instructions: [official TFD repository](https://github.com/IntelligentRoboticsLabs/plansys2_tfd_plan_solver)

Generate a fast execute command:

1. copy wherever you want: `launch_tfd.sh` script 
2. `echo alias tfd_launch='[launch_tfd.sh path] $1 $2' >> ~/.bashrc`
3. `cd ~/`
4. `source .bashrc`


## Execute TFD

Manual execution:

1. `python2.7 $TFD_HOME/translate/translate.py [domain_name].pddl [problem_name].pddl`
2. `$TFD_HOME/preprocess/preprocess < output.sas`
3. `$TFD_HOME/search/search y Y a T 10 t 5 e r O 1 C 1 p $TFD_HOME/plan < output`


Execute with the command (previously configurated):
```
tfd_launch [domain].pddl [problem].pddl
```

# Documentation

TFD (Temporal Fast Downward) is a planning system for temporal problems that is capable of finding low-makespan plans by performing a heuristic search in temporal search space. The importance of TFD is the ability to to handle concurrency and numeric fluents. TFD wants to approach the behavior to execute in real time scenarios.

Temporal planning also covers temporal dependencies and admits plans with concurrent durative actions. Also separates the planning and scheduling phases, this is basically separate time and space in a certain planification. This allows the planner to plan faster and more efficient and optimise better the solution 


TFD born from from FD (fast downward) planning system. Fast Downward uses hierarchical decompositions of planning tasks for computing its heuristic function, called the causal graph heuristic, which is very different from traditional HSP-like heuristics based on ignoring negative interactions of operators.



### Support FD

| Requeriment | Suported |
| ------------- | ------------- |
| :strips | 	Yes | 
| :requeriments	| Yes |
| :fluents	| Yes | 
| :equality	| Yes | 
| :durative-actions | Yes | 
| :typing	| Yes |
| :conditional-effects	| Yes |
| :equality	| Yes |

### Support FD

| Requeriment | Suported |
| ------------- | ------------- |
| :strips | 	Yes | 
| :typing	| Yes |
| :disjunctive-preconditions	| Yes | 
| :equality	| Yes | 
| :existential-preconditions | Yes | 
| :conditional-effects	| Yes
| :adl | Yes |
| PDDL2.2| 
| :derived-predicates	| Yes |
| PDDL3.1| 
| :action-costs	| Yes |


[official TFD papers](http://gki.informatik.uni-freiburg.de/tools/tfd/literature.html)

[official TFD website](http://gki.informatik.uni-freiburg.de/tools/tfd/index.html)

[FD references](https://planning.wiki/ref/planners/fd)
