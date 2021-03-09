# Metric-ff planner
This plan solver is an extension of the ff planner to PDDL 2.1, so it is numeric and could work with fluents.

## Instalation
1. Download the compressed file, with the source code from its official website [Metric-ff](https://fai.cs.uni-saarland.de/hoffmann/metric-ff.html)
2. Decompress and inside the folder compile with make.
    * `tar xzf "Metric-FF-v2.1.tgz"`
    * `cd Metric-FF-v2.1`
    * `make`

## Run
Once compiled just run the script `ff`
It has some compulsory options, launched as:
* `./ff -o [domain] -f [problem] -s [algorithm (0-5)]`

## About metric-ff
Metric-ff is a valid planner which solves problems correctly.
It is easy to install and run, also it has options that allows the user to choose the search algorithm and see different results (A* does not work properly).
About pddl, it supports some resources: strips, typing, disjunctive-preconditions, equality, conditional-effects and fluents. This last one is the great diference between the ff planner and metric-ff. Also, in the latest version they added a version which handles derived-predicates from pddl 2.2.
However, metric-ff does not allows durative-actions and nothing related with them, which is a clear disadvantage.