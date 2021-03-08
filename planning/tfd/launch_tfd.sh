#!/bin/bash
python2.7 $TFD_HOME/translate/translate.py [domain_name].pddl [problem_name].pddl
$TFD_HOME/preprocess/preprocess < output.sas
$TFD_HOME/search/search y Y a T 10 t 5 e r O 1 C 1 p $TFD_HOME/plan < output
