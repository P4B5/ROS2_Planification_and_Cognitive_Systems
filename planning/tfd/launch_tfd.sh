#!/bin/bash

python2.7 ~/tfd_planning/tfd-src-0.4/downward/translate/translate.py $1 $2
~/tfd_planning/tfd-src-0.4/downward/preprocess/preprocess < ./output.sas
~/tfd_planning/tfd-src-0.4/downward/search/search y Y a T 10 t 5 e r O 1 C 1 p ./tfd-src-0.4/downward/plan < ./output
