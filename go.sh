#!/bin/bash

# bin/find_max_range <(cat data/log/robotdata*.log)
bin/particle_filter data/log/robotdata2.log --max-range 8191 --weights 1,0.2,100,0.5
