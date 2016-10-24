#!/bin/bash

# bin/find_max_range <(cat data/log/robotdata*.log)
make &&
bin/particle_filter data/log/robotdata1.log \
--max-range 3000 \
--weights 1,0.8,100,0.5 \
--exp-decay 1e-4 \
--sigma 200 \
--motion-sigma 15 \
-n 5000 # --show-ray-tracing true
