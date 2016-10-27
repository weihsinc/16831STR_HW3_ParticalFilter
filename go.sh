#!/bin/bash

# bin/find_max_range <(cat data/log/robotdata*.log)
make o3 &&
bin/particle_filter data/log/robotdata2.log \
--max-range 8192 \
--weights 1,0.2,100,0.5 \
--exp-decay 1e-4 \
--sigma 200 \
--motion-sigma-xy 5 \
--motion-sigma-theta 0.01 \
-n 10000 # --show-ray-tracing true
