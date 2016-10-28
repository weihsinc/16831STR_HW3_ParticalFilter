#!/bin/bash

# bin/find_max_range <(cat data/log/robotdata*.log)
#make clean 
make o3 &&
bin/particle_filter data/log/robotdata4.log \
--max-range 8192 \
--weights 0.5,0.2,100,0.2 \
--exp-decay 1e-4 \
--sigma 80 \
--motion-sigma-xy 10 \
--motion-sigma-theta 0.03 \
-n 5000 # --show-ray-tracing true
# --show-ray-tracing true
#--weights 1.0.8.100.0.5
# --sigma 400 \ #200
