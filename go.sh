#!/bin/bash

# bin/find_max_range <(cat data/log/robotdata*.log)
make o3 &&
bin/particle_filter data/log/robotdata1.log \
--max-range 8191 \
--weights 1,0.8,100,0.5 \
--exp-decay 1e-4 \
--sigma 200 \
--motion-sigma-xy 10 \
-n 5000 --show-ray-tracing true
