#!/bin/bash

python /usr/share/sumo/tools/visualization/plot_net_dump.py -n pasubio_buslanes.net.xml \
--measures density,density --xlabel [m] --ylabel [m] \
--default-width 1 -i edgedata.xml,edgedata.xml \
--min-color-value 0 --max-color-value 100 \
--max-width-value 100 --min-width-value 0 \
--max-width 3 --min-width .5 \
--colormap "#0:#0000c0,.25:#404080,.5:#808080,.75:#804040,1:#c00000" \
-o plot_density.png