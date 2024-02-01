#!/bin/bash

python /usr/share/sumo/tools/visualization/plot_summary.py \
    -i due.actuated.summary.xml \
    --xlim 0,86400 \
    -o img/summary_running.png \
    --xticks 0,86400,10800,10 --ygrid --xgrid \
    --ylabel "Density (Active cars)" --xlabel "Time(min)" \
    -l Density \
    --xticks-file ./img/xticks \
    --legend-position "lower right"\
    --adjust .14,.1 



# python /usr/share/sumo/tools/visualization/plotXMLAttributes.py ssm.xml \
#     -o img/TTC_summary.png \
#     -x time --xlabel "Time [s]" -y value --ylabel "TTC [s]" -i ego \
#     --title "time to collision over simulation time" --scatterplot
