#!/bin/bash

###############################################################################################""

#Creation des routes en fonctions des fichiers flows

#---------------------

duarouter --repair --ignore-errors -n data/RondPoint.net.xml -f data/RondPoint.flow.xml -o data/RondPoint.rou.xml
duarouter --repair --ignore-errors -n data/RondPoint.net.xml -f data/RondPointBus.flow.xml -o data/RondPointBus.rou.xml



#probleme :
#Bus S-W 4.0 / N-E 2.0
#Vehicle N-S 2.12 / N-S 3.3 / N-N 1.0 / N-S 1.6 / N-E 3.7

