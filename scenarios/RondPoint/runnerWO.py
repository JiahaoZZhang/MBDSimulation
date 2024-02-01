#!/usr/bin/env python
import libxml2
from libxml2 import xmlAttr
import os,subprocess,sys,shutil
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', "tools")) # tutorial in tests
sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(os.path.dirname(__file__), "..", "..", "..")), "tools")) # tutorial in docs
from sumolib import checkBinary
import traci
import CreateListAndProgram
from CreateListAndProgram import ChangeBusTrafficLights, CreateProgramTrafficLights, CreateProgramBusTrafficLights,Creation,Adapt
	
PORT = 8880

###############################################################################################""

#Initialisation

#---------------------

#initialisation des valeurs des feux West Bus et Voiture

BusGreen = 'rrrrrGGGGrrrrr' 
BusYellow = 'rrrrryyyyrrrrr'
BusRedOutRedInGreen = 'GGGGGrrrrrrrrr' 
BusRedOutRedInYellow ='yyyyyrrrrrrrrr' 
BusRedOutGreenInRed = 'rrrrrrrrrGGGGG'
BusRedOutYellowInRed = 'rrrrrrrrryyyyy'
BusRedOutRedInRed = 'rrrrrrrrrrrrrr'

#initialisation des valeurs des feux East Bus et Voiture

BusGreen2 = 'GGrrrrrrrrrrGG' 
BusYellow2 = 'yyrrrrrrrrrryy'
BusRedOutRedInGreen2 = 'rrrrrrrGGGGGrr' 
BusRedOutRedInYellow2 ='rrrrrrryyyyyrr' 
BusRedOutGreenInRed2 = 'rrGGGGGrrrrrrr'
BusRedOutYellowInRed2 = 'rryyyyyrrrrrrr'
BusRedOutRedInRed2 = 'rrrrrrrrrrrrrr'

#initialisation de l'interface TraCI pour interagir avec SUMO

if not traci.isEmbedded():
    sumoBinary = checkBinary('sumo-gui')
    sumoConfig = "data/RondPoint.sumocfg"
    sumoProcess = subprocess.Popen("%s -c %s" % (sumoBinary, sumoConfig), shell=True, stdout=sys.stdout)
    traci.init(PORT)

#creation du cycle voiture West
ProgramInitial1=CreateProgramTrafficLights([],BusRedOutRedInGreen,BusRedOutRedInYellow,BusRedOutGreenInRed,BusRedOutYellowInRed,BusRedOutRedInRed,'West')

#creation du cycle Bus West
Program1= CreateProgramBusTrafficLights([],BusGreen,BusYellow,BusRedOutRedInGreen,BusRedOutRedInYellow,BusRedOutGreenInRed,BusRedOutYellowInRed,BusRedOutRedInRed,'West')
programPointer1=len(Program1)-3

#creation du cycle voiture East
ProgramInitial2=CreateProgramTrafficLights([],BusRedOutRedInGreen2,BusRedOutRedInYellow2,BusRedOutGreenInRed2,BusRedOutYellowInRed2,BusRedOutRedInRed2,'East')

#creation du cycle Bus East
Program2= CreateProgramBusTrafficLights([],BusGreen2,BusYellow2,BusRedOutRedInGreen2,BusRedOutRedInYellow2,BusRedOutGreenInRed2,BusRedOutYellowInRed2,BusRedOutRedInRed2,'East')
programPointer2=len(Program2)-3

#creation feu North et West2
ProgramNorth=Creation('North',[])
ProgramWest2=Creation('West2',[])

#pas de temps
step = 0

#pas des feux
iWest2=iNorth=i1=i2=0

#enregistrement d'un passage de bus
check1=check2='ok'

#memorisation de la position du cycle du feu voiture pour passer au cycle du feu bus
counter1=counter2=0
b1=b2=0

#parametre de mise a jour toute les 3s du feu vert en fonction du taux d'occupation
cycleNorth=cycleWest2=cycle1=cycle2=0

#marque le taux d'occupation et le besoin d'etendre le feu vert
extensionNorth=extensionWest2=extension1=extension2=''

###############################################################################################""

#debut de la simulation

#---------------------


while step==0 or traci.simulation.getMinExpectedNumber() > 0:
    traci.simulationStep()

    #adaptation du feu vert Nord en fonction du taux d'occupation du lien entrant sur le rond point
    North=Adapt('431399475',ProgramNorth,iNorth,'North',cycleNorth,extensionNorth)
    iNorth=North[0]
    cycleNorth=North[1]
    extensionNorth=North[2]

    #adaptation du feu vert West2 en fonction du taux d'occupation du lien entrant sur le rond point
    West2=Adapt('2470886209',ProgramWest2,iWest2,'West2',cycleWest2,extensionWest2)
    iWest2=West2[0]
    cycleWest2=West2[1]
    extensionWest2=West2[2]

    #on bloque les 100 premieres secondes au rouge pour accumuler les voitures pour avoir l'effet d'une simulation continue
    if step<100:
	    traci.trafficlights.setRedYellowGreenState("2479049221",'rrrrrrrrrrrrrr')
	    traci.trafficlights.setRedYellowGreenState("1214943442",'rrrrrrrrrrrrrr')
    else:
	
            #adaptation du feu West avec le bus
	    result1=ChangeBusTrafficLights("3","0","1214943442",step,check1,Program1,i1,ProgramInitial1,counter1,b1,'West',cycle1,extension1)
	    check1=result1[0]
	    i1=result1[1]
	    counter1=result1[2]
	    b1=result1[3]
	    cycle1=result1[4]
	    extension1=result1[5]


            #adaptation du feu East avec le bus et le taux d'occupation sur le lien entrant
	    result2=ChangeBusTrafficLights("2","1","2479049221",step,check2,Program2,i2,ProgramInitial2,counter2,b2,'East',cycle2,extension2)
	    check2=result2[0]
	    i2=result2[1]
	    counter2=result2[2]
	    b2=result2[3]
	    cycle2=result2[4]
	    extension2=result2[5]


     
  
    #pas de temps suivant    
    step += 1

traci.close()
sys.stdout.flush()
