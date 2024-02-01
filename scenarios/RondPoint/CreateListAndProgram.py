
import traci 

#################################################################################################################

#creation des cycles des feux voitures East et West sans le cycle des bus

#------------------------------------------------------------------------

def CreateProgramTrafficLights(Program,BusRedOutRedInGreen,BusRedOutRedInYellow,BusRedOutGreenInRed,BusRedOutYellowInRed,BusRedOutRedInRed,pos):
	i=1

	#programme du feu East sans intervention du feu bus
	if pos=='East':
		while i<=26:
			#exterieur rouge
			if i<=7:
				Program.append(BusRedOutGreenInRed)
			#exterieur jaune
			if 7<i<=10:
				Program.append(BusRedOutYellowInRed)
			#tout rouge
			if 10<i<=12:
				Program.append(BusRedOutRedInRed)
			#interieur vert
			if 12<i<=20:
				Program.append(BusRedOutRedInGreen)
			#interieur jaune
			if 20<i<=24:
				Program.append(BusRedOutRedInYellow)
			#tout rouge
			if 24<i<=26:
				Program.append(BusRedOutRedInRed)
			i=i+1

	#programme du feu West sans intervention du feu bus
	if pos=='West':
		while i<=25:
			#exterieur vert
			if i<=11:
				Program.append(BusRedOutGreenInRed)
			#exterieur jaune
			if 11<i<=14:
				Program.append(BusRedOutYellowInRed)
			#tout rouge
			if 14<i<=15:		
				Program.append(BusRedOutRedInRed)
			#interieur vert
			if 15<i<=21:
				Program.append(BusRedOutRedInGreen)
			#interieur jaune
			if 21<i<=24:
				Program.append(BusRedOutRedInYellow)
			#tout rouge
			if 24<i<=25:
				Program.append(BusRedOutRedInRed)
			i=i+1
	return Program

#################################################################################################################

#creation du cycle des feux de bus East et West selon l'etat du cycle du feu des voitures

#-----------------------------------------------------------------------------------------

def CreateProgramBusTrafficLights(Program,BusGreen,BusYellow,BusRedOutRedInGreen,BusRedOutRedInYellow,BusRedOutGreenInRed,BusRedOutYellowInRed,BusRedOutRedInRed,pos):
	i=j=k=1
	Esc1=[]
	Esc2=[]

	#creation du programme du feu du bus sur le carrefour East
	if pos=='East':
		while i<=7:

			#enregistrement de la position du feu avant que le bus arrive pour i=1,2,5,6
			if i==1:
				Program.append(BusRedOutYellowInRed)
			if i==2:
				Program.append(BusRedOutRedInYellow)

			#passage au vert du bus lorsque la voie exterieure est au vert ou au jaune
			if i==3:
				while j<=23 :
					if 1<=j<=2:
						Esc1.append(BusRedOutGreenInRed)
					if 3<=j<=6:
						Esc1.append(BusRedOutYellowInRed)
					if 7<=j<=19:
						Esc1.append(BusGreen)
					if 20<=j<=23:
						Esc1.append(BusYellow)
					j=j+1
				Program.append(Esc1)

			#passage au vert du bus lorsque la voie interieure est au vert ou au jaune
			if i==4:
				while k<23:
					if 1<=k<=7:
						Esc2.append(BusRedOutRedInGreen)
					if 8<=k<11:
						Esc2.append(BusRedOutRedInYellow)
					if 11<=k<20:
						Esc2.append(BusGreen)
					if 20<=k<23:
						Esc2.append(BusYellow)
					k=k+1
				Program.append(Esc2)

			if i==5:
				Program.append(BusRedOutGreenInRed)
			if i==6:
				Program.append(BusRedOutRedInGreen)

			#pour reprendre le feu vehicule i=7 (tout le monde rouge)
			if i==7:
				Program.append(BusRedOutRedInRed)
			i=i+1

	#creation du programme du feu du bus sur le carrefour West
	if pos=='West':
		while i<=7:

			#enregistrement de la position du feu avant que le bus arrive pour i=1,2,5,6
			if i==1:
				Program.append(BusRedOutYellowInRed)
			if i==2:
				Program.append(BusRedOutRedInYellow)

			#passage au vert du bus lorsque la voie exterieure est au vert ou au jaune
			if i==3:
				while j<=21 :
					if 1<=j<=3:
						Esc1.append(BusRedOutGreenInRed)
					if 4<=j<=7:
						Esc1.append(BusRedOutYellowInRed)
					if 8<=j<=17:
						Esc1.append(BusGreen)
					if 18<=j<=21:
						Esc1.append(BusYellow)
					j=j+1
				Program.append(Esc1)

			#passage au vert du bus lorsque la voie interieure est au vert ou au jaune
			if i==4:
				while k<24:
					if 1<=k<=3:
						Esc2.append(BusRedOutRedInGreen)
					if 4<=k<=7:
						Esc2.append(BusRedOutRedInYellow)
					if 8<=k<21:
						Esc2.append(BusGreen)
					if 21<=k<24:
						Esc2.append(BusYellow)
					k=k+1
				Program.append(Esc2)

			if i==5:
				Program.append(BusRedOutGreenInRed)
			if i==6:
				Program.append(BusRedOutRedInGreen)

			#pour reprendre le feu vehicule i=7 (tout le monde rouge)
			if i==7:
				Program.append(BusRedOutRedInRed)
			i=i+1


	return Program




#################################################################################################################

#fonction de synchronisation des feux Bus et Voitures East et West

#-----------------------------------------------------------------
			
	

def ChangeBusTrafficLights(loop1,loop2,feu,step,check,Prog,i,ProgInitial,counter,b,pos,cycle,extension):

    #enregistrement si des bus sont sur les boucles
    no1 = traci.inductionloop.getLastStepVehicleNumber(loop1)
    no2 = traci.inductionloop.getLastStepVehicleNumber(loop2)
    no=no1+no2

    #va permettre d'enregistrer la position du feu voiture lorsqu'un bus est detecte
    a=len(Prog)-3
    c=len(Prog)-2

    #Pas de bus detecte, le cycle tourne sans les feux de Bus. Une adaptation du feu vert est mis en place en fonction de l'occupation des voies
    if no==0 and check=='ok':

	#si on arrrive a la fin du cycle du feu vehicule on le reprend au debut
	if i==len(ProgInitial)-1:
		traci.trafficlights.setRedYellowGreenState(feu,ProgInitial[i])
		i=0
	else:	
		if pos=='East':

			#debut du cycle du feu East des voitures 
			if i<6:
			        traci.trafficlights.setRedYellowGreenState(feu,ProgInitial[i])
				i=i+1
				if i==5:
					cycle=0

			#l'adaptation au feu vert ne peut depasser une duree 'cycle' 
			elif i>=6 and cycle<37 :

				#mise a jour toutes les 3 secondes et le vert au total ne peut depasser 38s
				if cycle%3==0:

					#mesure du taux d'occupation interieur et exterieur
					if (traci.trafficlights.getRedYellowGreenState(feu)=='rrGGGGGrrrrrrr' and traci.edge.getLastStepOccupancy('-1354')>0.15) :
						extension='yes'
					else:
						extension='no'

				#prolongement du vert de 3s
				if extension=='yes':
					traci.trafficlights.setRedYellowGreenState(feu,'rrGGGGGrrrrrrr')
					cycle=cycle+1

				#arret du prolongement
				elif extension=='no': 
					cycle=38

			#reprise du cycle normal 
			elif i>=6 and cycle>=37 :
				traci.trafficlights.setRedYellowGreenState(feu,ProgInitial[i])
				i=i+1

		#le feu West ne possede pas de prolongement de vert
		if pos=='West':
			traci.trafficlights.setRedYellowGreenState(feu,ProgInitial[i])
			i=i+1
				

    #un bus est detecte sur une boucle, on retient l'etat du feu et on initialise le feu du bus
    if no > 0 and check=='ok':
	check=''

	#enregistrement de la position du feu
	statutactuel=traci.trafficlights.getRedYellowGreenState(feu)
	programPointeur=Prog.index(statutactuel)
	b=programPointeur

	#selon cette position (exterieur vert ou interieur vert) on attaque le programme du feu de bus de maniere differente
	if b==len(Prog)-1:

		#tous les feux sont rouges
		if pos=='East':
			traci.trafficlights.setRedYellowGreenState(feu,Prog[2][6])
			i=7
		if pos=='West':
			traci.trafficlights.setRedYellowGreenState(feu,Prog[2][7])
			i=8
		counter=0

	elif b==1:

		#l'exterieur est au jaune
		if pos=='East':
			traci.trafficlights.setRedYellowGreenState(feu,Prog[2][2])
			i=3
		if pos=='West':
			traci.trafficlights.setRedYellowGreenState(feu,Prog[2][3])
			i=4
		counter=2


	elif c-b==0:

		#l'interieur est au vert
		traci.trafficlights.setRedYellowGreenState(feu,Prog[3][0])
		i=1
		counter=3


	elif a-b==0:

		#l'exterieur est au vert
		traci.trafficlights.setRedYellowGreenState(feu,Prog[2][0])
		i=1
		counter=4

	else:

		#l'interieur est au jaune
		if pos=='East':
			traci.trafficlights.setRedYellowGreenState(feu,Prog[3][7])
			i=8
		if pos=='West':		
			traci.trafficlights.setRedYellowGreenState(feu,Prog[3][3])
			i=4
		counter=1
		
    #le bus est sur la boucle et l'initialisation a ete fait : cycle du bus en route
    if no > 0 and check=='':

	if counter==1 or counter==4 or counter==0:

		#si on arrive au bout du cycle du bus, on le reprend
		if i==len(Prog[2])-1:
			traci.trafficlights.setRedYellowGreenState(feu,Prog[2][i])
			i=0
		#cycle du bus 
		else:
			traci.trafficlights.setRedYellowGreenState(feu,Prog[2][i])
			i=i+1
		
	if counter==2 or counter==3:

		#si on arrive au bout du cycle du bus, on le reprend
		if i==len(Prog[3])-1:
			traci.trafficlights.setRedYellowGreenState(feu,Prog[3][i])
			i=0

		#cycle du bus 
		else:
			traci.trafficlights.setRedYellowGreenState(feu,Prog[3][i])
			i=i+1
	
    #le bus n'est plus sur la boucle on fini le cycle du bus et on reprend le cycle des voitures		
    if no==0 and check=='':
	if counter==1 or counter==4 or counter==0:

		#le cycle du bus est fini on reprend le cycle des voitures avec vert a l'exterieur
		if i==len(Prog[2])-1:
			check='ok'
			i=len(ProgInitial)-1
			traci.trafficlights.setRedYellowGreenState(feu,ProgInitial[i])

		#sinon on fini le cycle du bus
		else:
			traci.trafficlights.setRedYellowGreenState(feu,Prog[2][i])
			i=i+1
		
	if counter==2 or counter==3:

		#le cycle du bus est fini on reprend le cycle des voitures avec vert a l'interieur
		if i==len(Prog[3])-1:
			check='ok'
			if pos=='East':
				i=12
			else:
				i=15
			traci.trafficlights.setRedYellowGreenState(feu,ProgInitial[i])

		#sinon on fini le cycle du bus
		else:
			traci.trafficlights.setRedYellowGreenState(feu,Prog[3][i])
			i=i+1

    #on retourne les valeurs pour le pas de temps suivant
    return check,i,counter,b,cycle,extension

################################################################################################################

#Creation cycle feu Nord et West2

#--------------------------------

def Creation(feu,Program):
	d=l=0
	if feu=='North':
		while l<=29:
			#interieur vert
			if l<=4:
				Program.append("rrGGG")
			#interieur jaune
			if 4<l<=7:
				Program.append("rryyy")
			#tout rouge
			if 7<l<=8:
				Program.append("rrrrr")
			#exterieur vert
			if 8<l<=15:
				Program.append("GGrrr")
			#exterieur jaune
			if 15<l<=18:
				Program.append("yyrrr")
			#tout rouge
			if 18<l<=20:
				Program.append("rrrrr")
			#interieur vert
			if 20<l<=29:
				Program.append("rrGGG")
			l=l+1
	if feu=='West2':
		while d<=27:
			#vert
			if d<=7:
				Program.append("GG")
			#jaune
			if 7<d<=10:
				Program.append("yy")
			#rouge
			if 10<d<=20:
				Program.append("rr")
			#vert
			if 20<d<=27:
				Program.append("GG")
			d=d+1
	return Program

################################################################################################################

#adaptation en fonction du nombre de vehicule des feux nord et west2

#-------------------------------------------------------------------

def Adapt(feu,Prog,i,pos,cycle,extension):
	if pos=='North':

		#si on arrive a la fin du programme on le recommence
		if i==len(Prog)-1:
			traci.trafficlights.setRedYellowGreenState(feu,Prog[i])
			i=0
		else:	
			#sinon debut du cycle du feu
			if i<8:
				traci.trafficlights.setRedYellowGreenState(feu,Prog[i])
				i=i+1
				if i==7:
					cycle=0

			#l'adaptation au feu vert ne peut depasser une duree 'cycle' 
			elif i>=8 and cycle<15 :

				#mise a jour toutes les 3 secondes et le vert au total ne peut depasser 38s
				if cycle%3==0:

					#mesure du taux d'occupation exterieur
					if (traci.trafficlights.getRedYellowGreenState(feu)=="GGrrr" and traci.edge.getLastStepOccupancy('28925528')>0.15) :
						extension='yes'
					else:
						extension='no'

				#prolongement du vert de 3s
				if extension=='yes':
					traci.trafficlights.setRedYellowGreenState(feu,"GGrrr")
					cycle=cycle+1

				#arret du prolongement
				elif extension=='no': 
					cycle=15

			#reprise du cycle normal 
			elif i>=8 and cycle>=15 :
				traci.trafficlights.setRedYellowGreenState(feu,Prog[i])
				i=i+1
	if pos=='West2':
		if i==len(Prog)-1:
			traci.trafficlights.setRedYellowGreenState(feu,Prog[i])
			i=0
		else:	
			#debut du cycle du feu
			if i<8:
				traci.trafficlights.setRedYellowGreenState(feu,Prog[i])
				i=i+1
				if i==7:
					cycle=0

			#l'adaptation au feu vert ne peut depasser une duree 'cycle' 
			elif i>=8 and cycle<40 :

				#mise a jour toutes les 3 secondes et le vert au total ne peut depasser 38s
				if cycle%3==0:

					#mesure du taux d'occupation
					if (traci.trafficlights.getRedYellowGreenState(feu)=="GG" and traci.edge.getLastStepOccupancy('105490271#0')>0.15) :
						extension='yes'
					else:
						extension='no'

				#prolongement du vert de 3s
				if extension=='yes':
					traci.trafficlights.setRedYellowGreenState(feu,"GG")
					cycle=cycle+1

				#arret du prolongement
				elif extension=='no': 
					cycle=40

			#reprise du cycle normal 
			elif i>=8 and cycle>=40 :
				traci.trafficlights.setRedYellowGreenState(feu,Prog[i])
				i=i+1
		
	return i,cycle,extension
	
	




