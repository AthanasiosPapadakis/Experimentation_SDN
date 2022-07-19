from re import L
import sys
import numpy as np
np.set_printoptions(threshold=sys.maxsize)
from tkinter import *
from tkinter.ttk import *
import matplotlib.pyplot as plt
import pandas as pd
import statistics
from statistics import mean
import linecache

vehicles = 50
static_nodes = 24
maximum_refresh_rate=60
minimum_refresh_rate=10
medium_refresh_rate=20
perfectcounter = 0
mobile_range = 80

#Based on the length of the experiment
number_entries = 185
entries = 8461

# Create an array for storing the time spend in each Static Nodes visited by each Mobile node
timestayed = np.array(range(vehicles * number_entries))
timestayed = timestayed.reshape(vehicles, number_entries)

# Create an array for storing the Static Nodes visited by each Mobile node
sequence = np.array(range(vehicles * number_entries))
sequence = sequence.reshape(vehicles, number_entries)

# Initialize the array with zeros since there won't be Static node = 0
for a in range(vehicles):
    for b in range(number_entries):
        timestayed[a][b] = 0
        sequence[a][b] = 0

# Create an array for storing how many differentvisits we have to each Static Node by each Mobile node
differentvisits = np.array(range(vehicles * 1))
differentvisits = differentvisits.reshape(vehicles, 1)

# Initialize the array with zeros
for a in range(vehicles):
    for b in range(1):
        differentvisits[a][b] = 0

# Create an array with the perfect changes
perfectchanges = np.array(range(entries * 3))
perfectchanges = perfectchanges.reshape(entries, 3)

# Initialize the array with zeros
for a in range(entries):
    for b in range(3):
        perfectchanges[a][b] = 0

# Open the file with the data
f1 = open("raw50_1.txt", "r")

# Check every line from CARLA output
for line in f1:

    # Input the line to data
    data = line.split()

    for y in range(24, vehicles + static_nodes + 1):
        if int(data[0]) == y:
            if differentvisits[y - static_nodes - 1] == 0:
                # Store the first Static node and the first timestamp
                timestayed[y - static_nodes - 1][differentvisits[y - static_nodes - 1]] = data[1]
                # Store the first Static node and the first timestamp
                sequence[y - static_nodes - 1][differentvisits[y - static_nodes - 1]] = data[2]
                # Step
                differentvisits[y - static_nodes - 1] = differentvisits[y - static_nodes - 1] + 1
                # perfectchanges array
                perfectchanges[perfectcounter][0] = data[0]
                perfectchanges[perfectcounter][1] = data[1]
                perfectchanges[perfectcounter][2] = data[2]
                perfectcounter = perfectcounter + 1

            # Check if the next static node is different from the previous
            elif int(data[2]) != sequence[y - static_nodes - 1][differentvisits[y - static_nodes - 1] - 1]:
                # Calculate the time stayed in this Static node
                timestayed[y - static_nodes - 1][differentvisits[y - static_nodes - 1] - 1] = int(data[1]) - timestayed[y - static_nodes - 1][
                    differentvisits[y - static_nodes - 1] - 1]
                # We are using the next slot to store the previous timestamp:
                timestayed[y - static_nodes - 1][differentvisits[y - static_nodes - 1]] = int(data[1])
                # Store the new Static node
                sequence[y - static_nodes - 1][differentvisits[y - static_nodes - 1]] = data[2]
                # Step
                differentvisits[y - static_nodes - 1] = differentvisits[y - static_nodes - 1] + 1
                # perfectchanges array
                perfectchanges[perfectcounter][0] = data[0]
                perfectchanges[perfectcounter][1] = data[1]
                perfectchanges[perfectcounter][2] = data[2]
                perfectcounter = perfectcounter + 1

f1.close()

#We need to remove the last visit and time stayed from the experiment, from timestayed and sequence arrays
#Identify where the experiment ends
for i in range(0, vehicles):
    #print((np.where(timestayed[i] == 0)[0][0])-1)
    timestayed[i][((np.where(timestayed[i] == 0)[0][0])-1)]=0
    #print((np.where(sequence[i] == 0)[0][0]-1))
    sequence[i][((np.where(sequence[i] == 0)[0][0])-1)]=0

# Create an array to store the sum of time stayed each mobile node to each static node
sumtimestayed = np.array(range(vehicles * static_nodes))
sumtimestayed = sumtimestayed.reshape(vehicles, static_nodes)

# Create an array to store the sum of time visited each mobile node to each static node
sumtimesvisited = np.array(range(vehicles * static_nodes))
sumtimesvisited = sumtimesvisited.reshape(vehicles, static_nodes)

# Initialize the array with zeros
for a in range(0, vehicles):
    for b in range(0, static_nodes):
        sumtimestayed[a][b] = 0
        sumtimesvisited[a][b] = 0

# Calculate the sum time stayed each mobile node at each static node
for i in range(0, vehicles):
    for j in range(0, static_nodes):
        for z in range(0, int(differentvisits[i])):
            if int(sequence[i][z]) == j + 1:
                sumtimestayed[i][j] = sumtimestayed[i][j] + timestayed[i][z]
                sumtimesvisited[i][j] = sumtimesvisited[i][j] + 1

#___________________________________________________________________________

totaltimeSN = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
totalvisitsSN = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
timeaverage = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

#Calculate the time stayed to each static node for all the vehicles
for i in range (0, static_nodes):
    for j in range (0, vehicles):
        totaltimeSN[i]=totaltimeSN[i]+sumtimestayed[j][i]
        totalvisitsSN[i]=totalvisitsSN[i]+sumtimesvisited[j][i]

#Calculate the t average of each static node
for i in range (0, static_nodes):
    timeaverage[i]= totaltimeSN[i]/totalvisitsSN[i]
    timeaverage[i]= round(timeaverage[i],1)

df1 = pd.DataFrame(differentvisits, index=['MN1', 'MN2', 'MN3', 'MN4', 'MN5', 'MN6', 'MN7', 'MN8', 'MN9', 'MN10', 'MN11', 'MN12', 'MN13', 'MN14', 'MN15', 'MN16', 'MN17', 'MN18', 'MN19', 'MN20', 'MN21', 'MN22', 'MN23', 'MN24', 'MN25', 'MN26', 'MN27', 'MN28', 'MN29', 'MN30', 'MN31', 'MN32', 'MN33', 'MN34', 'MN35', 'MN36', 'MN37', 'MN38', 'MN39', 'MN40', 'MN41', 'MN42', 'MN43', 'MN44', 'MN45', 'MN46', 'MN47', 'MN48', 'MN49', 'MN50'], columns=["Visits"])
df1.to_csv('differentvisits.csv')
df2 = pd.DataFrame(perfectchanges, columns=["MobileNode", "Timestamp", "StaticNode"])
df2.to_csv('perfectchanges.csv')
df3 = pd.DataFrame(timestayed, index=['MN1', 'MN2', 'MN3', 'MN4', 'MN5', 'MN6', 'MN7', 'MN8', 'MN9', 'MN10', 'MN11', 'MN12', 'MN13', 'MN14', 'MN15', 'MN16', 'MN17', 'MN18', 'MN19', 'MN20', 'MN21', 'MN22', 'MN23', 'MN24', 'MN25', 'MN26', 'MN27', 'MN28', 'MN29', 'MN30', 'MN31', 'MN32', 'MN33', 'MN34', 'MN35', 'MN36', 'MN37', 'MN38', 'MN39', 'MN40', 'MN41', 'MN42', 'MN43', 'MN44', 'MN45', 'MN46', 'MN47', 'MN48', 'MN49', 'MN50'])
df3.to_csv('timestayed.csv')
df4 = pd.DataFrame(sequence, index=['MN1', 'MN2', 'MN3', 'MN4', 'MN5', 'MN6', 'MN7', 'MN8', 'MN9', 'MN10', 'MN11', 'MN12', 'MN13', 'MN14', 'MN15', 'MN16', 'MN17', 'MN18', 'MN19', 'MN20', 'MN21', 'MN22', 'MN23', 'MN24', 'MN25', 'MN26', 'MN27', 'MN28', 'MN29', 'MN30', 'MN31', 'MN32', 'MN33', 'MN34', 'MN35', 'MN36', 'MN37', 'MN38', 'MN39', 'MN40', 'MN41', 'MN42', 'MN43', 'MN44', 'MN45', 'MN46', 'MN47', 'MN48', 'MN49', 'MN50'])
df4.to_csv('sequence.csv')
df5 = pd.DataFrame(sumtimestayed, columns=["SN1", "SN2", "SN3", "SN4", "SN5", "SN6", "SN7", "SN8", "SN9", "SN10", "SN11", "SN12", "SN13", "SN14", "SN15", "SN16", "SN17", "SN18", "SN19", "SN20", "SN21", "SN22", "SN23","SN24"], index=['MN1', 'MN2', 'MN3', 'MN4', 'MN5', 'MN6', 'MN7', 'MN8', 'MN9', 'MN10', 'MN11', 'MN12', 'MN13', 'MN14', 'MN15', 'MN16', 'MN17', 'MN18', 'MN19', 'MN20', 'MN21', 'MN22', 'MN23', 'MN24', 'MN25', 'MN26', 'MN27', 'MN28', 'MN29', 'MN30', 'MN31', 'MN32', 'MN33', 'MN34', 'MN35', 'MN36', 'MN37', 'MN38', 'MN39', 'MN40', 'MN41', 'MN42', 'MN43', 'MN44', 'MN45', 'MN46', 'MN47', 'MN48', 'MN49', 'MN50'])
df5.to_csv('sumtimestayed.csv')
df6 = pd.DataFrame(sumtimesvisited, columns=["SN1", "SN2", "SN3", "SN4", "SN5", "SN6", "SN7", "SN8", "SN9", "SN10", "SN11", "SN12", "SN13", "SN14", "SN15", "SN16", "SN17", "SN18", "SN19", "SN20", "SN21", "SN22", "SN23","SN24"], index=['MN1', 'MN2', 'MN3', 'MN4', 'MN5', 'MN6', 'MN7', 'MN8', 'MN9', 'MN10', 'MN11', 'MN12', 'MN13', 'MN14', 'MN15', 'MN16', 'MN17', 'MN18', 'MN19', 'MN20', 'MN21', 'MN22', 'MN23', 'MN24', 'MN25', 'MN26', 'MN27', 'MN28', 'MN29', 'MN30', 'MN31', 'MN32', 'MN33', 'MN34', 'MN35', 'MN36', 'MN37', 'MN38', 'MN39', 'MN40', 'MN41', 'MN42', 'MN43', 'MN44', 'MN45', 'MN46', 'MN47', 'MN48', 'MN49', 'MN50'])
df6.to_csv('sumtimesvisited.csv')
df7 = pd.DataFrame(timeaverage, columns=["Seconds"], index=["SN1", "SN2", "SN3", "SN4", "SN5", "SN6", "SN7", "SN8", "SN9", "SN10", "SN11", "SN12", "SN13", "SN14", "SN15", "SN16", "SN17", "SN18", "SN19", "SN20", "SN21", "SN22", "SN23","SN24"])
df7.to_csv('timeaverage.csv')

#_______________________________________________________________________________

totalmismatch = np.array(range(vehicles * 9))
totalmismatch = totalmismatch.reshape(vehicles, 9)
totalmismatch = [[0 for col in range(9)] for row in range(vehicles)]

totalcost = np.array(range(5 * vehicles))
totalcost = totalcost.reshape(5 * vehicles)
totalcost = [[0 for col in range(vehicles)] for row in range(5)]

static_nodes_coordinates = [(365,35), (365,105), (365,175), (365,245), (365,315),
                            (295,35), (295,175), (295,315),
                            (225,35), (225,175), (225,315),
                            (155,35), (155,175), (155,315),
                            (85,35), (85,105), (85,175), (85,245), (85,315),
                            (25,35), (25,105), (25,175),(25,245), (25,315)]

#This goes from 1 to 50
for num in range (1, vehicles+1):

    num = int(num) -1
    result = np.where(sequence[int(num)] == 0)

    # Create an array with the perfect changes
    realrun = np.array(range(6 * result[0][0]))
    realrun = realrun.reshape(6, result[0][0])

    # Initialize the array with zeros
    for a in range(6):
        for b in range(result[0][0]):
            realrun[a][b] = 0

    for i in range (0, result[0][0]):
        if i ==0:
            #visit
            realrun[0][i]=i
            #beacon
            realrun[1][i]=i
            #static node
            realrun[2][i]=sequence[int(num)][i]
            #time entered
            realrun[3][i]=0
            #time stayed/rr
            realrun[4][i]=timestayed[int(num)][i]
            #time exited
            realrun[5][i]=timestayed[int(num)][i]
        else:
            realrun[0][i]=i
            realrun[1][i]=i
            realrun[2][i]=sequence[int(num)][i]
            realrun[3][i]=realrun[5][i-1]
            realrun[4][i]=timestayed[int(num)][i]
            realrun[5][i]=realrun[5][i-1]+timestayed[int(num)][i]

    #df8 = pd.DataFrame(realrun, index=['Visit', 'Beacon', 'Static Node', 'Time entered', 'Time Stayed(Refresh Rate)','Time exited'])
    #df8.to_csv('realrun.csv')

    #------------------------- RUNS ---------------------------------------
    #-------------------------AVERAGE--------------------------------------

    # Create an array with the averagerun
    averagerun = np.array(range(6 * result[0][0]))
    averagerun = averagerun.reshape(6, result[0][0])

    # Initialize the array with zeros
    for a in range(6):
        for b in range(result[0][0]):
            averagerun[a][b] = 0

    #Since the user gives the number, calculate the run with t AVERAGE
    for i in range (0, result[0][0]):
        if i ==0:
            #visit
            averagerun[0][i]=i
            #beacon
            averagerun[1][i]=i
            #static node
            averagerun[2][i]=realrun[2][i]
            #time entered
            averagerun[3][i]=0
            #time stayed/rr
            averagerun[4][i]=timeaverage[(sequence[int(num)][i])-1]
            #time exited
            averagerun[5][i]=timeaverage[(sequence[int(num)][i])-1]
        else:
            for j in range (0, result[0][0]):
                if averagerun[5][i-1] < realrun[5][j]:
                    averagerun[2][i]=realrun[2][j]
                    if averagerun[2][i]!= averagerun[2][i-1]:
                        averagerun[0][i] = averagerun[0][i-1] + 1
                    else:
                        averagerun[0][i] = averagerun[0][i-1]
                    averagerun[1][i]= averagerun[1][i-1] + 1
                    averagerun[3][i]=averagerun[5][i-1]
                    averagerun[4][i]=timeaverage[averagerun[2][i]-1]
                    averagerun[5][i]=averagerun[3][i] + averagerun[4][i]
                    break

    #df9 = pd.DataFrame(averagerun, index=['Visit', 'Beacon', 'Static Node', 'Time entered', 'Time Stayed(Refresh Rate)','Time exited'])
    #df9.to_csv('averagerun.csv')

    #---------------------------- MIN ---------------------------------------

    min_size=int((perfectchanges[perfectcounter-1][1])/minimum_refresh_rate)
    minimumrun = np.array(range(6 * min_size))
    minimumrun = minimumrun.reshape(6, min_size)

    for i in range(0, min_size):
        if i ==0:
            #visit
            minimumrun[0][i]=i
            #beacon
            minimumrun[1][i]=i
            #static node
            minimumrun[2][i]=realrun[2][i]
            #time entered
            minimumrun[3][i]=0
            #time stayed/rr
            minimumrun[4][i]=minimum_refresh_rate
            #time exited
            minimumrun[5][i]=minimum_refresh_rate
        else:
            for j in range (0, result[0][0]):
                if minimumrun[5][i-1] < realrun[5][j]:
                    minimumrun[2][i]=realrun[2][j]
                    if minimumrun[2][i]!= minimumrun[2][i-1]:
                        minimumrun[0][i] = minimumrun[0][i-1] + 1
                    else:
                        minimumrun[0][i] = minimumrun[0][i-1]
                    minimumrun[1][i]= minimumrun[1][i-1] + 1
                    minimumrun[3][i]=minimumrun[5][i-1]
                    minimumrun[4][i]=minimum_refresh_rate
                    minimumrun[5][i]=minimumrun[3][i] + minimumrun[4][i]
                    break
                elif minimumrun[5][i-1] < realrun[5][0] :
                    minimumrun[2][i]=realrun[2][j]
                    if minimumrun[2][i]!= minimumrun[2][i-1]:
                        minimumrun[0][i] = minimumrun[0][i-1] + 1
                    else:
                        minimumrun[0][i] = minimumrun[0][i-1]
                    minimumrun[1][i]= minimumrun[1][i-1] + 1
                    minimumrun[3][i]=minimumrun[5][i-1]
                    minimumrun[4][i]=minimum_refresh_rate
                    minimumrun[5][i]=minimumrun[3][i] + minimumrun[4][i]
                    break

    #df10 = pd.DataFrame(minimumrun, index=['Visit', 'Beacon', 'Static Node', 'Time entered', 'Time Stayed(Refresh Rate)','Time exited'])
    #df10.to_csv('minimumrun.csv')

    #---------------------------- MEDIUM -----------------------------------

    medium_size =int((perfectchanges[perfectcounter-1][1])/medium_refresh_rate)
    mediumrun = np.array(range(6 * medium_size))
    mediumrun = mediumrun.reshape(6, medium_size)

    for i in range(0, medium_size):
        if i ==0:
            #visit
            mediumrun[0][i]=i
            #beacon
            mediumrun[1][i]=i
            #static node
            mediumrun[2][i]=realrun[2][i]
            #time entered
            mediumrun[3][i]=0
            #time stayed/rr
            mediumrun[4][i]=medium_refresh_rate
            #time exited
            mediumrun[5][i]=medium_refresh_rate
        else:
            for j in range (0, result[0][0]):
                if mediumrun[5][i-1] < realrun[5][j]:
                    mediumrun[2][i]=realrun[2][j]
                    if mediumrun[2][i]!= mediumrun[2][i-1]:
                        mediumrun[0][i] = mediumrun[0][i-1] + 1
                    else:
                        mediumrun[0][i] = mediumrun[0][i-1]
                    mediumrun[1][i]= mediumrun[1][i-1] + 1
                    mediumrun[3][i]=mediumrun[5][i-1]
                    mediumrun[4][i]=medium_refresh_rate
                    mediumrun[5][i]=mediumrun[3][i] + mediumrun[4][i]
                    break
                elif mediumrun[5][i-1] < realrun[5][0] :
                    mediumrun[2][i]=realrun[2][j]
                    if mediumrun[2][i]!= mediumrun[2][i-1]:
                        mediumrun[0][i] = mediumrun[0][i-1] + 1
                    else:
                        mediumrun[0][i] = mediumrun[0][i-1]
                    mediumrun[1][i]= mediumrun[1][i-1] + 1
                    mediumrun[3][i]=mediumrun[5][i-1]
                    mediumrun[4][i]=medium_refresh_rate
                    mediumrun[5][i]=mediumrun[3][i] + mediumrun[4][i]
                    break

    #df11 = pd.DataFrame(mediumrun, index=['Visit', 'Beacon', 'Static Node', 'Time entered', 'Time Stayed(Refresh Rate)','Time exited'])
    #df11.to_csv('mediumrun.csv')

    #---------------------------- MAX ---------------------------------------

    max_size=int((perfectchanges[perfectcounter-1][1])/maximum_refresh_rate)
    maximumrun = np.array(range(6 * max_size))
    maximumrun = maximumrun.reshape(6, max_size)

    for i in range(0, max_size):
        if i ==0:
            #visit
            maximumrun[0][i]=i
            #beacon
            maximumrun[1][i]=i
            #static node
            maximumrun[2][i]=realrun[2][i]
            #time entered
            maximumrun[3][i]=0
            #time stayed/rr
            maximumrun[4][i]=maximum_refresh_rate
            #time exited
            maximumrun[5][i]=maximum_refresh_rate
        else:
            for j in range (0, result[0][0]):
                if maximumrun[5][i-1] < realrun[5][j]:
                    maximumrun[2][i]=realrun[2][j]
                    if maximumrun[2][i]!= maximumrun[2][i-1]:
                        maximumrun[0][i] = maximumrun[0][i-1] + 1
                    else:
                        maximumrun[0][i] = maximumrun[0][i-1]
                    maximumrun[1][i]= maximumrun[1][i-1] + 1
                    maximumrun[3][i]=maximumrun[5][i-1]
                    maximumrun[4][i]=maximum_refresh_rate
                    maximumrun[5][i]=maximumrun[3][i] + maximumrun[4][i]
                    break
                elif maximumrun[5][i-1] < realrun[5][0] :
                    maximumrun[2][i]=realrun[2][j]
                    if maximumrun[2][i]!= maximumrun[2][i-1]:
                        maximumrun[0][i] = maximumrun[0][i-1] + 1
                    else:
                        maximumrun[0][i] = maximumrun[0][i-1]
                    maximumrun[1][i]= maximumrun[1][i-1] + 1
                    maximumrun[3][i]=maximumrun[5][i-1]
                    maximumrun[4][i]=maximum_refresh_rate
                    maximumrun[5][i]=maximumrun[3][i] + maximumrun[4][i]
                    break

    #df12 = pd.DataFrame(maximumrun, index=['Visit', 'Beacon', 'Static Node', 'Time entered', 'Time Stayed(Refresh Rate)','Time exited'])
    #df12.to_csv('maximumrun.csv')

    #------------------------- Calc Mismatch --------------------------------
    #---------------------------- MIN ---------------------------------------

    minrun_counter=0

    for i in range (0, min_size-1):
        if (minimumrun[5][i]>=3500 and minimumrun[5][i+1]<=3500) or i==(min_size-2):
            min_mismatch_size=i+1
            break

    #print(min_mismatch_size)

    for i in range (0, min_mismatch_size):
        if i==0 :
            minmismatch = abs(realrun[4][0]-minimumrun[4][0])
            minmismatchtotal = abs(realrun[5][0]-minimumrun[5][0])
            minrun_counter = minrun_counter + 1
            continue
        if i>1:
            if (minimumrun[2][i]==minimumrun[2][i-1]):
                continue
            while (minimumrun[2][i]!=realrun[2][minrun_counter]):
                minmismatch = minmismatch + realrun[4][minrun_counter]
                minmismatchtotal = minmismatchtotal + realrun[5][minrun_counter]
                minrun_counter = minrun_counter + 1
            minmismatch = minmismatch + abs(realrun[4][minrun_counter]-minimumrun[4][i])
            minmismatchtotal = minmismatchtotal + abs(realrun[5][minrun_counter]-minimumrun[5][i])
            minrun_counter = minrun_counter + 1

    #---------------------------- MEDIUM -----------------------------------

    medrun_counter=0

    for i in range (0, medium_size-1):
        if (mediumrun[5][i]>=3500 and mediumrun[5][i+1]<=3500):
            med_mismatch_size=i+1
            break
        elif(i==medium_size-2):
            med_mismatch_size=medium_size

    #print(med_mismatch_size)

    for i in range (0, med_mismatch_size):
        if i==0 :
            medmismatch = abs(realrun[4][0]-mediumrun[4][0])
            medmismatchtotal = abs(realrun[5][0]-mediumrun[5][0])
            medrun_counter = medrun_counter + 1
            continue
        if i>1:
            if (mediumrun[2][i]==mediumrun[2][i-1]):
                continue
            while (mediumrun[2][i]!=realrun[2][medrun_counter]):
                medmismatch = medmismatch + realrun[4][medrun_counter]
                medmismatchtotal = medmismatchtotal + realrun[5][medrun_counter]
                medrun_counter = medrun_counter + 1
            medmismatch = medmismatch + abs(realrun[4][medrun_counter]-mediumrun[4][i])
            medmismatchtotal = medmismatchtotal + abs(realrun[5][medrun_counter]-mediumrun[5][i])
            medrun_counter = medrun_counter + 1

    #---------------------------- MAX ---------------------------------------

    maxrun_counter=0

    for i in range (0, max_size-1):
        if (maximumrun[5][i]>=3500 and maximumrun[5][i+1]<=3500):
            max_mismatch_size=i+1
            break
        elif  i==(max_size-2) :
            max_mismatch_size=i+2

    #print(max_mismatch_size)

    for i in range (0, max_mismatch_size):
        if i==0 :
            maxmismatch = abs(realrun[4][0]-maximumrun[4][0])
            maxmismatchtotal = abs(realrun[5][0]-maximumrun[5][0])
            maxrun_counter = maxrun_counter + 1
            continue
        if i>1:
            if (maximumrun[2][i]==maximumrun[2][i-1]):
                continue
            while (maximumrun[2][i]!=realrun[2][maxrun_counter]):
                maxmismatch = maxmismatch + realrun[4][maxrun_counter]
                maxmismatchtotal = maxmismatchtotal + realrun[5][maxrun_counter]
                maxrun_counter = maxrun_counter + 1
            maxmismatch = maxmismatch + abs(realrun[4][maxrun_counter]-maximumrun[4][i])
            maxmismatchtotal = maxmismatchtotal + abs(realrun[5][maxrun_counter]-maximumrun[5][i])
            maxrun_counter = maxrun_counter + 1

    #---------------------------- AVERAGE ---------------------------------------

    averun_counter=0

    for i in range (0, result[0][0]-1):
        if (averagerun[5][i]>=3498 and averagerun[5][i+1]==0):
            ave_mismatch_size=i+1
            break
        elif (i==result[0][0]-2):
            ave_mismatch_size=i+2
            
    #print(ave_mismatch_size)

    for i in range (0, ave_mismatch_size):
        if i==0 :
            avermismatch = abs(realrun[4][0]-averagerun[4][0])
            avermismatchtotal = abs(realrun[5][0]-averagerun[5][0])
            averun_counter = averun_counter + 1
            continue
        if i>1:
            if (averagerun[2][i]==averagerun[2][i-1]):
                continue
            while (averagerun[2][i]!=realrun[2][averun_counter]):
                avermismatch = avermismatch + realrun[4][averun_counter]
                avermismatchtotal = avermismatchtotal + realrun[5][averun_counter]
                averun_counter = averun_counter + 1
            avermismatch = avermismatch + abs(realrun[4][averun_counter]-averagerun[4][i])
            avermismatchtotal = avermismatchtotal + abs(realrun[5][averun_counter]-averagerun[5][i])
            averun_counter = averun_counter + 1

    #beacons
    totalmismatch[num][0]=result[0][0]
    totalmismatch[num][1]=minmismatch
    totalmismatch[num][2]=minmismatchtotal
    totalmismatch[num][3]=medmismatch
    totalmismatch[num][4]=medmismatchtotal
    totalmismatch[num][5]=maxmismatch
    totalmismatch[num][6]=maxmismatchtotal
    totalmismatch[num][7]=avermismatch
    totalmismatch[num][8]=avermismatchtotal

    #--------------------------- Calc Cost ----------------------------------
    #---------------------------- REAL ---------------------------------------

    #vehicle
    #print(num)
    #real beacons
    #print(result[0][0])

    coordinates = open("mobility50_1.txt", "r")
    #Set when the experiment started
    exp_start = 15
    exp_beacons = exp_start * vehicles
    neighbour_count_real = 0
    neighbour_count_min = 0
    neighbour_count_med = 0
    neighbour_count_max = 0
    neighbour_count_aver = 0
    
    for num_beacon in range (0, result[0][0]-1):
                
        if num_beacon==0:
            particular_line = linecache.getline('mobility50_1.txt', num+1)
            data = particular_line.split()
            #The coordinates where the vehicle was created x = data[2] y = data[3]
            a = np.array((int(data[2]), int(data[3])))
            for i in range (0, vehicles):
                if i != num:
                    particular_line = linecache.getline('mobility50_1.txt', i+1)
                    data = particular_line.split()
                    b = np.array((int(data[2]), int(data[3])))
                    if ((((a[0] - b[0])**2 + (a[1] - b[1])**2)**0.5)<=mobile_range):
                        neighbour_count_real = neighbour_count_real + 1
            for i in range (0, static_nodes):
                if ((((a[0] - static_nodes_coordinates[i][0])**2 + (a[1] - static_nodes_coordinates[i][1])**2)**0.5)<=mobile_range):
                    neighbour_count_real = neighbour_count_real + 1
            totalcost[0][num]= 3+(2*int(neighbour_count_real))
        elif num_beacon>0:
            #750beacons if we had 15 sec of coordinates (15*50)
            particular_line = linecache.getline('mobility50_1.txt', ((realrun[3][num_beacon]+exp_start)*vehicles)+num+1-exp_beacons)
            data = particular_line.split()
            #The coordinates where the vehicle was created x = data[2] y = data[3]
            a = np.array((int(data[2]), int(data[3])))
            for i in range (0, vehicles):
                if i != num:
                    particular_line = linecache.getline('mobility50_1.txt', ((realrun[3][num_beacon]+exp_start)*vehicles)+i+1)
                    data = particular_line.split()
                    b = np.array((int(data[2]), int(data[3])))
                    if ((((a[0] - b[0])**2 + (a[1] - b[1])**2)**0.5)<=mobile_range):
                        neighbour_count_real = neighbour_count_real + 1
            for i in range (0, static_nodes):
                if ((((a[0] - static_nodes_coordinates[i][0])**2 + (a[1] - static_nodes_coordinates[i][1])**2)**0.5)<=mobile_range):
                    neighbour_count_real = neighbour_count_real + 1
            totalcost[0][num]= totalcost[0][num] + (3+(2*neighbour_count_real))

    for num_beacon in range (0, min_mismatch_size-1):
                
        if num_beacon==0:
            particular_line = linecache.getline('mobility50_1.txt', num+1)
            data = particular_line.split()
            #The coordinates where the vehicle was created x = data[2] y = data[3]
            a = np.array((int(data[2]), int(data[3])))
            for i in range (0, vehicles):
                if i != num:
                    particular_line = linecache.getline('mobility50_1.txt', i+1)
                    data = particular_line.split()
                    b = np.array((int(data[2]), int(data[3])))
                    if ((((a[0] - b[0])**2 + (a[1] - b[1])**2)**0.5)<=mobile_range):
                        neighbour_count_min = neighbour_count_min + 1
            for i in range (0, static_nodes):
                if ((((a[0] - static_nodes_coordinates[i][0])**2 + (a[1] - static_nodes_coordinates[i][1])**2)**0.5)<=mobile_range):
                    neighbour_count_min = neighbour_count_min + 1
            totalcost[1][num]= 3+(2*int(neighbour_count_min))
        elif num_beacon>0:
            #750beacons if we had 15 sec of coordinates (15*50)
            particular_line = linecache.getline('mobility50_1.txt', ((minimumrun[3][num_beacon]+exp_start)*vehicles)+num+1-exp_beacons)
            data = particular_line.split()
            #The coordinates where the vehicle was created x = data[2] y = data[3]
            a = np.array((int(data[2]), int(data[3])))
            for i in range (0, vehicles):
                if i != num:
                    particular_line = linecache.getline('mobility50_1.txt', ((minimumrun[3][num_beacon]+exp_start)*vehicles)+i+1)
                    data = particular_line.split()
                    b = np.array((int(data[2]), int(data[3])))
                    if ((((a[0] - b[0])**2 + (a[1] - b[1])**2)**0.5)<=mobile_range):
                        neighbour_count_min = neighbour_count_min + 1
            for i in range (0, static_nodes):
                if ((((a[0] - static_nodes_coordinates[i][0])**2 + (a[1] - static_nodes_coordinates[i][1])**2)**0.5)<=mobile_range):
                    neighbour_count_min = neighbour_count_min + 1
            totalcost[1][num]= totalcost[1][num] + (3+(2*neighbour_count_min))

    for num_beacon in range (0, med_mismatch_size-1):
                
        if num_beacon==0:
            particular_line = linecache.getline('mobility50_1.txt', num+1)
            data = particular_line.split()
            #The coordinates where the vehicle was created x = data[2] y = data[3]
            a = np.array((int(data[2]), int(data[3])))
            for i in range (0, vehicles):
                if i != num:
                    particular_line = linecache.getline('mobility50_1.txt', i+1)
                    data = particular_line.split()
                    b = np.array((int(data[2]), int(data[3])))
                    if ((((a[0] - b[0])**2 + (a[1] - b[1])**2)**0.5)<=mobile_range):
                        neighbour_count_med = neighbour_count_med + 1
            for i in range (0, static_nodes):
                if ((((a[0] - static_nodes_coordinates[i][0])**2 + (a[1] - static_nodes_coordinates[i][1])**2)**0.5)<=mobile_range):
                    neighbour_count_med = neighbour_count_med + 1
            totalcost[2][num]= 3+(2*int(neighbour_count_med))
        elif num_beacon>0:
            #750beacons if we had 15 sec of coordinates (15*50)
            particular_line = linecache.getline('mobility50_1.txt', ((mediumrun[3][num_beacon]+exp_start)*vehicles)+num+1-exp_beacons)
            data = particular_line.split()
            #The coordinates where the vehicle was created x = data[2] y = data[3]
            a = np.array((int(data[2]), int(data[3])))
            for i in range (0, vehicles):
                if i != num:
                    particular_line = linecache.getline('mobility50_1.txt', ((mediumrun[3][num_beacon]+exp_start)*vehicles)+i+1)
                    data = particular_line.split()
                    b = np.array((int(data[2]), int(data[3])))
                    if ((((a[0] - b[0])**2 + (a[1] - b[1])**2)**0.5)<=mobile_range):
                        neighbour_count_med = neighbour_count_med + 1
            for i in range (0, static_nodes):
                if ((((a[0] - static_nodes_coordinates[i][0])**2 + (a[1] - static_nodes_coordinates[i][1])**2)**0.5)<=mobile_range):
                    neighbour_count_med = neighbour_count_med + 1
            totalcost[2][num]= totalcost[2][num] + (3+(2*neighbour_count_med))

    for num_beacon in range (0, max_mismatch_size-1):
                
        if num_beacon==0:
            particular_line = linecache.getline('mobility50_1.txt', num+1)
            data = particular_line.split()
            #The coordinates where the vehicle was created x = data[2] y = data[3]
            a = np.array((int(data[2]), int(data[3])))
            for i in range (0, vehicles):
                if i != num:
                    particular_line = linecache.getline('mobility50_1.txt', i+1)
                    data = particular_line.split()
                    b = np.array((int(data[2]), int(data[3])))
                    if ((((a[0] - b[0])**2 + (a[1] - b[1])**2)**0.5)<=mobile_range):
                        neighbour_count_max = neighbour_count_max + 1
            for i in range (0, static_nodes):
                if ((((a[0] - static_nodes_coordinates[i][0])**2 + (a[1] - static_nodes_coordinates[i][1])**2)**0.5)<=mobile_range):
                    neighbour_count_max = neighbour_count_max + 1
            totalcost[3][num]= 3+(2*int(neighbour_count_max))
        elif num_beacon>0:
            #750beacons if we had 15 sec of coordinates (15*50)
            particular_line = linecache.getline('mobility50_1.txt', ((maximumrun[3][num_beacon]+exp_start)*vehicles)+num+1-exp_beacons)
            data = particular_line.split()
            #The coordinates where the vehicle was created x = data[2] y = data[3]
            a = np.array((int(data[2]), int(data[3])))
            for i in range (0, vehicles):
                if i != num:
                    particular_line = linecache.getline('mobility50_1.txt', ((maximumrun[3][num_beacon]+exp_start)*vehicles)+i+1)
                    data = particular_line.split()
                    b = np.array((int(data[2]), int(data[3])))
                    if ((((a[0] - b[0])**2 + (a[1] - b[1])**2)**0.5)<=mobile_range):
                        neighbour_count_max = neighbour_count_max + 1
            for i in range (0, static_nodes):
                if ((((a[0] - static_nodes_coordinates[i][0])**2 + (a[1] - static_nodes_coordinates[i][1])**2)**0.5)<=mobile_range):
                    neighbour_count_max = neighbour_count_max + 1
            totalcost[3][num]= totalcost[3][num] + (3+(2*neighbour_count_max))

    for num_beacon in range (0, ave_mismatch_size-1):
                
        if num_beacon==0:
            particular_line = linecache.getline('mobility50_1.txt', num+1)
            data = particular_line.split()
            #The coordinates where the vehicle was created x = data[2] y = data[3]
            a = np.array((int(data[2]), int(data[3])))
            for i in range (0, vehicles):
                if i != num:
                    particular_line = linecache.getline('mobility50_1.txt', i+1)
                    data = particular_line.split()
                    b = np.array((int(data[2]), int(data[3])))
                    if ((((a[0] - b[0])**2 + (a[1] - b[1])**2)**0.5)<=mobile_range):
                        neighbour_count_aver = neighbour_count_aver + 1
            for i in range (0, static_nodes):
                if ((((a[0] - static_nodes_coordinates[i][0])**2 + (a[1] - static_nodes_coordinates[i][1])**2)**0.5)<=mobile_range):
                    neighbour_count_aver = neighbour_count_aver + 1
            totalcost[4][num]= 3+(2*int(neighbour_count_aver))
        elif num_beacon>0:
            #750beacons if we had 15 sec of coordinates (15*50)
            particular_line = linecache.getline('mobility50_1.txt', ((averagerun[3][num_beacon]+exp_start)*vehicles)+num+1-exp_beacons)
            data = particular_line.split()
            #The coordinates where the vehicle was created x = data[2] y = data[3]
            a = np.array((int(data[2]), int(data[3])))
            for i in range (0, vehicles):
                if i != num:
                    particular_line = linecache.getline('mobility50_1.txt', ((averagerun[3][num_beacon]+exp_start)*vehicles)+i+1)
                    data = particular_line.split()
                    b = np.array((int(data[2]), int(data[3])))
                    if ((((a[0] - b[0])**2 + (a[1] - b[1])**2)**0.5)<=mobile_range):
                        neighbour_count_aver = neighbour_count_aver + 1
            for i in range (0, static_nodes):
                if ((((a[0] - static_nodes_coordinates[i][0])**2 + (a[1] - static_nodes_coordinates[i][1])**2)**0.5)<=mobile_range):
                    neighbour_count_aver = neighbour_count_aver + 1
            totalcost[4][num]= totalcost[4][num] + (3+(2*neighbour_count_aver))
 
# -------------------- CALC AVER/SD COST ---------------------------

df13 = pd.DataFrame(totalcost, index=['Real run', 'Minimum run', 'Medium run', 'Max run', 'Average run'])
df13.to_csv('totalcost.csv')

"""
# -------------------- CALC AVER/SD MISMATCH ---------------------------

#df14 = pd.DataFrame(totalmismatch, columns=['Real Beacons', 'Minmismatch', 'Minmismatchtotal', 'Medmismatch', 'Medmismatchtotal', 'Maxmismatch', 'Maxmismatchtotal', 'Avermismatch','Avermismatchtotal'])
#df14.to_csv('totalmismatch.csv')

#swap line-columns for easier calculation of SD
swapped_totalmismatch = [list(x) for x in zip(*totalmismatch)]

sd_mismatch=[0,0,0,0]
average_mismatch = [0,0,0,0]
j=0

for i in range (1, 9, 2):
    #Calculate the standard deviation from a sample of data
    #print(round(statistics.stdev(swapped_totalmismatch[i]),1))
    sd_mismatch[j] = round(statistics.stdev(swapped_totalmismatch[i]),1)
    average_mismatch[j] = round(mean(swapped_totalmismatch[i]),1)
    j = j + 1

# -------------------------- GUI START ------------------------------------

data1 = {'Minimum RR': average_mismatch[0], 'Medium RR': average_mismatch[1], 'Maximum RR': average_mismatch[2], 'Average RR': average_mismatch[3]}
annotate1 = [average_mismatch[0], average_mismatch[1], average_mismatch[2], average_mismatch[3]]
rr1 = list(data1.keys())
values1 = list(data1.values())

plt.bar(rr1, values1, color=['red', 'green', 'blue', 'cyan'])
plt.errorbar(rr1, values1, yerr = sd_mismatch, fmt='o')

for i in range (0,4):
    plt.annotate(round(annotate1[i],1), xy=(rr1[i],values1[i]), ha='center', va='top')
    plt.annotate(round(sd_mismatch[i],1), xy=(rr1[i],values1[i]), ha='center', va='bottom')

plt.title("Mismatch")
plt.ylabel("Seconds")
plt.show()

# -------------------------- GUI END ------------------------------------
"""