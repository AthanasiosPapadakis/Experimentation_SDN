import sys
import numpy as np
np.set_printoptions(threshold=sys.maxsize)
from tkinter import *
from tkinter.ttk import *
import matplotlib.pyplot as plt
import pandas as pd

vehicles = 50
static_nodes = 24
maximum_refresh_rate=60
minimum_refresh_rate=10
medium_refresh_rate=20
perfectcounter = 0

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

while True:
    num = input("Enter a number: ")
    try:
        val = int(num)
        if val < 0 or val > vehicles:  # if not a positive int print message and ask for input again
            print("Sorry, input must be a positive integer and less than 50, try again")
            continue
        break
    except ValueError:
        print("That's not an int!")     

num = int(num) -1

#totalmismatch = np.array(range(vehicles * result[0][0]))
#totalmismatch = totalmismatch.reshape(vehicles, result[0][0])

#for k in range (0, vehicles):
    #k = k - 1
    #result = np.where(sequence[int(k)] == 0)

#Find the size of array
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

df8 = pd.DataFrame(realrun, index=['Visit', 'Beacon', 'Static Node', 'Time entered', 'Time Stayed(Refresh Rate)','Time exited'])
df8.to_csv('realrun.csv')

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

df9 = pd.DataFrame(averagerun, index=['Visit', 'Beacon', 'Static Node', 'Time entered', 'Time Stayed(Refresh Rate)','Time exited'])
df9.to_csv('averagerun.csv')

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

df10 = pd.DataFrame(minimumrun, index=['Visit', 'Beacon', 'Static Node', 'Time entered', 'Time Stayed(Refresh Rate)','Time exited'])
df10.to_csv('minimumrun.csv')

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

df11 = pd.DataFrame(mediumrun, index=['Visit', 'Beacon', 'Static Node', 'Time entered', 'Time Stayed(Refresh Rate)','Time exited'])
df11.to_csv('mediumrun.csv')

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

df12 = pd.DataFrame(maximumrun, index=['Visit', 'Beacon', 'Static Node', 'Time entered', 'Time Stayed(Refresh Rate)','Time exited'])
df12.to_csv('maximumrun.csv')

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

"""
print (num+1)
print (result[0][0])
print (minmismatch)
print (minmismatchtotal)    
print (medmismatch)
print (medmismatchtotal)   
print (maxmismatch)
print (maxmismatchtotal)  
print (avermismatch)
print (avermismatchtotal)    
"""


#print(ave_mismatch_size)
averagerun2=averagerun[0:6, 0:(ave_mismatch_size)]
#np.resize(averagerun,(5, ave_mismatch_size-1))
#print(averagerun2)

# -------------------------- GUI START ------------------------------------

# creates a Tk() object
master = Tk()

# sets the geometry of main
# root window
master.geometry("200x200")

label = Label(master,
              text="This is the main window")
label.pack(pady=10)

def graphs():
    # Add Title
    plt.title("Comparison between the selected mobile node and the average ")

    # Add Axes Labels
    plt.xlabel("Total time passed")
    plt.ylabel("Beacon timing (sec)")
    
    plt.plot(realrun[5],realrun[4], label = "Perfect", marker = 'P')
    plt.plot(averagerun2[5],averagerun2[4], label = "Average", marker = '*')
        
    # show a legend on the plot
    plt.legend()
    
    # setting x and y axis range
    plt.xticks(np.arange(0, 3800, 150))
    plt.yticks(np.arange(0, 100, 10))
    plt.show()

# a button widget which will open a
# new window on button click
btn = Button(master,
             text ="Show graphs",
             command = graphs)
btn.pack(pady = 10)

def info():

    # Add Title
    plt.title("Info")

    plt.text(0.1, 0.9, "The vehicle: " + str(int(num)+1))
    plt.text(0.5, 0.9, "Total real beacons: " + str(result[0][0]))
    plt.text(0.01, 0.8, "Refresh rate = minimum (10s) the mismatch per beacon is: {:.1f}s".format(minmismatch/result[0][0]))
    plt.text(0.01, 0.7, "Refresh rate = minimum (10s) the total mismatch per beacon is: {:.1f}s".format(minmismatchtotal/result[0][0]))
    plt.text(0.01, 0.6, "Refresh rate = medium (20s) the mismatch per beacon is: {:.1f}s".format(medmismatch/result[0][0]))
    plt.text(0.01, 0.5, "Refresh rate = medium (20s) the total mismatch per beacon is: {:.1f}s".format(medmismatchtotal/result[0][0]))
    plt.text(0.01, 0.4, "Refresh rate = max (60s) the mismatch per beacon is: {:.1f}s".format(maxmismatch/result[0][0]))
    plt.text(0.01, 0.3, "Refresh rate = max (60s) the total mismatch per beacon is: {:.1f}s".format(maxmismatchtotal/result[0][0]))
    plt.text(0.01, 0.2, "Refresh rate = average the mismatch per beacon is: {:.1f}s".format(avermismatch/result[0][0]))
    plt.text(0.01, 0.1, "Refresh rate = average the total mismatch per beacon is: {:.1f}s".format(avermismatchtotal/result[0][0]))
    plt.show()

# a button widget which will open a
# new window on button click
btn = Button(master,
             text ="Info",
             command = info)
btn.pack(pady = 10)

def info_total_graphs():

    # Add Title
    plt.title("info graphs")
    
    data1 = {'Minimum RR': minmismatch/result[0][0], 'Medium RR': medmismatch/result[0][0], 'Maximum RR': maxmismatch/result[0][0], 'Average RR': avermismatch/result[0][0]}
    #data2 = {'Min RR': minmismatchtotal/result[0][0], 'Medium RR': medmismatchtotal/result[0][0], 'Max RR': maxmismatchtotal/result[0][0], 'Aver RR': avermismatchtotal/result[0][0]}

    annotate1 = [minmismatch/result[0][0], medmismatch/result[0][0], maxmismatch/result[0][0], avermismatch/result[0][0]]
    #annotate2 = [minmismatchtotal/result[0][0], medmismatchtotal/result[0][0], maxmismatchtotal/result[0][0], avermismatchtotal/result[0][0]]

    rr1 = list(data1.keys())
    values1 = list(data1.values())
    #rr2 = list(data2.keys())
    #values2 = list(data2.values())

    plt.bar(rr1, values1, color=['red', 'green', 'blue', 'cyan'])
    #plt.bar(rr2, values2, color=['red', 'green', 'blue', 'cyan'])

    for i in range (0,4):
        plt.annotate(round(annotate1[i],1), xy=(rr1[i],values1[i]), ha='center', va='bottom')
        #plt.annotate(round(annotate2[i],1), xy=(rr2[i],values2[i]), ha='center', va='bottom')

    plt.ylabel("Seconds")
    plt.show()

# a button widget which will open a new window on button click
btn = Button(master,
             text ="Info total graphs",
             command = info_total_graphs)
btn.pack(pady = 10)

# mainloop, runs infinitely to show the window
mainloop()
# ------------------------------GUI END-----------------------------------------
