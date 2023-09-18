'''  Aircraft Landing Scheduling (Static Case) - VNS'''
import numpy as np
import os
import random

# function for data txt
def get_data(file_name):
    data=open(os.getcwd()+'\\data_files\\'+file_name,'r')
    lines=data.readlines()
    num_planes=int(lines[0].split()[0])
    freeze_time=int(lines[0].split()[1])
    plane_details=np.empty([num_planes,6],dtype=float)
    sep_time=np.empty([num_planes,num_planes],dtype=int)
    s=''
    for line in lines[1:]:
        s=s+line
    s=s.split()
    count=0
    for items in [s[x:x+6+num_planes] for x in range(0,len(s),num_planes+6)]:
        plane_details[count]=[float(x) for x in items[:6]]
        sep_time[count]=[int(x) for x in items[6:]]
        count=count+1
    data.close()
    return num_planes,plane_details,sep_time


# first solution
def initial_solution(num_planes,plane_details):
    sorted_planes = plane_details[plane_details[:,0].argsort()]
    x0 = np.zeros(num_planes)
    for i in range(0, num_planes, 1):
        for j in range(0, num_planes, 1):
           if sorted_planes[i,0] == plane_details[j,0]:
               x0[i] = j+1
    print ('First solution :',x0)
    return x0 



def swap(x0, num_planes, point1, point2):
    x1 = np.copy(x0)
    (x1[point1], x1[point2]) = (x1[point2], x1[point1])    
    return x1  
    


def relocate(x0, num_planes, point1, point2):
    x1 = np.copy(x0)
    x1[point2] = x0[point1]
    x1[point1:point2] = x0[point1+1:point2+1]   
    return x1    
        
def opt2(x0, num_planes, point1, point2):
    x1 = np.copy(x0)
    x1[point1:point2+1] = np.fliplr([x0[point1:point2+1]])[0]
    return x1
    
    

        
def single_fitness(y, num_planes, plane_details, sep_time):
    y = y.astype(int)
    landing_time = np.zeros(num_planes)
    landing_time[0] = plane_details[y[0]-1, 2]
    for j in range(1, num_planes, 1): 
            if landing_time[j-1] + sep_time[y[j-1]-1, y[j]-1] < plane_details[y[j]-1, 1]:
                landing_time[j] = plane_details[y[j]-1, 2]
            else:
                landing_time[j] = landing_time[j-1] + sep_time[y[j-1]-1, y[j]-1]   
    distance = np.zeros(num_planes)
    count_invalid = 0
    fitness = 0 
    for j in range(0, num_planes, 1):
        distance[j] = landing_time[j] - plane_details[y[j]-1, 2]  #landing time - target ld
        if landing_time[j] > plane_details[y[j]-1, 3]:
            count_invalid+=1
        if distance[j] > 0:    #lands after target time
            fitness += distance[j] * plane_details[y[j]-1, 5]
        elif distance[j] < 0:    #lands before target time
            fitness -= distance[j] * plane_details[y[j]-1, 4]
    fitness += count_invalid * 10000    #penalty for invalid landing times       
    fitness = 1 / fitness
    return landing_time, count_invalid, fitness


def vns(x0, num_planes, plane_details, sep_time):         
    functions = (swap, relocate, opt2)
    tmax = False
    while tmax == False:
        for fnc in functions:
            first_improvement = False
            points = np.random.choice(range(0, num_planes), size = 2, replace=False)
            point1 = np.amin(points)
            point2 = np.amax(points)
            x1 = fnc(x0, num_planes, point1, point2)
            for i in range (num_planes):
                for j in range(i+1, num_planes):
                    x2 = fnc(x1, num_planes, point1, point2)
                    l_time0, invalid0, x0fit = single_fitness(x0, num_planes, plane_details, sep_time)
                    l_time2, invalid2, x2fit = single_fitness(x2, num_planes, plane_details, sep_time)
                    if x2fit > x0fit:
                        x0 = x2
                        x0fit = x2fit
                        l_time0 = l_time2
                        invalid0 = invalid2
                        first_improvement = True
                    if first_improvement == True:
                        break
                else:
                    continue
                break
            if fnc is opt2 :
                tmax = True                     
    return x0, x0fit, l_time0, invalid0        





def main():
    num_planes,plane_details,sep_time = get_data('airland2.txt') 
    x0 = initial_solution(num_planes,plane_details)
    x_best, best_fit, l_time, invalid = vns(x0, num_planes, plane_details, sep_time)
    print (x_best, best_fit, l_time)
    if invalid != 0:
        print('Infeasible solution')
    else:
        print('Feasible solution')
    
    
    
    
if __name__ == "__main__":
    main()            
    


