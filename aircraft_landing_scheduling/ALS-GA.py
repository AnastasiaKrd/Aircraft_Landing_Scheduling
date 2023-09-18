'''  Aircraft Landing Scheduling (Static Case) - GA '''
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
    flag=0
    count=0
    for items in [s[x:x+6+num_planes] for x in range(0,len(s),num_planes+6)]:
        plane_details[count]=[float(x) for x in items[:6]]
        sep_time[count]=[int(x) for x in items[6:]]
        count=count+1
    data.close()
    return num_planes,plane_details,sep_time

# case 1 - create random first generation
def initialize_generation(num_planes, plane_details, sep_time, size):
    first_gen = np.stack([np.random.choice(range(1, num_planes+1), num_planes, replace=False) for _ in range(size)])
    return first_gen
# case 2 - create one solution FCFS (target landing time)
'''def initialize_generation(num_planes,plane_details, sep_time, size):
    first_gen = np.stack([np.random.choice(range(1, num_planes+1), num_planes, replace=False) for _ in range(size-1)])
    sorted_planes = plane_details[plane_details[:,0].argsort()]
    fcfs = np.zeros(num_planes)
    for i in range(0, num_planes, 1):
        for j in range(0, num_planes, 1):
           if sorted_planes[i,0] == plane_details[j,0]:
               fcfs[i] = j+1
    first_gen = np.row_stack((first_gen, fcfs.astype(int)))
    print ('First come fist served :',fcfs)
    return first_gen '''
# case 3 - create 3 solutions (earliest l.t, target l.t., latest l.t.)
'''def initialize_generation(num_planes,plane_details, sep_time, size):
    first_gen = np.stack([np.random.choice(range(1, num_planes+1), num_planes, replace=False) for _ in range(size-3)])
    sorted_elt = plane_details[plane_details[:,1].argsort()]
    sorted_tlt = plane_details[plane_details[:,2].argsort()]
    sorted_llt = plane_details[plane_details[:,3].argsort()]
    earliest_lt = np.zeros(num_planes)
    target_lt = np.zeros(num_planes)
    latest_lt = np.zeros(num_planes)
    for i in range(0, num_planes, 1):
        for j in range(0, num_planes, 1):
           if sorted_elt[i,1] == plane_details[j,1]:
               earliest_lt[i] = j+1
           if sorted_tlt[i,2] == plane_details[j,2]:
               target_lt[i] = j+1 
           if sorted_llt[i,3] == plane_details[j,3]:
               latest_lt[i] = j+1    
    first_gen = np.row_stack((first_gen, earliest_lt.astype(int), target_lt.astype(int), latest_lt.astype(int)))
    print ('Sorted earliest l.t. :',earliest_lt, '\n Sorted taget l.t', target_lt, 'Sorted latest l.t', latest_lt)
    return first_gen'''
    
# fitness function
def evaluate_fitness(population_size, num_planes, plane_details, sep_time, generation):
    landing_time = np.zeros((population_size, num_planes))
    for i in range(0, population_size, 1):
        landing_time[i, 0] = plane_details[generation[i,0]-1, 2]  #first plane lands at target landing time        
        for j in range(1, num_planes, 1): 
            if landing_time[i, j-1] + sep_time[generation[i,j-1]-1, generation[i,j]-1] < plane_details[generation[i,j]-1, 1]:
                landing_time[i, j] = plane_details[generation[i,j]-1, 2]
            else:
                landing_time[i, j] = landing_time[i, j-1] + sep_time[generation[i,j-1]-1, generation[i,j]-1]   
  
    distance = np.zeros((population_size, num_planes))
    count_invalid = np.zeros(population_size)
    fitness = np.zeros(population_size)  
    for i in range(0, population_size, 1):
        for j in range(0, num_planes, 1):
            distance[i,j] = landing_time[i,j] - plane_details[generation[i,j]-1, 2]  #landing time - target ld
            if landing_time[i,j] > plane_details[generation[i,j]-1, 3]:
                count_invalid[i]+=1
            if distance[i,j] > 0:    #lands after target time
                fitness[i] += distance[i,j] * plane_details[generation[i,j]-1, 5]
            elif distance[i,j] < 0:    #lands before target time
                fitness[i] -= distance[i,j] * plane_details[generation[i,j]-1, 4]
        fitness[i] += count_invalid[i] * 10000    #penalty for invalid landing times       
        fitness[i] = 1 / fitness[i]
    max_fitness = np.amax(fitness)
    index = np.where(fitness == max_fitness)
    max_index = index[0][0].astype(int)
    sum_fitness = np.sum(fitness)    
    return landing_time, count_invalid, fitness , max_fitness, max_index, sum_fitness

#Roulette wheel selection
def roulette(fitness, population_size, sum_fitness):      
    random_number = random.uniform(0, sum_fitness)
    add_fitness = np.zeros(population_size)
    i=0
    add_fitness[0] = fitness[0]
    while random_number > add_fitness[i]:
        add_fitness[i+1] = add_fitness[i] + fitness[i+1]
        i+=1
    return i

#Partially mapped crossover
def crossover(fitness, population_size, generation, num_planes, sum_fitness):
    index_p1 = roulette(fitness, population_size, sum_fitness)    
    index_p2 = roulette(fitness, population_size, sum_fitness) 
    while index_p1 == index_p2:
        index_p2 = roulette(fitness, population_size, sum_fitness)    
    random_array = np.random.randint(0, num_planes, size=2)
    cut_point1 = np.amin(random_array)  
    cut_point2 = np.amax(random_array) 
    parent1 = np.copy(generation[index_p1]) 
    parent2 = np.copy(generation[index_p2]) 
    offspring1 = np.empty(num_planes)    
    offspring2 = np.empty(num_planes)
    for i in range(cut_point1, cut_point2+1, 1):
        offspring1[i] = parent2[i]
        offspring2[i] = parent1[i]
        
    #offspring1[cut_point1:cut_point2] = np.copy(parent2[cut_point1:cut_point2])
    #offspring2[cut_point1:cut_point2] = np.copy(parent1[cut_point1:cut_point2])
    if cut_point1 != 0:
        for i in range(0, cut_point1, 1):
            index = i
            while parent1[index] in offspring1[cut_point1:cut_point2+1]:
                index = np.where(parent2 == parent1[index])
            offspring1[i] = parent1[index]
            index = i
            while parent2[index] in offspring2[cut_point1:cut_point2+1]:
                index = np.where(parent1 == parent2[index])
            offspring2[i] = parent2[index]            
    if cut_point2 != num_planes-1:
        for i in range(cut_point2+1, num_planes, 1):
            index = i
            while parent1[index] in offspring1[cut_point1:cut_point2+1]:
                index = np.where(parent2 == parent1[index])
            offspring1[i] = parent1[index]
            index = i
            while parent2[index] in offspring2[cut_point1:cut_point2+1]:
                index = np.where(parent1 == parent2[index])
            offspring2[i] = parent2[index]                 
    return  offspring1.astype(int), offspring2.astype(int)

# Create new generation
def new_generation(fitness, population_size, generation, num_planes, sum_fitness):
    new_gen = np.empty((population_size, num_planes))
    d = population_size // 2
    for i in range(0, d, 1):
        new_gen[i], new_gen[d+i] = crossover(fitness, population_size, generation, num_planes, sum_fitness)
    if  population_size % 2 != 0:
        a, b = crossover(fitness, population_size, generation, num_planes, sum_fitness)
        new_gen[population_size-1] = a
    return new_gen.astype(int)     
    
#Mutation - swap    
def mutation(generation, mutation_prob, num_planes, population_size):
    for i in range(0, population_size, 1):
        p = random.uniform(0, 1)
        if p < mutation_prob:
            points = np.random.choice(range(0,num_planes), 2, replace=False)
            (generation[i, points[0]], generation[i, points[1]]) = (generation[i, points[1]], generation[i, points[0]] )    
    return generation.astype(int)                

#Survival of the best individuals
def best_individuals(first_gen, fitness, new_gen, new_fitness, num_planes, population_size):
    array_all = np.row_stack((np.column_stack((first_gen, fitness)), np.column_stack((new_gen, new_fitness))))  
    sorted_array = array_all[array_all[:, num_planes].argsort()[::-1]]
    new_gen = np.empty((population_size, num_planes))
    new_gen[:] = sorted_array[:population_size, :num_planes] 
    return new_gen.astype(int)








def main():
    population_size = 5
    mutation_prob = 0.01
    num_generations = 10
    num_planes,plane_details,sep_time = get_data('airland1.txt')
    print('Number of planes : ', num_planes)    
    print('Plane details : appearance time, earliest l.t., target l.t., latest l.t., penalty before t.l.t., penalty after t.l.t. :\n',plane_details)
    print("Separation time :\n",sep_time)
    first_gen = initialize_generation(num_planes, plane_details, sep_time, population_size)
    print('First generation :\n', first_gen)
    best_lt, count_invalid, fitness, best_fitness, best_index, sum_fitness = evaluate_fitness(population_size, num_planes,plane_details, sep_time, first_gen)
    best_solution = first_gen[best_index]
    iteration = 0
    invalid = count_invalid[best_index]
    
    
    for gen in range(0, num_generations, 1):        
        new_gen = new_generation(fitness, population_size, first_gen, num_planes, sum_fitness)
        new_gen = mutation(new_gen, mutation_prob, num_planes, population_size)
        landing_time,count_invalid, new_fitness , max_fitness, max_index, sum_fitness = evaluate_fitness(population_size, num_planes,plane_details, sep_time, new_gen)
        new_gen = best_individuals(first_gen, fitness, new_gen, new_fitness, num_planes, population_size)
        print('New generation (best individuals)', gen+1,'\n', new_gen,)
        landing_time,count_invalid, new_fitness, max_fitness, max_index, sum_fitness = evaluate_fitness(population_size, num_planes,plane_details, sep_time, new_gen)
        if best_fitness < max_fitness:
            best_fitness = max_fitness
            best_index = max_index
            best_solution = first_gen[best_index]
            iteration = gen+1
            best_lt = landing_time
            invalid = count_invalid[best_index]
        first_gen = new_gen
        fitness = new_fitness
    print('Best solution \n', best_solution, '\n in generation \n', iteration)    
    if invalid != 0:
        print('Infeasible solution')
    else:
        print('Feasible solution')
    print('\n Fitness \n', best_fitness)
    print('\n Landing times \n', best_lt[best_index])
    
        
if __name__ == "__main__":
    main()    