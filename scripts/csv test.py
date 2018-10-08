import csv
array=[]

with open('turns.txt','r') as turns_file:
    turns_reader = csv.reader(turns_file,delimiter=',')
    #print(turns_reader(1))
    for row in turns_reader:
       array.append(row)
print(array[0][1])