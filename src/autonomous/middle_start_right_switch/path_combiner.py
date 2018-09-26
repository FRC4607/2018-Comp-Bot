import os
import pickle
from wpilib.command import CommandGroup
         
with open(os.path.join(os.path.dirname(__file__), 'path1.pickle'), "rb") as fp:
    path1 = pickle.load(fp)          
with open(os.path.join(os.path.dirname(__file__), 'path2.pickle'), "rb") as fp:
    path2 = pickle.load(fp)        
with open(os.path.join(os.path.dirname(__file__), 'path3.pickle'), "rb") as fp:
    path3 = pickle.load(fp)
with open(os.path.join(os.path.dirname(__file__), 'path4.pickle'), "rb") as fp:
    path4 = pickle.load(fp)  
with open(os.path.join(os.path.dirname(__file__), 'path5.pickle'), "rb") as fp:
    path5 = pickle.load(fp)

def PathAppender(first_path, second_path, reverse=False):
 
    if reverse:
        for i in range(len(second_path['left'])):    
            second_path['left'][i][0] = -second_path['left'][i][0]
            second_path['left'][i][1] = -second_path['left'][i][1] 
               
        for i in range(len(second_path['right'])):    
            second_path['right'][i][0] = -second_path['right'][i][0]    
            second_path['right'][i][1] = -second_path['right'][i][1]    
     
    for i in range(len(first_path['left'])):    
        lastPoint = first_path['left'][-1][0]      
        lastHeading = first_path['left'][-1][2]
      
    for i in range(len(second_path['left'])):
        firstPoint = second_path['left'][0][2]
          
    for i in range(len(second_path['left'])):    
        second_path['left'][i][0] = second_path['left'][i][0] + lastPoint  
        second_path['left'][i][2] = second_path['left'][i][2] - firstPoint + lastHeading
          
    for i in range(len(first_path['right'])):    
        lastPoint = first_path['right'][-1][0]      
        lastHeading = first_path['right'][-1][2]
      
    for i in range(len(second_path['right'])):
        firstPoint = second_path['right'][0][2]
          
    for i in range(len(second_path['right'])):    
        second_path['right'][i][0] = second_path['right'][i][0] + lastPoint  
        second_path['right'][i][2] = second_path['right'][i][2] - firstPoint + lastHeading 
          
    for key in second_path:
       for data in (second_path[key]):       
           first_path[key].append(data)

#path['left'][-1][2]


PathAppender(path1, path2, True)   
PathAppender(path1, path3)
PathAppender(path1, path4, True)
PathAppender(path1, path5)

for key in path1:
    for data in (path1[key]):
        print(data)

with open(os.path.join(os.path.dirname(__file__), "best_path.pickle"), "wb") as fp:
    pickle.dump(path1, fp)
           
