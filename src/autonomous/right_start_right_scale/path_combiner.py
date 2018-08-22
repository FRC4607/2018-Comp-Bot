import os
import pickle
from wpilib.command import CommandGroup


with open(os.path.join(os.path.dirname(__file__), 'path.pickle'), "rb") as fp:
    path = pickle.load(fp)          
with open(os.path.join(os.path.dirname(__file__), 'path1.pickle'), "rb") as fp:
    path1 = pickle.load(fp)          
with open(os.path.join(os.path.dirname(__file__), 'path2.pickle'), "rb") as fp:
    path2 = pickle.load(fp)        
with open(os.path.join(os.path.dirname(__file__), 'path3.pickle'), "rb") as fp:
    path3 = pickle.load(fp)
with open(os.path.join(os.path.dirname(__file__), 'path4.pickle'), "rb") as fp:
    path4 = pickle.load(fp)  
    
        
for i in range(len(path2['left'])):    
    path2['left'][i][0] = -path2['left'][i][0]
    path2['left'][i][1] = -path2['left'][i][1] 
     
for i in range(len(path2['right'])):    
    path2['right'][i][0] = -path2['right'][i][0]    
    path2['right'][i][1] = -path2['right'][i][1]
     
for i in range(len(path4['left'])):    
    path4['left'][i][0] = -path4['left'][i][0]
    path4['left'][i][1] = -path4['left'][i][1] 
     
for i in range(len(path4['right'])):    
    path4['right'][i][0] = -path4['right'][i][0]    
    path4['right'][i][1] = -path4['right'][i][1]    
      
for key in path2:
    for data in (path2[key]):       
        path1[key].append(data)

for key in path1:
    for data in (path1[key]):
        print(data)
        
for key in path3:
    for data in (path3[key]):       
        path1[key].append(data)

for key in path4:
    for data in (path4[key]):         
        path1[key].append(data)         
       
with open(os.path.join(os.path.dirname(__file__), "best_path.pickle"), "wb") as fp:
    pickle.dump(path1, fp)  
with open(os.path.join(os.path.dirname(__file__), "best_path.pickle"), "rb") as fp:
    bestPath = pickle.load(fp)         
       
 
#===============================================================================
# for key in bestPath:
#     for data in (bestPath[key]):     
#         print(data)   
#===============================================================================