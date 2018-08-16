import os
import pickle
from wpilib.command import CommandGroup
from importlib.resources import path

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
            
      
for key in path2:
    for data in (path2[key]):       
        #print(data)
        path1[key].append(data)
        
for key in path3:
    for data in (path3[key]):       
        #print(path2[key][key2])
        path1[key].append(data)
        
for key in path4:
    for data in (path4[key]):      
        #print(path2[key][key2])
        path1[key].append(data)        
        
with open(os.path.join(os.path.dirname(__file__), "best_path.pickle"), "wb") as fp:
    pickle.dump(path1, fp)       
#===============================================================================
# print(len(path[key]))
# print(len(path2[key]))
# print(len(path1[key]))
#===============================================================================