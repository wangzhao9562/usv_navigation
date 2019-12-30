# map_loader

Download map from upper controller through mavlink in ros environment. 

Two methods to implement the assumed function:   
1. Make this pacakge becomes a superset of map_sever  
2. Realize the package in an independent way  
  
Plan:  
1. Make map_loader becomes a part of map_server, like MapGenerator invoked by node of map_server.  
2. Realize its function through mavlink.

Interface in QGC or other control station:  
1. Design a new interface of mavlink to transfer raw map information(screenshot of Google map or Bing map from control station).    
2. Design class of MapLoader which is similar with MapGenerator and integrated with mavlink interface to receive raw map information.  
3. Raw map information is in form of char array.  
