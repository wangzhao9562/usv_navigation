map_loader

Download map from upper controller through mavlink in ros environment. 

Two methods to implement the assumed function:   
1. Make this pacakge becomes a superset of map_sever  
2. Realize the package in an independent way  
  
Decision:  
1. Make map_loader becomes a part of map_server, like MapGenerator invoked by node of map_server.  
2. Realize its function through mavlink.
