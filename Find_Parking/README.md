# README for Find Parking Feature

Bolin Zhao

2019/4/2: update, add rhomboid case;

2019/3/28: first version;



This function aim to find the acceptable parking space from a quadrangle area which given by the perception system.  The area is defined by four points in the vehicle coordinate.    The final parking spot can be treated as a right-trapezoid.



- ![Find Parking example](Pics\find_park_1.jpg)





The general cases can be divided as parallel parking and vertical parking refer to the vehicle position.



The key function is [find_P.m].



Input:
Four point location: [a,b;c,d;e,f;g,h];  
 the sequence should be:

 ```    
4 3  | 4 3
1 2  | 1 2
-----*-------x
1 2  | 1 2
4 3  | 4 3
     y
 ```

The * location is the vehicle location and heading to X axis.  

Output:
 Point_Target = [x;y;yaw]. The point is the center point of the parking lot and the final vehicle heading when the car finish the parking process under the current coordinate;  
 Type : 1 for Vertical, 3 for Parallel;  
 size: [short edge, long edge] for final parking cube space.    



