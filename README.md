
# Model-Predictive-Control
 **Reference: Model Predictive Control System Design and Implementation Using MATLAB**

## MPC001 is tutotial 1.3 Page 20 <br/>
System Model : <br/>
x[k+1] = A x[k] + B u[k] <br/>
y[k] = C x[k] <br/>
Augmented System <br/>
A<sub>aug</sub> = [A 0; CA I]  <br/>
B<sub>aug</sub> = [B;CB] <br/>
C<sub>aug</sub> = [0 .. 1] <br/>

## MPC002 is for system with input feedthrough matrix D!=0
System Model : <br/>
x[k+1] = A x[k] + B u[k] <br/>
y[k] = C x[k] + **D u[k]** <br/>
Augmented System <br/>
A<sub>aug</sub> = [A 0; CA I]  <br/>
B<sub>aug</sub> = [B;CB] <br/>
C<sub>aug</sub> = [0 .. 1] <br/>
D<sub>aug</sub> = [0;D]

**Np** is the prediction horizon <br/>
**Nc** is the control horizon <br/>
Np>=Nc<br/>
Minimum value of Np for example 2 in **MPC002** is 4 and minimum value of Nc is 2. Otherwise the system becomes unstable. <br/>
P.S. : Np = 5 and Nc = 2 is also unstable. (Reason unknown)

## QuadIneqConst is to solve the quadratic programming problem with inequality constraints
Hildert's Quadratic Programming provides a programmable solution to the Primal Dual method for solving inequality constraints. <br/>
It calculates a vector of the Lagranges multipliers (lambda) where, <br/>
S<sub>act</sub> is the set of indices of active constraints. <br/>
lambda<sub>i</sub> = 0 if i does not belong to S<sub>act</sub> <br/> 
                   
## MPC003 
Implementation of Receding Horizon control with **inequality constraints** on the rate of change of control input. <br/>
Hildert's Quadratic Programming approach is used to determine the active set of constraints. 
