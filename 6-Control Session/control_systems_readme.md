# control systems in robotics

- **Understanding the rule of control engineering**
- **feed back control & Open loop vs closed loop systems**
- **Control process flow**
- **mathematical modelling**
- **types of control theories**
- **Controllers types**
- **Simulation vs reality (What the real world doesn't tell you)**

---

## **Understanding the rule of control engineering**

**tends to**

- reduce the difference between the output of a system and some reference input
- increase the stability of the system
- decrease the sensitivity of the system to the external disturbances (not the error)
- increase the speed of response of the system

---

## components of control Systems

**system is a combination of components that act together and perform a certain objective these components like :**

### Plant

**any physical object to be controlled (such as a mechanicaldevice, a heating furnace, a chemical reactor, or a spacecraft) is called a plant**

![1754842984122](image/Kinematics/1754842984122.png)

### Process

**we shall call any operation to be controlled a process**

### Controlled Variable

**The controlled variable is the quantity or condition that is measured and controlled(like speed or temperature)**

### Error

**Reference input – actual output**

### Control Signal

**The control signal or manipulated variable is the quantity or condition that is varied by the controller so as to affect the value of the controlled variable**

![1754843106144](image/Kinematics/1754843106144.png)

### Disturbances

**A disturbance is a signal that tends to adversely affect the value of the output of a system (may be internal or external)**

**(described in block diagram as an input from no ware)**

![1754843190506](image/Kinematics/1754843190506.png)

**Disturbance doesn't have a constant form of wave that may be a constant shape or random**

![1754843168293](image/Kinematics/1754843168293.png)

---

## **feed back controlling & Open loop vs closed loop systems**

### open loop

**Those systems in which :**

**output has no effect on the control action are**

**output is neither measured nor fed back for comparison with the**

**each reference input there corresponds a fixed operating condition**

**In the presence of disturbances, an open-loop control system will not perform the desired task**

![1754843444663](image/Kinematics/1754843444663.png)

### closed loop (feedback control)

**A system that maintains a prescribed relationship between the output and the reference input enabling it to function properly in a changing environment**

**We use feedback control action in order to reduce system error**

![1754843460676](image/Kinematics/1754843460676.png)

---

## Control process flow

### understanding the system

**to understand how the system work**

**defining the inputs and outputs of the system**

### mathematical modelling

**converting the system to a number of equations that give a precise description of the system**

**modelling can be static , dynamic , or kinematic**

**wheeled , marine , aerial robots have kinematic models**

**electrical , mechanical , thermal , hydraulic , Pneumatic systems have dynamic models**

**most of systems have a mathematical model chain consisting of one or more type of systems**

**wheeled mobile robotics for example consisting both kinematic and dynamic (electrical , mechanical) models**

#### Statics vs Kinematics vs dynamics in mechanical systems

##### Statics

**the Study of forces in equilibrium without consideration of changes in velocity or acceleration over time at ( V = 0  ||  A = 0 )**

**Fundamental laws :**
- **ΣF = 0**
- **ΣM = 0**

##### Dynamics

**means a study of the rules governing the interactions of these particles( forces , torques ) which allow you to determine why the quantities have the values they do**

**Fundamental laws :**
- **ΣF = ma**
- **ΣM = Jα**

##### kinematics

**the study of properties of motion , position, velocity, acceleration, etc without any consideration of why those quantities have the values they do (ignoring forces and torques)**

**for example :**

**A small cart of mass 5 kg  is placed on a horizontal surface. The coefficient of static friction is μs=0.3  and the coefficient of kinetic friction is μk=0.2 . A horizontal force F is applied to the cart in the direction of motion. Assume g=9.81 m/s^2 .**

**statics : what is the minimum required force to start moving the cart**

**dynamics : what will be the acceleration if we applied a force / torque with a certain value on the cart**

**kinematics : what will be the velocity, position after n seconds**

---

### Kinematic modelling & motion planning process flow

#### Define the Robot Configuration

**choosing a reference point to control on which oftenis the body-centered**

**its better if the RP is co-centered with the center of the wheels**

#### Defining the robot's pose & orientation

**usualy represented as a matrix or array P = [ x , y , θ ]**

**x , y are the cordinates in the global frame , θ is the orientation (heading) of the robot**

#### building a kinematic model

**it may be a relative model like unicycle / bicycle / Omni directional model or a custom designed model when modelling a new machanism**

**forward and inverse kinematic formulas are deriven at this step**

**constrains are taken into considration like :**

- **Any component can't move in a direction orthogonal to its joint axis of rotation**
- **movementdirection is parallel to the forces & torques which causes the motion**

#### applying Forward Kinematics

**wheel velocities ( Vr , Vl ) → input**
**robot's linear and angular velocity V , ω → output**

#### applying Inverse Kinematics

**desired V , ω , x , y , θ → input**
**required wheel velocities Vr , Vl → output**

**notes :**

**each wheel linear velocity is coming from a wheel angular velocity ω emanates from the center of the wheel**

**inverse kinematics come before forward kinematics in motion planning**

#### Apply a Trajectory Controller

**These generate the desired velocities to follow a path**

![1754526226605](https://file+.vscode-resource.vscode-cdn.net/c%3A/Users/Osama%20Helal/Desktop/mms%20sessions/image/session/1754526226605.png)

#### Position Estimation

**calculate the position of the robot using its velocity and orientation**

![1754526871006](https://file+.vscode-resource.vscode-cdn.net/c%3A/Users/Osama%20Helal/Desktop/mms%20sessions/image/session/1754526871006.png)

---

### approches of kinematic modelling

**we can apply a reduction to the model because any wheels do the same job move with the same velocity but the total torque is devided between them but torques and forces is a dynamic not kinematic issue**

#### Point mass model

**The velocity is resolved into components along the x and y axes No angular rotation**

![1754528519525](https://file+.vscode-resource.vscode-cdn.net/c%3A/Users/Osama%20Helal/Desktop/mms%20sessions/image/session/1754528519525.png)

#### Rigid body model

**Some reference points is chosen to control on**

**The velocity of any point on the rigid body is a resultant velocity of the reference points velocities**

**there is total angular rotation around an axis (z-axis in 2d typically)**

![1754528694829](https://file+.vscode-resource.vscode-cdn.net/c%3A/Users/Osama%20Helal/Desktop/mms%20sessions/image/session/1754528694829.png)

#### Kinematic chains

**a number of indevidual links (== bodies) connected by joints**

**have a base link (the link which is connected to all the other links in the kinematic chain)**

**The velocity of each link is resolved into components along the x and y axes & the angular rotation of any link is tanken into considration, then the total linear and angular velocity of the body is calculated**

##### Open Kinematic Chain

**the chain does not form a loop**

**One end is typically fixed (the base link), and the other is free to move**

![1754529769070](https://file+.vscode-resource.vscode-cdn.net/c%3A/Users/Osama%20Helal/Desktop/mms%20sessions/image/session/1754529769070.png)

##### Closed Kinematic Chain

**a sequence of links and joints that form one or more closed loops**

**these links and joints can be imaginary, like : swarm robotics**

![1754530081108](image/session/1754530081108.png)

**both ends are constrained, often connected back to the base or another point in the system, like delta robots**

![1754530026720](image/session/1754530026720.png)

---

### Kinematics models of mobile robots

#### Wheeled mobile robots

**each pair of identical co-axial driving wheels can be represented by a single equivalent wheel**

##### Unicycle model (differential drive robots)

**it can contain a 4wd or 2wd (using gears for transmiting torque for all the wheels) or 2wd and a caster wheel**

![1754530309314](image/session/1754530309314.png) ![1754531393806](image/session/1754531393806.png)

![1754530320751](image/session/1754530320751.png)

**its reduction is a unicycle or 2wd model**

![1754530338850](image/session/1754530338850.png)      ![1754531373126](image/session/1754531373126.png)

##### Bicycle model (for car like robots)

**a steering control wheel and a lateral control wheel**

![1754531824272](image/session/1754531824272.png)

##### omni directional model

**reduction is hard because each wheel model has a uniqe axis of rotation**

![1754532295374](image/session/1754532295374.png)

![1754532325110](image/session/1754532325110.png)

![1754532346803](image/session/1754532346803.png)

![1754532442805](image/session/1754532442805.png)

#### marine robotics

**each thuruster axis of rotation is parallel to the linear velocity**

![1754532564894](image/session/1754532564894.png)

#### Aerial robotics

**wings positions are taken into consideration at determining the direction of movement**

**the direction of movement is paraller to the axis of rotation of the propeller ,too**

![1754532817635](image/session/1754532817635.png)

![1754532829138](image/session/1754532829138.png)

---

### Some fundamental laws of kinematics

#### converting angular vel to linear

**can be used to determine wheels angular vel from the linear vel**

**can be used to determine the required angular vel for the robot to turn around a certain corner**

![1754533067075](image/session/1754533067075.png)

![1754533079653](image/session/1754533079653.png)

#### general laws to determine displacement , current vel

**all the laws hold on both linear and angular velocities**

##### at constant acceleration

![1754533092098](image/session/1754533092098.png)

##### at variable acceleration

**at discrete time systems we use the euler numerical integration method at a specific time step to calculate the summition instead of the integration**

![1754533637496](image/session/1754533637496.png)

**Practically**

**displacement and velocity is calculated from the encoders (absolute /incremental) by summation**

**orientation is calculated using IMUs**

---

### Low-Level Wheel Controllers

**typically used for manual controlling of robotics**

#### Differential drive controllers

##### Forward Kinematics

**The golden fundamental law is**

**V = ω * r**

![1754534247991](image/session/1754534247991.png)

**applying this law at each wheel center and the reference point with a direction of rotation counter clockwise then, we deduce that**

**ω * ( R + l/2 ) = Vr        ω * ( R - l/2 ) = Vl**

**where l is the distance between the centers of the two wheels : the width of the robot from the center of one wheel to the other , Vr ; Vl are the right and left wheel velocities along the ground , and R is the signed distance from the ICC to the midpoint between thewheels**

**adding and subtracting these equations resulting in**

![1754534598937](image/session/1754534598937.png)

**There are three interesting cases with these kinds of drives**

**If Vl = Vr, then we have forward linear motion in a straight line. R becomes infinite, and there is effectively no rotation then ω is zero.**

**If Vl = - Vr, then R = 0, and we have rotation about the midpoint of the wheel axis - we rotate in place.**

**If Vl = 0, then we have rotation about the left wheel. In this case R= L Same is true if Vr = 0 but ω will be negative (clock wise)**

##### inverse kinematics

![1754537047514](image/session/1754537047514.png)

**to determine pose & orientation**

**don't use the integration if you want to determine the velocity**

**You can use any transformation between polar and Cartesian coordinates at determining any values**

![1754536645863](image/session/1754536645863.png)

#### Ackermann Steering Controller

**Car-like which have a rear-wheel drive, front-wheel steering)**

![1754537789944](image/session/1754537789944.png)

#### Omnidirectional / Mecanum Wheel Controller

**3 or 4 specially designed wheels allowing full 2D**

![1754537931534](image/session/1754537931534.png)

---

### Path Tracking / Trajectory Tracking Controllers

#### Pure pursuit controller

**aims to reduce cross track error**

![1754538022225](image/session/1754538022225.png)

#### Stanley controller

**aims to reduce heading error**

![1754538072175](image/session/1754538072175.png)

---

### dynamic modelling :

**Dynamic modeling uses mathematical equations to represent how systems change over time, such as electrical circuits with resistors and capacitors, or mechanical systems like a mass–spring–damper, by describing the relationships between inputs, outputs, and internal dynamics**

#### mechanical model

![1754844883136](image/Kinematics/1754844883136.png)

![1754844891231](image/Kinematics/1754844891231.png)

![1754844899639](image/Kinematics/1754844899639.png)

![1754844908159](image/Kinematics/1754844908159.png)

![1754844917935](image/Kinematics/1754844917935.png)

![1754844928559](image/Kinematics/1754844928559.png)

![1754844935883](image/Kinematics/1754844935883.png)

![1754844943263](image/Kinematics/1754844943263.png)

### types of control theories

#### Classic control :

**uses block diagrams, transfer functions , Laplace transform , and frequency domain to characterize the input-output relationships of components**

**for systems that can be described by linear , time-invariant, differential equations**

**preferred for SISO systems**

#### Modern control

**It works in the time domain and allows for more complex and flexible control strategies**

**more powerful for high-order or complex systems, especially when all internal states are not directly measurable**

**preferred for MIMO systems and uses state-space representation**

#### Robust control :

**focuses on system performance under uncertainty**

**ensures the system works well even when there are modeling errors or external disturbances**

#### Nonlinear control :

**used when the system's behavior does not follow linear equations**

**the system's response depends on more complex relationships – like powers, products of variables, trigonometric functions, etc**

#### Adaptive control :

**used when the system parameters change over time, or are initially unknown**

**the controller adjusts itself automatically to maintain performance**

---

### response analysis & System Stability

**before designing a controller we start analyzing the output of the system to gain informations about system stability**

**system stability is the system's ability to return to equilibrium after being disturbed**

![1754845021448](image/Kinematics/1754845021448.png)

#### systems classification based on stability

##### Stable

**for any bounded input, the output remains bounded and eventually settles to a steady value (doesn't oscillate around the desired value at ss)**

**may be under, critically, or over damped à critically damped is the best**

![1754845130177](image/Kinematics/1754845130177.png)

##### Critically stable

**it doesn't decay to zero error**

**The system keeps oscillating forever with constant amplitude around the desired at ss**

![1754845144083](image/Kinematics/1754845144083.png)

##### Unstable

**A system is unstable if small disturbances cause the output to grow without bound**

**the system may diverge due to its internal dynamics**

![1754845164536](image/Kinematics/1754845164536.png)

#### applying compensation for unstable systems

**some systems needs some compensations to make the system more stable**

**for example**

**a law pass filter is used to prevent / damp high frequency oscillations which make the system tend to over correcting errors which leads to instability**

---

### **Controllers types**

#### designing a controller

**after applying the compensation we apply a controller to enhance the output if needed controllers also need enhancements to be more powerful like PID controller enhancements**

#### Pid(classic)

##### Proportional Control Action

**a controller with proportional control action**

**the relationship between the output of the controller u(t) and the actuating error signal e(t) is**

**u(t) = Kp e(t)                                  (time domain)**

**U(s) = Kp E(s)                               (Laplace or frequency domain)**

**where Kp is termed the proportional gain**

##### Integral Control Action

**a controller with integral control action**

**the value of the controller output u(t) is changed proportional to the integration of the actuating error signal e(t)**

**u(t) = Ki ∫ e(t) dt**

**U(s) = (Ki/s) * E(s)**

**note : integration(summation) from 0 to t, where t is the instantaneous time at the cycle of the process**

**Its not a real integration but it's a summation process of the errors during a period of time to calculate the area under the curve**

**where Ki is termed the integral gain which increases the effect or power of the integral term**

##### Derivative Control Action :

**A controller with a derivative control action**

**the value of the controller output u(t) is changed proportional to the differentiation the actuating error signal e(t)**

**Kd =de(t)/dt**

**U(s) = Kd * s * E(s)**

**its not a real differentiation but it's a slope equation to calculate the rate of change with the time at a period of time(the period is equal to the cycle time)**

**Kd is termed the differentiation gain which increases the effect or power of the differential term)**

##### Proportional-Plus-Integral-Plus-Derivative Control Action

![1754848849173](image/Kinematics/1754848849173.png)

![1754848855480](image/Kinematics/1754848855480.png)

![1754848863880](image/Kinematics/1754848863880.png)

#### Contorollers modifications & enhancments

##### Derivative Noise Sensitivity

**The derivative term amplifies high-frequency noise from sensors, causing unstable or jittery control signals**

**square waves has a very high slope value**

![1754849185976](image/Kinematics/1754849185976.png)

**Apply a low-pass filter**

**apllied to the derivative term to reduce noise influence**

##### Integral Windup

**When the actuator is saturated or the error is large for a long time, the integral term accumulates excessively, causing overshoot and slow recovery**

**Use anti-windup techniques**

**To clamp Integral term (limit the integral term)**

![1754849402835](image/Kinematics/1754849402835.png)

##### Setpoint Overshoot

**A sudden change in setpoint produces a large error, causing the PID to overreact and overshoot**

**Setpoint ramp**

**Gradually change the setpoint instead of jumping instantly**

**Used when a sudden, large change in the reference input occurs, generating a significant error that the system cannot handle in a single step**

**for example : requiring a torque higher than the car engine can produce instantly. In such cases, the change is divided into smaller steps to avoid exceeding actuator limits or causing damage**

![1754849666708](image/Kinematics/1754849666708.png)

#### LQR (optimal but can be included in modern control)

**An optimal control method that minimizes a defined cost function to achieve the best trade-off between control effort and system performance; typically part of modern control theory.**

#### H∞ (robust)

**A robust control method designed to maintain system stability and performance under model uncertainties and external disturbances by minimizing the worst-case gain from disturbances to outputs.**

---

### **Simulation vs reality (What the real world doesn't tell you)**

#### Model Accuracy vs Real-World Complexity

**Simulated mathematical or physical models are often simplified (ideal conditions) Ignoring effects like irregular friction, wear, or unexpected noise**

**The real world is full of unpredictable factors.**

**Some phenomena are hard to model accurately (temperature variation, mechanical wear)**

**for example : A micromouse robot simulation may ignore wheel slippage on a wet surface.**

#### Perfect Measurements vs Sensor Limitations :

**sensors Provide perfectly accurate values with no measurement error or delay in simulations but actually Sensors have noise, bias, and measurement errors**

**There is latency in reading data, which can affect control performance**

**for example : gyroscopes give perfect angles in simulation ,but in reality they suffer from drift and noise**

**we apply corrections to the readings to remove the drifts and statistical analysis like taking the average to reduce the noise effect (can't be totally removed) )**

#### Instant Control vs Actuator Limits

**We should consider the limitations of the actuators and sensors used like the maximum deflection of the steering system or maximum torque of the lateral system to avoid over loading on the components**

#### Life span and unpredicted failure of components

**components parameters affects the system output and stability**

**Some designing  approaches tend to reduce the effect of the transfer function parameters like using high gain**

**diagnostics auto checking may be a powerful way to avoid disasters**