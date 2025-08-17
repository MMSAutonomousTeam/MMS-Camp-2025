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

<img width="405" height="326" alt="1754842984122" src="https://github.com/user-attachments/assets/2d480f9c-947b-45a7-9880-6bf440ffc58a" />


### Process

**we shall call any operation to be controlled a process**

### Controlled Variable

**The controlled variable is the quantity or condition that is measured and controlled(like speed or temperature)**

### Error

**Reference input – actual output**

### Control Signal

**The control signal or manipulated variable is the quantity or condition that is varied by the controller so as to affect the value of the controlled variable**

<img width="550" height="166" alt="1754843106144" src="https://github.com/user-attachments/assets/be7838f2-2ba1-43eb-9c13-a17fabfe7e0f" />


### Disturbances

**A disturbance is a signal that tends to adversely affect the value of the output of a system (may be internal or external)**

**(described in block diagram as an input from no ware)**

<img width="624" height="303" alt="1754843190506" src="https://github.com/user-attachments/assets/7ef91b20-8fe6-4469-b0ee-c9ab4a58a00d" />


**Disturbance doesn't have a constant form of wave that may be a constant shape or random**

<img width="455" height="211" alt="1754843168293" src="https://github.com/user-attachments/assets/64528e8c-7bfd-4170-8574-7f9fdd2dbe2e" />


---

## **feed back controlling & Open loop vs closed loop systems**

### open loop

**Those systems in which :**

**output has no effect on the control action are**

**output is neither measured nor fed back for comparison with the**

**each reference input there corresponds a fixed operating condition**

**In the presence of disturbances, an open-loop control system will not perform the desired task**

<img width="554" height="142" alt="1754843444663" src="https://github.com/user-attachments/assets/f40f919d-5ae6-4e84-8db8-54830b3f7e11" />


### closed loop (feedback control)

**A system that maintains a prescribed relationship between the output and the reference input enabling it to function properly in a changing environment**

**We use feedback control action in order to reduce system error**

<img width="434" height="280" alt="1754843460676" src="https://github.com/user-attachments/assets/991e54b0-1509-4c18-b0ff-9acb5be3c1cb" />


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

**A small cart of mass 5 kg  is placed on a horizontal surface. The coefficient of static friction is μs=0.3  and the coefficient of kinetic friction is μk=0.2 . A horizontal force F is applied to the cart in the direction of motion. Assume g=9.81 m/s^2**

**statics : what is the minimum required force to start moving the cart**

**dynamics : what will be the acceleration if we applied a force / torque with a certain value on the cart**

**kinematics : what will be the velocity, position after n seconds**

---

### Kinematic modelling & motion planning process flow

  - **Defining the Robot Configuration**
  - **building a kinematic model**
  - **applying Forward Kinematics**
  - **applying Inverse Kinematics**
  - **Apply a Low Level Controller**
  - **Apply a Trajectory tracking Controller**
  - **Position Estimating**
---
### Defining the Robot Configuration

#### choosing a reference point to control on 
**often is the body-centered**
**its better if the RP is co-centered with the center of the wheels**

#### Defining the robot's pose & orientation
**usualy represented as a matrix or array P = [ x , y , θ ]**
**x , y are the cordinates in the global frame , θ is the orientation (heading) of the robot**

---
### building a kinematic model

**it may be a relative model or a custom designed model when modelling a new machanism**

**holonomic and nonholonomic constrains are taken into considration**

**forward and inverse kinematic formulas are deriven depending on this step**

**movement direction is parallel to the forces & torques which causes the motion**

**a reduction is applied to the model if possible**

#### approches of kinematic modelling
##### Point mass model

**The movement is resolved into x,y components but No angular rotation**

<img width="530" height="392" alt="1754528519525" src="https://github.com/user-attachments/assets/fd8c60cb-6e18-4145-a09a-851538aed368" />


##### Rigid body model

**Some reference points is chosen to control on**

**The velocity of any point on the rigid body is a resultant velocity of the reference points velocities**

**there is total angular rotation around an axis (z-axis in 2d typically)**

<img width="320" height="205" alt="1754528694829" src="https://github.com/user-attachments/assets/eeba7de6-c232-4574-816b-d269a20305aa" />


##### Kinematic chains

**a number of indevidual links (== bodies) connected by joints**

**have a base link (the link which is connected to all the other links in the kinematic chain)**

**The velocity of each link is resolved into components along the x and y axes & the angular rotation of any link is tanken into considration, then the total linear and angular velocity of the body is calculated**

**these links and joints can be imaginary, like : swarm robotics**

<img width="493" height="390" alt="1754530081108" src="https://github.com/user-attachments/assets/97924f7c-8554-4fe8-9a25-8a7f0b923026" />

###### Open Kinematic Chain

**the chain does not form a loop**

**One end is typically fixed (the base link), and the other is free to move**

<img width="473" height="256" alt="1754529769070" src="https://github.com/user-attachments/assets/fd132512-c719-4324-b594-5a496c358b20" />


###### Closed Kinematic Chain

**a sequence of links and joints that form one or more closed loops**

**both ends are constrained, often connected back to the base or another point in the system, like delta robots**

<img width="513" height="288" alt="1754530026720" src="https://github.com/user-attachments/assets/3d267987-6c1e-4272-b8ec-6a37554fbf52" />

---

### Kinematics models of mobile robots

#### Wheeled mobile robots

**each pair of identical co-axial driving wheels can be represented by a single equivalent wheel**

##### Unicycle model (differential drive robots)

**it can contain a 4wd or 2wd (using gears for transmiting torque for all the wheels) or 2wd and a caster wheel**

<img width="299" height="291" alt="1754530309314" src="https://github.com/user-attachments/assets/98e9af0e-8c44-4261-84ba-308aede51cbd" />
<img width="486" height="324" alt="1754531393806" src="https://github.com/user-attachments/assets/c0c57d0f-802a-4f74-876a-475ef1f47c1e" />

![light-following-robot-arduino-uno](https://github.com/user-attachments/assets/38049d4c-27ba-4901-8cb4-d97861aa8098)



**its reduction is a unicycle or 2wd model**

<img width="445" height="253" alt="1754531373126" src="https://github.com/user-attachments/assets/bb1af3fc-6357-4635-862e-c1355a1baf7b" />
<img width="335" height="283" alt="1754530338850" src="https://github.com/user-attachments/assets/f8c4e3af-020a-43ac-9f90-75c4b01db749" />

##### Bicycle model (for car like robots)

**a steering front wheel and a lateral rear wheel**

<img width="627" height="510" alt="1754531824272" src="https://github.com/user-attachments/assets/965759c0-de35-4816-a07e-f4bd194aa939" />


##### omni directional model

**reduction is hard because each wheel model has a uniqe axis of rotation**

![Nexus-Omni-Wheel-Mobile-Robot (1)](https://github.com/user-attachments/assets/c5920271-61b7-4d7f-88df-dc436d9999a3)

<img width="463" height="450" alt="1754532325110" src="https://github.com/user-attachments/assets/bc37b2c1-a651-4b1e-a81c-9541ba1538be" />

![Different_Motion_Square_Normal_Mecanum](https://github.com/user-attachments/assets/96f5d515-fcd2-4883-bb99-d49c17377153)

<img width="734" height="811" alt="1754532442805" src="https://github.com/user-attachments/assets/a9d7ebb1-0168-46a0-9a47-98221d2f62cb" />




#### marine robotics

**each thuruster axis of rotation is parallel to the movement direction**

<img width="614" height="409" alt="1754532564894" src="https://github.com/user-attachments/assets/7bdb15e9-7a86-41d5-9027-bfca58a9fa87" />


#### Aerial robotics

**wings positions are taken into consideration at determining the direction of movement**

**the direction of movement is paraller to the axis of rotation of the propeller ,too**

<img width="624" height="389" alt="1754532817635" src="https://github.com/user-attachments/assets/37abfea6-baa5-48cd-8654-7faf2eb0d593" />
<img width="297" height="169" alt="1754532829138" src="https://github.com/user-attachments/assets/ab0b2353-68e8-4cac-ac81-907930c50a82" />


---

### Some fundamental laws of kinematics

V, ω, and R 

![Uniform-cirular-translation](https://github.com/user-attachments/assets/ef931f13-0d79-42d7-8333-474069662710)


#### converting angular vel to linear

**can be used to determine wheels angular vel from the linear vel**

**can be used to determine the required angular vel for the robot to turn around a certain corner**

<img width="350" height="224" alt="1754533067075" src="https://github.com/user-attachments/assets/0d96f1c5-e1bd-45c8-9172-1891dd21086f" />
<img width="624" height="123" alt="1754533079653" src="https://github.com/user-attachments/assets/e72e335f-cf03-4853-8a72-a775c4934ef0" />


#### general laws to determine displacement , current vel

**all the laws hold on both linear and angular velocities**

##### at constant acceleration

<img width="500" height="229" alt="1754533092098" src="https://github.com/user-attachments/assets/c39c8f94-b330-43ea-9ded-fbc7e7e8ccfe" />


##### at variable acceleration

**at discrete time systems we use the euler numerical integration method at a specific time step to calculate the summition instead of the integration**

<img width="531" height="232" alt="1754533637496" src="https://github.com/user-attachments/assets/93b628eb-0e9e-4c85-a551-8eb76461b67f" />


**Practically**

**displacement and velocity is calculated from the encoders (absolute /incremental) by summation**

**orientation is calculated using IMUs**

---

### deriving forward & inverse kinematics laws 

**typically used for manual controlling of robotics**

**for example**

#### Differential drive model

##### Forward Kinematics

**wheel velocities ( Vr , Vl ) → input**

**robot's linear and angular velocity V , ω → output**

**The golden fundamental law is**

**V = ω * r**

<img width="525" height="428" alt="1754534247991" src="https://github.com/user-attachments/assets/579e0ee3-a136-4be9-aab5-7bfbe0591acb" />


**applying this law at each wheel center and the reference point with a direction of rotation counter clockwise then, we deduce that**

**ω * ( R + l/2 ) = Vr**        
**ω * ( R - l/2 ) = Vl**

**where l is the distance between the centers of the two wheels : the width of the robot from the center of one wheel to the other , Vr ; Vl are the right and left wheel velocities along the ground , and R is the signed distance from the ICC to the midpoint between thewheels**

**adding and subtracting these equations resulting in**

<img width="307" height="71" alt="1754534598937" src="https://github.com/user-attachments/assets/9562935f-fe1b-4319-8e1a-e9741d9e37f4" />


**There are three interesting cases with these kinds of drives**

**If Vl = Vr, then we have forward linear motion in a straight line. R becomes infinite, and there is effectively no rotation then ω is zero.**

**If Vl = - Vr, then R = 0, and we have rotation about the midpoint of the wheel axis - we rotate in place.**

**If Vl = 0, then we have rotation about the left wheel. In this case R= L Same is true if Vr = 0 but ω will be negative (clock wise)**

##### inverse kinematics

**desired V , ω → input**

**required wheel velocities Vr , Vl → output**

**each wheel linear velocity is coming from a wheel angular velocity ω emanates from the center of the wheel**

**applying (not deriving) inverse kinematics comes before forward kinematics in motion planning**
**forward kinematics is used only for deducing inverse kinematics**

---
### Low-Level Wheel Controllers
#### defferintial drive controller

**omega and V_base are given by sliders and joysticks positions respetively**
**omega and v are insersted in the inverse kinematic laws to calculate vl & vr**
**v and omega are your choise in manual controlling but are dedicated by the path in autononmous controlling**

<img width="2008" height="1636" alt="41598_2024_75500_Fig1_HTML" src="https://github.com/user-attachments/assets/ab09a023-f898-4c4f-b28c-e480dc25a461" />


#### Ackermann Steering Controller

**Car-like which have a rear-wheel drive, front-wheel steering)**

<img width="532" height="468" alt="1754537789944" src="https://github.com/user-attachments/assets/217a861a-416d-450d-b156-2737833077e0" />

#### Omnidirectional / Mecanum Wheel Controller

**3 or 4 specially designed wheels allowing full 2D**

<img width="495" height="258" alt="1754537931534" src="https://github.com/user-attachments/assets/633c8984-1db7-4c46-a3d4-da256c6cebcd" />

---

### Path Tracking / Trajectory Tracking Controllers

**These controllers generate the desired velocities to follow a path**

<img width="1299" height="781" alt="1754526226605" src="https://github.com/user-attachments/assets/1c891622-b686-4e44-a4c6-ffb8bfb7b3cd" />

#### Pure pursuit controller

**aims to reduce cross track error**

<img width="539" height="339" alt="1754538022225" src="https://github.com/user-attachments/assets/8d3bc7af-48c3-4320-abc7-b6f72a81082b" />

#### Stanley controller

**aims to reduce heading error**

<img width="434" height="297" alt="1754538072175" src="https://github.com/user-attachments/assets/0eee7863-bf55-40e3-afac-d9f839d1de82" />

---
### pose & orientation estimating

**calculating the position of the robot using its velocity and orientation**

**You can use any transformation between polar and Cartesian coordinates at determining any values**

<img width="256" height="143" alt="1754536645863" src="https://github.com/user-attachments/assets/6d2579a0-99e3-498e-8696-c83287e73b07" />

<img width="879" height="687" alt="1754526871006" src="https://github.com/user-attachments/assets/6f863e20-df56-430a-bcc1-16550c1d7bfd" />

---

### dynamic modelling :

**Dynamic modeling uses mathematical equations to represent how systems change over time, such as electrical circuits with resistors and capacitors, or mechanical systems like a mass–spring–damper, by describing the relationships between inputs, outputs, and internal dynamics**

#### mechanical model
<img width="602" height="266" alt="1754844883136" src="https://github.com/user-attachments/assets/a6291aa7-c893-44ce-ad8e-33a8b3e27d18" />

#### electrical model
<img width="624" height="331" alt="1754844891231" src="https://github.com/user-attachments/assets/05a42bd1-04fd-4a47-afbd-da1e0e51e918" />

#### fluid system
<img width="624" height="257" alt="1754844899639" src="https://github.com/user-attachments/assets/bff23a8a-8131-4e64-8a4f-cd6b8fd0c60b" />

#### pneumatic system
<img width="624" height="281" alt="1754844908159" src="https://github.com/user-attachments/assets/24a47bae-99e5-4fc2-aa71-5b4603ac188b" />

#### hydraulic system
<img width="579" height="432" alt="1754844917935" src="https://github.com/user-attachments/assets/003bdcef-b356-40e0-bb86-5b03ab4bbbf7" />

#### thermal system
<img width="434" height="232" alt="1754844928559" src="https://github.com/user-attachments/assets/75f28b46-6e15-49cf-b988-22500a33c93f" />

### chain dynamic models :
#### elcetro mechanical system 
<img width="624" height="237" alt="1754844935883" src="https://github.com/user-attachments/assets/d56b558c-b76b-4c87-9676-5e64a56f8f57" />

#### hydraulic & pneumatic system
<img width="624" height="421" alt="1754844943263" src="https://github.com/user-attachments/assets/f5ecd7a6-b43a-43cf-b9e1-608f94f8eb37" />







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

<img width="580" height="460" alt="1754845021448" src="https://github.com/user-attachments/assets/d193ce22-1d77-4fb4-b41a-33616d896e04" />

#### systems classification based on stability

##### Stable

**for any bounded input, the output remains bounded and eventually settles to a steady value (doesn't oscillate around the desired value at ss)**

**may be under, critically, or over damped à critically damped is the best**

<img width="616" height="360" alt="1754845130177" src="https://github.com/user-attachments/assets/03d7e2db-41bd-41c0-8b6c-842d167eee23" />

##### Critically stable

**it doesn't decay to zero error**

**The system keeps oscillating forever with constant amplitude around the desired at ss**

<img width="400" height="284" alt="1754845144083" src="https://github.com/user-attachments/assets/9da222e5-ba94-46ae-8a0f-db2550691cdc" />

##### Unstable

**A system is unstable if small disturbances cause the output to grow without bound**

**the system may diverge due to its internal dynamics**

<img width="400" height="269" alt="1754845164536" src="https://github.com/user-attachments/assets/9bf6185d-b773-4e0c-bcb8-189bfc66adbd" />

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

<img width="624" height="288" alt="1754848863880" src="https://github.com/user-attachments/assets/cc604d8e-b80f-4be0-9cde-7bc4930f8605" />
<img width="316" height="103" alt="1754848855480" src="https://github.com/user-attachments/assets/11e7a8f2-46c4-4156-85bd-f9fed0e4bdbf" />
<img width="498" height="103" alt="1754848849173" src="https://github.com/user-attachments/assets/a6a2070c-13b2-47c3-8558-c814b2f1be54" />

#### Contorollers modifications & enhancments

##### Derivative Noise Sensitivity

**The derivative term amplifies high-frequency noise from sensors, causing unstable or jittery control signals**

**square waves has a very high slope value**

<img width="561" height="212" alt="1754849185976" src="https://github.com/user-attachments/assets/052fa0c9-3030-46ae-85cd-14a96ce8c958" />

**Apply a low-pass filter**

**apllied to the derivative term to reduce noise influence**

##### Integral Windup

**When the actuator is saturated or the error is large for a long time, the integral term accumulates excessively, causing overshoot and slow recovery**

**Use anti-windup techniques**

**To clamp Integral term (limit the integral term)**

<img width="624" height="406" alt="1754849402835" src="https://github.com/user-attachments/assets/e12b78f5-801c-463b-aa56-b9219feafd79" />

##### Setpoint Overshoot

**A sudden change in setpoint produces a large error, causing the PID to overreact and overshoot**

**Setpoint ramp**

**Gradually change the setpoint instead of jumping instantly**

**Used when a sudden, large change in the reference input occurs, generating a significant error that the system cannot handle in a single step**

**for example : requiring a torque higher than the car engine can produce instantly. In such cases, the change is divided into smaller steps to avoid exceeding actuator limits or causing damage**

<img width="400" height="327" alt="1754849666708" src="https://github.com/user-attachments/assets/d52eef85-2adc-4b98-9dba-b1d0d00601f3" />

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
