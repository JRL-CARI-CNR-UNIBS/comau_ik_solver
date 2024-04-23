# Comau IK Solver

The package implements a plugin of type `ik_solver` whose documentation is [here](https://github.com/JRL-CARI-CNR-UNIBS/ik_solver/blob/parallel-ik/README.md)

## The package functionalities

The package deals with the parallelogram of the COMAU robots. 

The package assumption is that the PARALLELOGRAM is a RECTANGLE.

Two angles are therefore defined: 

* *gamma*: this is the parallelogram Angle in proximity to the robot's shoulder

* *epsilon*: this is the parallelogram Angle in proximity to the robot elbow

To avoid any collision, we have two limits:

* *gamma_min*

* *epsilon_min*

Then, we define two angles: 

* *alpha* = angle between the active crank and the horizontal axis

* *beta*  = angle between link3 and the horizontal axis

Therefore, 

    gamma = pi - (alpha + beta) -> the angle of the parallelogram at the corner the bottom is the complement to PI of alpha
    and beta epsilon = (alpha+beta) -> the angle of the parallelogram at the top corner  is the complement to PI of gamma

    alpha = -q3 -pi/2
    beta = -q2 + pi/2

    gamma > gamma_min     ==>  pi - (alpha + beta) > gamma_min  ==>   pi - (-q2-q3) > gamma_min ==> q2+q3 > gamma_min - pi
    epsilon > epsilon_min ==> (alpha + beta) > epsilon_min      ==> (-q2-q3) > epsilon_min      ==> q2+q3 < -epsilon_min

The parameters *gamma_min* and *epsilon_min* are mandatory parameters to be inserted in the param configuration file.

## The plugin

The package builds three different plugins:
The geometric dimensions of **Comau_NJ_370_27_IkSolver**** and **Comau_NJ_220_27_IkSolver** are embedded inside the cpp code. The **Comau_NJ_Generic_IkSolver** reads the dimensions from the URDF at the configuration stage.

## Config Parameters

```yaml
###################################################
#
# The param needed by the every plugin that inherits 
# from ik_solver base class
#
###################################################

# This param is inherited from the ik_solver base class.
# This param tells the plugin loader to load the 
# SPECIFIC_PLUGIN

type: ik_solver/Comau_NJ_Generic_IkSolver # The package exposes three plugins: Comau_NJ_Generic_IkSolver, Comau_NJ_220_27_IkSolver, Comau_NJ_370_27_IkSolver

# Parameters inherited from the ik_solver base class 
base_frame: base_link #world # base frame of the chain
flange_frame: flange # end frame of the chain

tool_frame: flange # destination frame of the IK (it should be rigid attached to flange_frame)
desired_solutions: 8 # number of desired solution
joint_names:
- joint_1
- joint_2
- joint_3
- joint_4
- joint_5
- joint_6

#Parameters specific for COMAU implementation
gamma_min_deg: 40     # This parameter is used from the Comau_NJ_Generic_IkSolver. It used to avoid collision of the parallelogram.
epsilon_min_deg: 40   # This parameter is used from the Comau_NJ_Generic_IkSolver. It used to avoid collision of the parallelogram.
```
