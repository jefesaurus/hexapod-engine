hexapod-engine
==============

python hexapedal IK and gait engine

HIGH LEVEL PLAN:
There are 4 modules in this system, 2-3 of which will actually be implemented here.

1. Navigation Engine
2. Gait Engine
3. Inverse Kinematics Engine
4. Servo Engine

The nav engine takes input from sensors and/or controllers and outputs velocity(speed and direction relative to current orientation of chassis). It also outputs angular velocity(yaw).

The gait engine takes the output from the nav engine and converts it into the required foot placement positions over time.

The IK engine takes the foot placements and turns them into servo angles.

The servo engine takes angles and converts them into actual motion.


The majority of input happens at the very top of the stack, from a controller. There is the possibility for input to the gait engine in the form of the required gait, but this could also be programmatically set (ie. tripod for high speed, ripple for low speed). The gait engine also needs to majority of the robot-specific parameters, such as limits on leg translation, and physical constraints of the platform. The IK engine could make use of this information, but because of the specific leg design in mind there will only be two solutions possible for a given input, only one of which will be realizable.
