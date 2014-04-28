hexapod-engine
==============

python hexapedal IK and gait engine

Current status:
As before, IK and FK are doing well.
I almost had a gait sequencer working the way I wanted it to, but then I decided to refactor everything and change the model/pose system. So now I don't have any kind of gait sequencer, but the underlying code is probably a little bit nicer and more extensible.

I want to implement a gait sequencer that actually plans out dynamically where and when to place each foot instead of merely following a sequence and changing the vectors involved(Though that works very well apparently). The main problem I face when I think about how it should work is whether to plan ahead or not. The optimal foot position for a body that will continue moving forward is entirely different from one which will turn or reverse motion after the next step. As such, if I want to calculate 'optimal' positions I need to lock in the motion for the next few steps or so, which can potentially be bad news for someone trying to control the robot in real time or if the robot itself is planning motion based on changing information.

So I'll have to figure that out at some point, but for now the working code is in the 'staging' directory, and it uses some old stuff from other top level directories as well as refactored stuff within the staging directory, so its pretty big mess that I should try to sort out soon.


Plan
====

There will be 4 top level parts:

1. Navigation
2. Gait
3. Inverse Kinematics
4. Servo

The nav engine takes input from sensors and/or controllers and outputs velocity(speed and direction relative to current orientation of chassis). It also outputs angular velocity(yaw).

The gait engine takes the output from the nav engine and converts it into the required foot placement positions over time.

The IK engine takes the foot placements and turns them into servo angles.

The servo engine takes angles and converts them into actual motion.


The majority of input happens at the very top of the stack, from a controller. There is the possibility for input to the gait engine in the form of the required gait, but this could also be programmatically set (ie. tripod for high speed, ripple for low speed). The gait engine also needs to majority of the robot-specific parameters, such as limits on leg translation, and physical constraints of the platform. The IK engine could make use of this information, but because of the specific leg design in mind there will only be two solutions possible for a given input, only one of which will be realizable.

CURRENT STATUS:
FK and IK are entirely working.
Gaits are partially working... still ironing out details on timing, and interpolating, and foot placement selection is something that will require continual improvement
Everything currently is in the kinematics directory. I'll actually organize the modules and stuff when they are functioning a little bit better.
The control system doesn't exist yet in any fashion. It will probably be a separate python thread that will be queried by the main loop, but I'll figure it out later.
The servo/hardware interfacing doesn't really exist either, but it probably won't take much time since Pololu(the makers of the servo controller I plan on using) have some great serial control libraries.
