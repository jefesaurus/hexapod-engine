hexapod-engine
==============
python/C hexapedal IK and gait engine

Current status:

- Generic FK math works well and can output ready-to-compile C to handle transformations quickly
- IK only exists for one very specific arrangement of revolute joints, but you can drop in a new one quite easily
- Step controller is in progress. Does high-level bezier or linear interpolation between points over time, and low-level speed variation to synchronize starts and stops
- Gait controller doesn't exist yet

To run:

Most objects, such as the Kinematic Chain and leg modles have some demo/test methods in them

To start the interactive system use:

  python test_controller.py

This starts forks two processes and a thread

- The new thread takes input as keypresses on the command line
- The original thread turns the graphical output
- One of the new processes runs the update/input processing loop for the main model controller
- The other process runs the update/input loop for the simulated model

So there are 4 main blocks in the system:

Input Parser->Leg Controller->Simulated Model->Graphical Representation


C++ Version:

I stalled out in the python version, and wanted something that would have the possibility of running on embedded hardware so I started working on a C++ version which can be found in

/hexapod-engine/src/c_version/simulator/

There is a GLUT viewer that can show animated legs and chassis. Aside from GLUT the only other dependency is Eigen, for linear algebra.
On ubuntu just download the eigen folder and drop it into the "simulator" folder(or edit the makefile). For GLUT, I think just do the package manager installation and that should be fine.
Check the main file for some examples on what it does currently. Mostly Leg stuff for now, but I'll add the chassis stuff in the next month or so and then eventually actually test it.
