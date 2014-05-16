hexapod-engine
==============
python/C hexapedal IK and gait engine

Current status:
-Generic FK math works well and can output ready-to-compile C to handle transformations quickly
-IK only exists for one very specific arrangement of revolute joints, but you can drop in a new one quite easily
-Step controller is in progress. Does high-level bezier or linear interpolation between points over time, and low-level speed variation to synchronize starts and stops
-Gait controller doesn't exist yet

To run:
Most objects, such as the Kinematic Chain and leg modles have some demo/test methods in them

To start the interactive system use:
  python test_controller.py

This starts forks two processes and a thread
  -The new thread takes input as keypresses on the command line
  -The original thread turns the graphical output
  -One of the new processes runs the update/input processing loop for the main model controller
  -The other process runs the update/input loop for the simulated model

So there are 4 main blocks in the system:
User->Input Parser->Leg Controller->Simulated Model->Graphical Representation
