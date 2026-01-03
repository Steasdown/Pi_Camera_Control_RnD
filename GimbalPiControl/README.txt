GimbalJSControl simple refactor (BigMover-style)

Goal
- Keep behaviour/output IDENTICAL to your current GimbalJSControl.ino
- Split into: main .ino + Global header + MotorControl (cpp/h) + PiCommunication (cpp/h)

How to use
1) Create a new Arduino sketch folder named: GimbalJSControl
2) Copy all files from this package into that folder
3) Open GimbalJSControl.ino and upload as usual

Notes
- PiCommunication is included but NOT used yet (no Serial reads are performed unless you call piComm.update()).
- MotorControl encapsulates enable pin + AccelStepper + speed ramp state.
