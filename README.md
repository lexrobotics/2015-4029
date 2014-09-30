2015-4029
=========

FTC 4029 repository for the 2014-2015 season. 

Do NOT place test files in here. Keep them in the non-version-controlled Documents/Programming/test folder.

Design Philosophy
================
This year, we are adopting a software design philosophy of using smaller, independent modules for each aspect of robot control. Each module will interface with a central struct that contains a variety of information about the robot. The basic structure of a module looks as follows:
```C
struct ModuleName {
  int last_update; // time of lastUpdate
  //other private data for module
};

void modulename_init() {
  //initializes module
}

RobotState modulename_update(RobotState prevState) {
  int dT = time1[T1] - ModuleName.lastUpdate; 
  RobotState newState; 
  // update state
  ModuleName.lastUpdate = time1[T1]; 
  return newState;
}

void modulename_otherPrivateFunction() {
}

// if more than one module is conflicting with the same data, you may write a module with an update function that takes multiple robot-states as an input and returns the filtered robotstate as an output

RobotState modulename_update(RobotState gyroState, RobotState encoderState) {
  RobotState newState
  return newState;
}
```

Using a container struct for data and prefixing functions with the module name will prevent pollution of the global namespace. Although verbose, this will allow modules to be more independent. 

Style
=====
####Variables:
Different words in variables are separated by underscores. e.g. `int current_speed`. 

####Functions:
Function names are written with camelCase. e.g. `moveForward()`. 

####Modules:
For both TeleOp and Autonomous

