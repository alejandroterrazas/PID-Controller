###
# CarND-Proportional-Integral-Derivative Encoder
Self-Driving Car Engineer Nanodegree Program

## Purpose

The purpose of this project is to understand the function of a proportional integral derviative **(PID)** encoder.  The code is written in C++ (see below for dependencies).  Also, there is code for optimization of the hyperparameters (coeeficients for each of the PID componennts).  The optimization code is called Twiddle, as it is known colloquially.  To run the PID code, type ./run_PID.sh.  Then start the CarND Simulator and select the PID configuration.  To run the Twiddle code, type ./run_Twiddle.sh and then start the CarND Simulator and select the PID module.

A **PID** provides the autonomous vehicle with corrective action in the form of an error that is fed into the steering (and accleration) of the car.  In addition the steering PID, this project includes a separate PID (throttle_pid) to control the throttle of the simulataed car.  
### The code

Much of the logic and code for the PID controller are found in `src/main.cpp`.  Additional functions are found in PID.cpp (the PID class).  Global variables are found in helpers.h.  For the Twiddle optimizer look in `src/twiddle.cpp`.

### Simulator Output Measurments

The simulator provides a cross track error (CTE) from the center of the road.  The PID controler responds to this error by incrementally adjusting the steering to reduce the CTE.  The adjustments are made in three ways described immediately below under _PID Components_.

## PID Components
Each of the PID components affects the steering in a different way and each has a tunable coefficient (weight):  a) The proportional component is the error of the car from the set point (which, in this case 0 for the center of the lane).  The proportional component will overshoot the target periodically because the error becomes 0 and eliminates control near that point.  The differential component partially addresses this issue.  b) The integral coponent removes bias by averaging the CTE over observations; thus, if steering exhibits an overall bias toward the right or left, the integral component will largely offset this bias (as long as it remains in a steady state). c) The derivative component represents the rate of change of the error and thus provides an estimate of the error at the next time point, thereby dampening the 'ringing' effect and improving the overall performance.

## The PID Class

The PID class is simple and contains three primary methods: a) Init, b) UpdateError and c) TotalError.  Two PID objects are used: 1) steering_PID and 2) throttle_pid for steering and throttle control respectively.  Init initializes the three PID coefficients (Kp, Ki, Kd) and sets the three error terms (p_error, i_error, d_error) to zero.  UpdateError computes the error terms for each coefficint.  The d_error must be computed before assigning the CTE (cross track error) to p_error.  

`void PID::UpdateError(double cte) { 
    d_error = cte - p_error;
   // std::cout << "d_error: " << d_error << std::endl;
    p_error = cte;
    i_error += cte;
}
`
Finally TotalError is used to adjust the steering.  It is a linear combination of the three error terms multiplied by the coefficients.

`
double PID::TotalError() {
  double total_error = - Kp * p_error - Ki * i_error - Kd * d_error;
  return total_error;
}

`

The UpdateError and TotalError methods are called for every sensor update from the simulator, as shown in the following code snippet.

` steering_pid.UpdateError(cte);
          double steer_value = steering_pid.TotalError();
          
          steer_value = (steer_value > 1) ? 1 : steer_value;
          steer_value  = (steer_value < -1) ? -1 : steer_value;
         // std::cout << "steer value: " << steer_value << std::endl;
          
          throttle_pid.UpdateError(cte);  //n.b. use abs with throttle
          double throttle_value = .4 + throttle_pid.TotalError();
          
          throttle_value = (throttle_value < .1) ? .1 : throttle_value;  //throttle min of .1
         // throttle_value = .3;  //for fixed velocity
         ...
 `

## Determining PID Coefficients with Twiddle

As noted, each PID component has a weight that is multiplied withe component. These weights are denoted Kp, Ki, Kd for proportion, integral, and differential, respectively. Finding the correct weight for each of these components can be a difficult task.  One approach is to experiment interatively; however, this approach can be unsatisfactory because with 3-6 parameters to consider at once, one can spend far to much time optimizing.  Moreover, when a new road is encountered, different coefficients might be necessary.  Twiddle, the collquial name for form of hill climbing optimization, was used to adjust the steering parameters.  Although, the throttle PID is included in the pid executable, it was not included in the twiddle executable.  

**_The final hyperparameters were chosen by a combination of experimentation-manual tuning and Twiddle.  It is nice to have a general idea of the parameters prior to running the optimization._** 

Overall, twiddle works to minimize the CTE by adjusting each of the component weights (Kp, Ki, Kd) incrementally over a large number of runs.  Twiddle can be sensitive to the starting conditions and it is recommended that the user begin with a good best guess.  Each of the weights is stored in a vector of double **p**.  In addition, there is a vector of the same dimension called **delta_p**, which can be thought of as proposed changes to the component weights (Kp, Ki, and Kd)  On each consecutive run, the average error is evaluated relative to the previous best error.  If the error becomes smaller on a subsequent run, it becomes the previous best error and the weights of the coefficientof error terms are updated the new values.  The delta_p vectors is also incrementally moved upward or downward depending on the results of the error comparisons.  The sum of the proposed changes in the vector delta_p is reduced over increments as the error is reduced.

[https://www.youtube.com/watch?v=2uQ2BSzDvXs] is Sebastien Thrun's tutorial on Twiddle.  

There are two stages to the Twiddle algorithm.  In the code, these are contained in two functions.  The reason for this is that the simulator is difficult to control from the program.  The first part is in a function called **first 

`
bool first_part(int index, double error) {

  bool flag;
  if (error < best_error) {  
    best_error = error;
    dp[index] *= up_multiplier;
    print_vals("first part: error is better");
    flag = true;
  } else { //!error is worse       
    p[index] -= 2*dp[index];
    dp[index] *= down_multiplier;
    print_vals("first part error is worse");
    flag = false;
  }
  return flag;
}

`

If the first_part returns false, then the second_part is active.  A second sample of errors terms is required
`
bool second_part(int index, double error) {
  if (error < best_error) {
    best_error = error;
    dp[index] *= up_multiplier;
    print_vals("second part: error is better");
  } else {
    p[index] += dp[parameter_index];
    dp[index] *= down_multiplier;
    print_vals("error > best_error--set first = true");
  }  //end of best_error
  return true; //resets flag  
}
`

For the twiddle program, The size of the vector is up to the user, as is the range of offsets and the range on intervals used.  The program uses random sampling to obtain a mix of these values; therefore, it 100 is listed for the offset, each iteration will have an offset from 0:100 frames.  An improvement would be to specify a beginngin frame; however, long offsets increase the time required and is not practical given GPU time allotment.  

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./build/pid .095, .00035, 1. .1 0 .3`.

5. To run Twiddle: ./twiddle <parameter list> (see below) The executiable is in the main directory, not source. 

I provide ./twiddle as an executable only. To run it:  `./twiddle` with 6 arguments, initial Kp, Ki, and Kd values and offset_max, interval_max, and max_n. For example `./twiddle .095, .00035, .1 200 4 100`. The offset_max and interval_max variables are the upper range for the randomization of offset and interval mentioned above.  The max_n variable is the size of the vector of error terms used to adjust p and delta_p.  A larger value of max_n provides more stable estimates and the cost of time.  

## Web sockets interface.
Some complexity in understanding `main.cpp` comes from its interface to the simulator. For example, the following function call receives incoming data from the simulator: `h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) 
'

### Simulator.
This code does not run on real autonomous vehicles but instead relies on a simulator.  In order to run this code, you must download the Term3 Simulator contining the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2)].  
To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```



## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
