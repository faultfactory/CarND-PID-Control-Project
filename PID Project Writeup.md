# PID Project Writeup

For this project, the task was to implement a PID controller that would adequately steer the simulated vehicle around the track.  Since the basic PID controller code is covered in the lesson and the Q&A by David Silver, the majority of this task consisted of tuning and understanding the algorithm. 

### PID Code:

The code used does not vary very much from what is provided in the Q&A session as it was a cleaner C++ implementation than I had originally attempted based on the python code. I added some minor tweaks that assisted in tuning which I will highlight here. 

###### Printing Error Terms: 

I found it useful to print out the error terms for the individual components of the PID when tuning to try to determine influence of each of the parameters. I added this Print method to the PID class which allowed me to see each contribution. Since the values are comma separated, it could be easily imported to Excel or Python, etc and plotted by pushing the command line output to a file using the >> operator in the terminal. I did not need to go that far though.  PID.cpp contains the following implementation. The appropriate function prototype exists in PID.h

```c++

void PID::PrintError() {
    // Prints each terms contribution so i can drag and drop 
std::cout<<"PID,"<<(Kp*p_error)<<","<<(Ki*i_error)<<","<<(Kd*d_error)<<std::endl;
```

###### Command Line Arguments:

One feature I found from the Q&A very helpful was the ability to pass command line arguments, however I wanted to be able to default to manually entered values so that I could quickly run a side-by-side comparison if i wanted to. I also didn't want to have a Segmentation fault if I forgot to pass an argument. 

```c++
 // value passing strategy pulled from David Silvers Q&A
  double Kp_strt, Ki_strt, Kd_strt;
    
  if(argc>1){
    Kp_strt=atof(argv[1]);
    Ki_strt=atof(argv[2]);
    Kd_strt=atof(argv[3]);
  }
  else{
    Kp_strt=-0.1;
    Ki_strt=0.0;
    Kd_strt=-1.1;  
  }
  
pid.Init(Kp_strt,Ki_strt,Kd_strt);
```

This essentially executes the same procedure with a default to predetermined values if no command line arguments are specified. 

### Problem Analysis: 

From the start of the project, I noticed that the unlike the previous exercises with the simulator, this project did not include any indication of time between measurements or clock rate. While it is safe to assume that these are fixed for the purposes of an exercise (and in many cases in industry), I would have preferred this to be provided so that irregular execution timing can be accomodated when calculating the derivative and integral error terms.  This is good information to have in the event that project constraints elsewhere determine that code needs to run faster or slower. In this case the PID should only require minor adjustment. 

Having a background in vehicle controls it was helpful for me to analyze the error and control terms in the context of the system. Changes to the cross track error (CTE) provided by the simulator as feedback to the PID obviously were influenced by pinion angle (PA). However, how quickly changes in PA would influence CTE are highly dependent on speed and the attitude of the vehicle relative to the "zero-error" path. For this exercise, we did not have access to the path. While speed was provided as feedback, throtte was fixed which did not provide adequate controls on speed which left it variable. While such constraints are unlikely in an industry setting, I would likely have attempted to modify the PID K terms based on vehicle speed (gain scheduling).

Instead of overcomplicating the steering approach, I simply added a secondary PID controller on throttle that provided a stable speed. That allowed my distance traveled per clock to remain as constant as possible and would create a stable basis on which to begin tuning. I set this speed control PID to 30kph.

### Speed Control Implementation:

In main.cpp, the PID object for the speed controller was created and initialized. Numbers on the left reflect the line numbers in the file. 

```C++
39  PID spdctrl;

58  spdctrl.Init(-0.05,-0.0001,-0.01);
```

In the main loop, throttle and speed_target variables were declared.  The throttle value being passed out via Json is then updated using the spdctrl pid and the calculated error. 

``` c++
80	double throttle;
81 	double speed_target = 30.0;


92 	spdctrl.UpdateError(speed-speed_target);
93	throttle = spdctrl.TotalError();

103 msgJson["throttle"] = throttle;
```

The tunings above a fairly very stable output at 30kph.

### Steering PID Tuning:

The approach I used for this was manual tuning based on insight and experience I've had. Not all of that experience was applicable because much of what I have worked with can be termed a 'self regulating' system. In those cases a stable control value will produce a stable process value once an equilibrium is reached.  These systems benefit from time spent tuning Ki terms to help with various non-linearities in the actuator and system.  

The steering problem presented is what could be termed an 'integrating' system which means that any steering value beyond what is used to absorb a bias, will continue to influence our CTE until it is actively restored to a stable state. Note that this label does not completely fit because the car needs to be steered slightly in the opposite direction to correct its attitude. 

###### P term:

I began tuning the Kp term to achieve an acceptable level of correction into the first turn of the track. As expected, this led to oscillations that I could see develop in the command window output.  When the the car stayed on the inside of the pavement for the first turn I moved on to other terms. At this point the Kp was set to  -0.15.

######I term:

I added some Ki as an experiment but I did not expect this to be fruitful since the Q&A described this system as being symmetric with no built in 'wheel mis-alignment' issues. With Ki set to -0.0001, the P and I error terms oscillated between adding together and cancelling each other out. The car at this point wildly swerved and was unstable when disturbed.

To prevent oscillations driven by the I-term, typically PID's will incorporate an integrator reset and an external controls model will determine the appropriate time to reset this to zero. If I believed it would have had a great influence I would have attempted to add it.  Based on the dynamic nature of the problem and the strength of the error integration mainly being removal of static biases, I moved on to tuning Kd.  

###### D term:

My work experience with DC motor based actuator controls showed me the true strength of the derivative terms, especially when an integrating system must respond quickly to changes to set point targets. The derivative gain is classically demonstrated as a way to reduce overshoot during a step input. Another benefit it bring is to boost the initial response to a change in error.  In this case, the CTE changes rapidly if the car continues straight into a curve without steering appropriately. 

I began adding Kd, starting with -0.2 using the value of Kp as a ball park. Running with this value showed that the oscillations were reduced or 'damped'. This makes sense as the D term counters large rates of change and is analogous to damping in dynamic systems. 
Since the derivative calculation does not account for the time between measurements, the delta CTE between measurements is quite small. Knowing this, I doubled the absolute value of the Kd until its influence made the system unstable. At this point which was -1.6, i split the difference between that and my prior increment(-1.2). The system was fairly stable so I moved back to Kp

#####Iterative tweaking:

Through several cycles of running and tweaking the Kp and Ki values by approximately 20% or so in either direction, I arrived at the current value of -0.1, 0 and -1.1 for Kp, Ki and Kd respectively. There specific instances where the car comes close to leaving the driveable surface but it remains steady and continues on. I likely could re-tune this for a higher speed or more accuracy but with only a PID and CTE, I feel there's only so far this can be taken within the bounds of the project. I am eager to move on to the MPC project so I am happy to leave this here and come back to this for a challenege later. 