# SDC-T2-P4
### Uda SDC CarND Term2 P4 
---
## describes the effect of the P, I, D component of the PID algorithm in their implementation
---
* P error (combine with weight Kp, the proportional gain), and is proportional to CTE (cross-track error). It determines the speed of the controller as response to the CET signal. When Kp goes up, the controller response more sharply to the CTE, and it helps with sharp turns but also it would cause the car to oscillate around the target trajectory, which leads to crash on the curb.

* I error (combine with weight Ki, the integral gain) is proportional to accumulated CTE over time. When Ki goes up, it helps the car converge to the target trajectory more quickly (to the setpoint) and eliminates the residual steady-state error that occurs with a pure proportional controller (using above only). However, it can cause the present value to overshoot the setpoint value.

* D error ((combine with weight Kd, the derivative gain) is proportional to the slope of the CTE over time. When Kd goes up, derivative action predicts system behavior and thus improves settling time and stability of the system, which reduce the oscillation. It serves as an additional low-pass filtering for the derivative term to limit the high-frequency gain and noise.

*the descriptions are my interpretion on Wiki(https://en.wikipedia.org/wiki/PID_controller) as well as what I learned in the course and experienced in the project.*

---

## Student discusses how they chose the final hyperparameters (P, I, D coefficients)
---
First I tried to set them manually, I do it one by one (Kp, Kd, Ki) by testing them on the simulator directly. After several experiment I set it with Kp = 0.25, Ki = 0.0001 and Kd = 4.0. It seems OK to drive on the road and complete the loops.

Then, I tried to use the **twiddle** method learned from the course. While, the implementation is a little bit different from the course since it is hard to directly calculate CTE:

### Twiddle implemenation:
---
```c++
void PID::twiddle();
```
The twiddle code tried to do the twiddle method, since it cannot directly call *run(robot,p_new)*, I use two assist parameter to record which coeffient is currently tuning and which state it is (have it already +dp or -2\*dp), *twiddling_index* and *twiddle_state* respectively.  

```c++
bool PID::should_restart_twiddle_iteration();
```
This part of code serves as calling the *run(robot,p_new)*, there are 3 parameters here: *is_twiddling* is the flag to determine if the code in twiddle mode (it is set to false now to turn off twiddle); *tolerance* to stop the twiddling if the dp is too small to affect the performance; and there is a hard code *4000*, which is actually how many msg I count as one loop (or at least sufficient to decide the total error), I set it to 4000 which is a little shorter than one loop (maybe there is a better way to set this, I didn't explore the websocket protocol a lot).  

```c++
 if (pid.should_restart_twiddle_iteration()) { // restart count define 1 iteration
          std::cout << "iteration: " << pid.iteration_for_twiddle / 2000 << std::endl;
          std::cout << "total error: " << pid.total_error << std::endl;
          std::cout << "best error: " << pid.best_error << std::endl;
          std::cout << "PID: " << pid.Kp << "," << pid.Ki << "," <<pid.Kd<< std::endl;
          pid.twiddle();
          // restart
          pid.i_error = 0.0;
          std::string reset_msg = "42[\"reset\",{}]";
          ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
```
In the main code, I restart the simulater to "implement" *run(robot,p_new)*.  

There is one more thing, since I've found a set of good hyperparameters, I start with them Kp = 0.25, Ki = 0.0001 and Kd = 4.0 (not 0,0,0 which will cause a lot of crash on curbs and waste a lot of iterations), and set dp to similar magnitude accordingly.
---

### Twiddle Result
Perhaps due to my tolerance is set too small, I ran more than 300 loops and there is no clue it will end soon ... And I picked from one of my twiddling result to reset the Kp = 0.239388, Ki = 0.000107181 , Kd = 5.13356. A sample of the twiddling log could be found in output folder (which is not the one I draw the parameters from, and the car behaves differently each time). 
*In the code, I didn't divide the total_error by N, since the N is always 4000*

### Twiddle Further Thinking
I think the tolerance magnitude is fine in practice but it takes too long to converge. If there is more time, I think a Bayesian method or SGD is helpful here. However, from my opinion, the most promising method is a genetic algorithm. The twiddle procedure is somehow similar to genetic generations, I could define the crossover as exchanging the Kp, Ki, Kd and mutation is similar to +/-dp, and dp itself is annealing by iteration. That is my guess and not tested yet.
