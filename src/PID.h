#ifndef PID_H
#define PID_H

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  // add for twiddle
  double dp[3] = {0.1, 0.00005, 0.5};
  bool is_twiddling = true;
  double total_error = 0.0;
  double best_error = 0.0;
  int iteration_for_twiddle = 0;
    double tolerace = 0.0001;
  int twiddle_state;
  int twiddling_index;
    double best_Kp;
    double best_Ki;
    double best_Kd;
  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
  
  // for twiddling
  bool should_restart_twiddle_iteration();
    
  void twiddle();
  void updateIndex(int index, double value);
};

#endif /* PID_H */
