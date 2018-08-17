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
  /*
  * Twiddle
  */ 
  double tolerance ;
  int t_step ;
  bool twiddle;

  double best_error;
  double err;


  std::vector<double> p = {0.0, 0.0, 0.0};
  std::vector<double> dp = {0.0, 0.0, 0.0};

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

  /*
  * Calculate the Sum of deltas.
  */
  double SumIncrements();
  /*
  * Calculate the Average Error.
  */
  double AverageError();

  void ResetSettings();

  void Restart(uWS::WebSocket<uWS::SERVER> ws);

  void Twiddle(int idx);

  void pparam();

  void perror();
};

#endif /* PID_H */
