#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"


#define  REF_CTE  0.0  //Cross Track Error is 0 ---> Final goal
#define  REF_EPSI 0.0  // PSI Error reference as 0
#define  REF_V    30.0 // Reference velocity 30

//setting weight parameters for the cost function
#define W_CTE    2.0  // Weight for cross track error
#define W_EPSI   20.0  // Weight for PSI error
#define W_DV     1.0   // increase to remove sharp turns at high speeds
#define W_DELTA  1.0     // Weight for Delta steering
#define W_A      21.0     // Weight for acceleration
#define W_DDELTA 100.0   // Weight for delta rate. To increase to remove sharp turns
#define W_DA     0.0   // Weight for acceleration rate. To increase to remove sudden acceleration or de-acceleration

// Set lower and upper limits for variables.
#define DED25RAD 0.436332 // 25 deg in rad, used as delta bound
#define MAXTHR 1.0 // Maximum a value
#define BOUND 1.0e19 // Bound value for other variables


using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuations.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

  vector<double> mpc_x;
  vector<double> mpc_y;

};

#endif /* MPC_H */
