/* UNCO_polynomial_optimization.cpp
 *
 * Marcus Abate 2018
 *

 IGNORE THIS FILE; IT IS INCOMPLETE


 * This algorithm optimizes the trajectory of a drone as given by the waypoints
 * it follows.
 * It is based heavily on the work done on nonlinear optimization by ethz-asl
 * group in their project, mav_trajectory_generation found at:
 * https://github.com/ethz-asl/mav_trajectory_generation
 *
 * NOTES:
 *    first go at this looks identical to matlab code, probably is really slow.
 *    need to figure out how to speed it up, probably mirror the work done in
 *    polynomial_optimization_nonlinear.h and its implementation
 *
 *    need to figure out what other optimization params are needed.
 *    check the work done in polyomial_optimization_nonlinear.h
 *
 */

#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/trajectory.h>

namespace mav_trajectory_generation {

// Structure holding all the important parameters for optimziation using UNCO.
struct OptimizationParameters {
  OptimizationParameters()
    // Default parameters should be good enough for optimization in general cases:
    : poly_order(9),
      C_penalty({0,0,0,1,0,0,0,0,0}) {}

  // The order of the polynomial. Doesn't include the zeroth order, so a 9th order poly
  // will have 10 coefficients.
  int poly_order;

  // Array of user-defined penalties on every derivative of the polynomial
  int C_penalty [poly_order];

}

// The class that performs optimization on the given problem.
class UNCO_polynomial_optimization {
  public:

    int numseg;               // The number of segments in the optimization
    int num_waypoints;        // The number of waypoints, not including start and goal points
    Vertex::Vector waypoints; // Holds the waypoints that the drone must follow
    int times [this.numseg];  // The runtime associated with each segment.
    type startstate;          // Desired starting state - [m x d] matrix where m is the nr of
                              // derivatives we wish to specify, starting from the 0th
    type goalstate;           // Desired goal state - same format as startstate
    vector poly;              // Polynomail coefficients after optimization
    type A_0;                 // Maps coeffs of poly to derivatives of poly at start
    type A_tau;               // Maps coeffs of poly to derivatives of poly at end
    type A;                   // Unconstrained A matrix
    type b_f;                 // Fixed derivatives (start and goal points)
    type b;                   // Unconstrained b vector
    type b_all;               // b vector with all polynomial segments together
    type Q;                   // Cost matrix to penalize polynomial derivatives
    type C;                   // Permutation matrix for converting to an unconstrained problem
    type R;                   // Augmented cost matrix
    type R_ff;                // Fixed-fixed cost matrix quadrant
    type R_pp;                // Free-free cost matrix quadrant
    type R_fp;                // Fixed-free cost matrix quadrant
    type R_pf;                // Free-fixed cost matrix quadrant
    type A_blk;               // Block diagonal for A
    type Q_blk;               // Block diagonal for Q
    type b_p_opt;             // Optimal free derivative values after UNCO completes

    // Parameterized constructor. Sets class attributes needed to perform optimization.
    UNCO_polynomial_optimization(Vertex::Vector wp,
                            const OptimizationParameters& parameters) {
      waypoints = wp;
      num_waypoints = waypoints.size();
      numseg = num_waypoints + 1;
    }

    // Runs the optimization until a stop criterion is met. Returns stop criterion code.
    int optimize() {
      this.build_Q();
      this.build_A();
      this.build_blkdiag();
      this.build_C();
      this.build_R();
      this.build_poly();
      //this.display_stuff();

      return 0; // needs to have cases for different situations
    }

    void build_Q() {
      for (int k=0; k<this.numseg; k++)
      {
        double tau = unco.times(k);
        this.Q
      }
    }

} // UNCO_polynomial_optimization class
} // namespace
