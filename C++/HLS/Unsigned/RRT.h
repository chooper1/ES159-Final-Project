/*
 * Design of an RRT Accelerator
 */

#ifndef __RRT_H__
#define __RRT_H__

#include <systemc.h>
#include <nvhls_int.h>
#include <nvhls_connections.h>

#include <ac_int.h>
#include <ac_fixed.h>
#include <ac_float.h>
#include <ac_complex.h>
#include <ac_math.h>

#include <stdlib.h>
#include <time.h>

#ifndef CONSTANTS
#define MAX_NUMBER_OBS 10
#define MAX_NUMBER_NODES 10000
#endif

using namespace std;

SC_MODULE(RRT)
{
  public:
  sc_in_clk     clk;
  sc_in<bool>   rst;

  // typedef ac_float<30, 15, true, AC_RND, AC_SAT> InputType;
  // typedef ac_float<30, 15, true, AC_RND, AC_SAT> OutputType;
  typedef ac_fixed<30, 15, false, AC_RND, AC_SAT> InputType;
  typedef ac_fixed<30, 15, false, AC_RND, AC_SAT> OutputType;
  typedef ac_complex<ac_fixed<30, 15, false, AC_RND, AC_SAT> > ComplexType;

  Connections::In<InputType>   start_position[2];
  Connections::In<InputType>   end_position[2];
  Connections::In<InputType>   obstacles[3]; // need to read in obs 1 at a time
  Connections::Out<OutputType>  configurations[2]; // need to read out configs 1 at a time


  SC_HAS_PROCESS(RRT);
  RRT(sc_module_name name_) : sc_module(name_) {
    SC_THREAD (run);
    sensitive << clk.pos();
    NVHLS_NEG_RESET_SIGNAL_IS(rst);
  }

  void run() {
    for (int i=0; i < 2; i++) {
      start_position[i].Reset();
      end_position[i].Reset();
      configurations[i].Reset();
    }
    for (int i=0; i < 3; i++) {
      obstacles[i].Reset();
    }
    
    InputType qs[2];
    InputType tmp;
    qs[0] = start_position[0].Pop();
    qs[1] = start_position[1].Pop();

    InputType qf[2];
    qf[0] = end_position[0].Pop();
    qf[1] = end_position[1].Pop();

    InputType obs[MAX_NUMBER_OBS][3];
    int i = 0;
    wait();
    while(obstacles[0].PopNB(tmp)) {
      obs[i][0] = tmp;
      obs[i][1] = obstacles[1].Pop();
      obs[i][2] = obstacles[2].Pop();
      i++;
      wait();
    }
    int NUMBER_OBS = i;

    int num_iter = 0;
    int max_iter = MAX_NUMBER_NODES;
    InputType max_step = 0.05; // everything will be in radians!
    InputType L1 = 1;
    InputType L2 = 1;

    InputType q1_max = 180*(3.14/180);
    InputType q1_min = -180*(3.14/180);

    InputType q2_max = 180*(3.14/180);
    InputType q2_min = 0*(3.14/180);

    InputType q_rand[2];
    InputType p_rand[2];
    InputType pf[2];
    ComplexType cs1;
    ComplexType cs2;
    ac_math::ac_sincos_lut(qf[0], cs1);
    ac_math::ac_sincos_lut(qf[0]+qf[1], cs2);
    pf[0] = L1*cs1.r() + L2*cs2.r();
    pf[1] = L1*cs1.i() + L2*cs2.i();

    InputType route[MAX_NUMBER_NODES][2];
    route[0][0] = qs[0];
    route[0][1] = qs[1];
    int route_size = 1;
    int edges[MAX_NUMBER_NODES];
    edges[0] = 1;

    while (num_iter <= max_iter) {
      q_rand[0] = (q1_max - q1_min)*(rand() % 100)/100 + q1_min;
      q_rand[1] = (q2_max - q2_min)*(rand() % 100)/100 + q2_min;
      ComplexType cs1;
      ComplexType cs2;
      ac_math::ac_sincos_lut(q_rand[0], cs1);
      ac_math::ac_sincos_lut(q_rand[0]+q_rand[1], cs2);
      p_rand[0] = L1*cs1.r() + L2*cs2.r();
      p_rand[1] = L1*cs1.i() + L2*cs2.i();

      if (!inObs(p_rand, obs, NUMBER_OBS)) {
        num_iter++;
        InputType nearVertAndInd[2];
        int near_ind;
        findNearestVert(p_rand, route, nearVertAndInd, &near_ind, route_size);
        InputType q_near[2];
        InputType p_near[2];
        q_near[0] = nearVertAndInd[0];
        q_near[1] = nearVertAndInd[1];

        ComplexType cs1;
        ComplexType cs2;
        ac_math::ac_sincos_lut(q_near[0], cs1);
        ac_math::ac_sincos_lut(q_near[0]+q_near[1], cs2);
        p_near[0] = L1*cs1.r() + L2*cs2.r();
        p_near[1] = L1*cs1.i() + L2*cs2.i();
        // if close enough, then we're done! :)
        if (findNorm(p_near, pf) < max_step) {
          if (hasEdgeCollision(p_near, pf, obs, NUMBER_OBS)) {
            continue;
          }
          route[route_size][0] = qf[0];
          route[route_size][1] = qf[1];
          edges[route_size] = near_ind;
          route_size++;
          break;
        }

        // compute this from q_near, q_rand, and max_step
        InputType q_new[2];
        InputType p_new[2];
        InputType magnitude = findNorm(p_rand, p_near);
        if(magnitude > 0.001) {
          p_new[0] = ((p_rand[0] - p_near[0])/magnitude)*max_step + p_near[0];
          p_new[1] = ((p_rand[1] - p_near[1])/magnitude)*max_step + p_near[1];

          inverseKin(p_new, q_new);

          // Don't push_back if we hit in process
          if (hasEdgeCollision(p_new, p_near, obs, NUMBER_OBS)) {
            continue;
          }
          route[route_size][0] = q_new[0];
          route[route_size][1] = q_new[1];
          edges[route_size] = near_ind;
          route_size++;


        }
      }
      wait();
    }

    // Now that we've got the total graph, pick out a trajectory
    // num_iter total number of nodes
    if (num_iter <= max_iter) {
      //InputType trajectory[2];
      InputType trajectory;
      trajectory = route[route_size-1][0];
      configurations[0].Push(trajectory);
      trajectory = route[route_size-1][1];
      configurations[1].Push(trajectory);

      int next_ind = edges[route_size-1];
      InputType q_curr[2];
      q_curr[0] = route[route_size-1][0];
      q_curr[1] = route[route_size-1][1];
      int i = 1;
      while (q_curr[0] != qs[0] || q_curr[1] != qs[1] ) {
        //pass out trajectories
        trajectory = route[next_ind][0];
        configurations[0].Push(trajectory);
        trajectory = route[next_ind][1];
        configurations[1].Push(trajectory);
        q_curr[0] = route[next_ind][0];
        q_curr[1] = route[next_ind][1];
        next_ind = edges[next_ind];
        i++;
        wait();
      }
    }
  }
  //finding nearest node
  void findNearestVert(InputType p_curr[2], InputType route[MAX_NUMBER_NODES][2], InputType node[3], int* near_ind, int route_size) {
    InputType L1 = 1;
    InputType L2 = 1;
    int num_nodes = route_size;
    //float min_dist = FLT_MAX;
    ComplexType cs1;
    ComplexType cs2;
    ac_math::ac_sincos_lut(route[0][0], cs1);
    ac_math::ac_sincos_lut(route[0][0] + route[0][1], cs2);
    InputType ee_pos[2];
    ee_pos[0] = L1*cs1.r() + L2*cs2.r();
    ee_pos[1] = L1*cs1.i() + L2*cs2.i();
    InputType min_dist = findNorm(ee_pos, p_curr);
    InputType dist;

    for (int i = 0; i < num_nodes; i++) {
      ac_math::ac_sincos_lut(route[i][0], cs1);
      ac_math::ac_sincos_lut(route[i][0] + route[i][1], cs2);
      ee_pos[0] = L1*cs1.r() + L2*cs2.r();
      ee_pos[1] = L1*cs1.i() + L2*cs2.i();
      dist = findNorm(ee_pos, p_curr);

      if (dist < min_dist) {
        min_dist = dist;
        node[0] = route[i][0];
        node[1] = route[i][1];
        *near_ind = i;
      }
      wait();
    }
  }

  //norm calculation
  InputType findNorm(InputType q1[2], InputType q2[2]) {
    InputType ret;
    InputType dist = (q1[0]-q2[0])*(q1[0]-q2[0]) + (q1[1]-q2[1])*(q1[1]-q2[1]);
    ac_math::ac_sqrt_pwl(dist, ret);
    return ret;
  }

  //checking for collisions in intermediate configurations between two nodes
  bool hasEdgeCollision(InputType p1[2], InputType p2[2], InputType obstacles[MAX_NUMBER_OBS][3], int NumberObs) {
    int resolution = 25;

    InputType m = (p2[1] - p1[1])/(p2[0] - p1[0]);
    InputType xdiff = p2[0] - p1[0];

    InputType p_[2];
    p_[0] = p1[0];
    p_[1] = p1[1];

    for (int i = 2; i < (resolution); i++) {
        p_[0] = p1[0] + i*(xdiff/resolution);
        p_[1] = p1[1] + m*(p_[0]-p1[0]);

        if (inObs(p_ , obstacles, NumberObs) == true) {
            return true;
        }
        wait();
    }
    return false;
  }

  //inverse kinematics function
  void inverseKin(InputType ee_pos[2], InputType q[2]) {
    InputType L1 = 1;
    InputType L2 = 1;
    InputType px = ee_pos[0];
    InputType py = ee_pos[1];
    InputType exp_px, exp_py, exp_L1, exp_L2, exp_c2, s2;
    ac_math::ac_pow2_pwl(px,exp_px);
    ac_math::ac_pow2_pwl(py,exp_py);
    ac_math::ac_pow2_pwl(L1,exp_L1);
    ac_math::ac_pow2_pwl(L2,exp_L2);
    InputType c2 = (exp_px + exp_py - exp_L1 - exp_L2) / (2*L1*L2);
    ac_math::ac_pow2_pwl(c2,exp_c2);
    InputType square = 1-exp_c2;
    ac_math::ac_sqrt_pwl(square, s2);
    //NOTE: FIX ATAN LINES (ONE QUADRANT)
    //q[0] = ac_math::ac_atan_pwl(py/px) - ac_math::ac_atan_pwl(L2*s2/(L1 + L2*c2));
    //q[1] = ac_math::ac_atan_pwl((s2/c2)); //plus or minus
    q[0] = 0;
    q[1] = 0;
    // check joint limits?
  }

  //obstacle checking function
  bool inObs(InputType p_curr[2], InputType obstacles[MAX_NUMBER_OBS][3], int NumberObs) {
    // Make sure that ee and joint don't hit!
    InputType L1 = 1;
    InputType L2 = 1;
    InputType q_curr[2];
    inverseKin(p_curr, q_curr);

    InputType tolerance = 0.02; // parameter to judge how close the ee should ever come to obstacles
    ComplexType cs1;
    ComplexType cs2;
    ac_math::ac_sincos_lut(q_curr[0], cs1);
    ac_math::ac_sincos_lut(q_curr[0] + q_curr[1], cs2);
    InputType ax_ = L1*cs1.r() + L2*cs2.r();
    InputType ay_ = L1*cs1.i() + L2*cs2.i();
    InputType bx_ = L1*cs1.r();
    InputType by_ = L1*cs1.i();
    InputType cx_ = 0; //base_pos[0]
    InputType cy_ = 0; //base_pos[1]
    InputType ax, ay, bx, by, cx, cy, rx, ry, r, c, b, a, disc;

    for (int i = 0; i < NumberObs; i++) {
      // https://math.stackexchange.com/questions/275529/check-if-line-intersects-with-circles-perimeter
      if (obstacles[i][2] == -1) {
        return false;
      }
      rx = obstacles[i][0];
      ry = obstacles[i][1];
      r  = obstacles[i][2] + tolerance;

      // joint pos and ee pos
      ax = ax_ - rx;
      ay = ay_ - ry;
      bx = bx_ - rx;
      by = by_ - ry;

      InputType exp_ax;
      ac_math::ac_pow2_pwl(ax, exp_ax);
      InputType exp_ay;
      ac_math::ac_pow2_pwl(ay, exp_ay);
      InputType exp_r;
      ac_math::ac_pow2_pwl(r, exp_r);
      InputType exp_bx_ax;
      ac_math::ac_pow2_pwl((bx - ax), exp_bx_ax);
      InputType exp_by_ay;
      ac_math::ac_pow2_pwl((by - ay), exp_by_ay);
      InputType exp_b;
      ac_math::ac_pow2_pwl(b, exp_b);

      c = exp_ax + exp_ay - exp_r;
      b = 2*(ax*(bx - ax) + ay*(by - ay));
      a = exp_bx_ax + exp_by_ay;
      disc = exp_b - 4*a*c;
      if(disc > 0) {
        //endpoint check
        if (ax > bx) {
          if (0 < ax && 0 > bx) {
            return true;
          }
        } else {
          if (0 < bx && 0 > ax) {
            return true;
          }
        }
      }
      // base pos
      cx = cx_ - rx;
      cy = cy_ - ry;

      InputType exp_bx;
      ac_math::ac_pow2_pwl(bx, exp_bx);
      InputType exp_by;
      ac_math::ac_pow2_pwl(by, exp_by);
      InputType exp_cx_bx;
      ac_math::ac_pow2_pwl((cx - bx), exp_cx_bx);
      InputType exp_cy_by;
      ac_math::ac_pow2_pwl((cy - by), exp_cy_by);
      ac_math::ac_pow2_pwl(b, exp_b);

      c = exp_bx + exp_by - exp_r;
      b = 2*(bx*(cx - bx) + by*(cy - by));
      a = exp_cx_bx + exp_cy_by;
      disc = exp_b - 4*a*c;
      if(disc > 0) {
        //endpoint check
        if (bx > cx) {
          if (0 < bx && 0 > cx) {
            return true;
          }
        } else {
          if (0 < cx && 0 > bx) {
            return true;
          }
        }
      }
      wait();
    }
    return false;
  }
};

#endif
