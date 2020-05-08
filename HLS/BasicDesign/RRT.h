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
#define CONSTANTS
#define MAX_NUMBER_OBS 200
#define MAX_NUMBER_NODES 5000
#define OBS_TOL 0.01
#define EDGECOL_RES 5
#define PI 3.1415926535
#endif

using namespace std;

SC_MODULE(RRT)
{
  public:
  sc_in_clk     clk;
  sc_in<bool>   rst;

  typedef ac_fixed<62, 30, true> InputType;
  typedef ac_fixed<62, 30, true> OutputType;
  typedef ac_fixed<62, 30, false> UnsignedType;
  typedef ac_complex<InputType> ComplexType;
  typedef NVINT32 RandType;

  Connections::In<InputType>   start_position[2];
  Connections::In<InputType>   end_position[2];
  Connections::In<InputType>   obstacles[3]; // need to read in obs 1 at a time
  Connections::In<RandType>    rand_seed; // need to read in obs 1 at a time
  Connections::Out<OutputType> configurations[2]; // need to read out configs 1 at a time

  SC_HAS_PROCESS(RRT);
  RRT(sc_module_name name_) : sc_module(name_) {
    SC_THREAD (run);
    sensitive << clk.pos();
    NVHLS_NEG_RESET_SIGNAL_IS(rst);
  }

  void run() {
    wait();
    while(1) {

      InputType qs[2];
      InputType tmp;
      qs[0] = start_position[0].Pop();
      qs[1] = start_position[1].Pop();

      InputType qf[2];
      qf[0] = end_position[0].Pop();
      qf[1] = end_position[1].Pop();

      RandType rnd_seed = rand_seed.Pop();

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
      int num_obs = i;
      int max_iter = MAX_NUMBER_NODES;
      InputType max_step = 0.25; // everything will be in radians!

      InputType q1_max = PI;
      InputType q1_min = -PI;
      InputType q2_max = PI;
      InputType q2_min = 0;

      InputType q_rand[2];

      InputType route[MAX_NUMBER_NODES][2];
      route[0][0] = qs[0];
      route[0][1] = qs[1];
      int num_iter = 1;
      int edges[MAX_NUMBER_NODES];
      edges[0] = 1;

      while (num_iter <= max_iter) {
        rnd_seed = rand_inp(rnd_seed);
        q_rand[0] = (q1_max - q1_min)*(((unsigned) rnd_seed) % 100)/100 + q1_min;
        rnd_seed = rand_inp(rnd_seed);
        q_rand[1] = (q2_max - q2_min)*(((unsigned) rnd_seed) % 100)/100 + q2_min;

        InputType nearVertAndInd[2];
        int near_ind;
        findNearestVert(q_rand, route, nearVertAndInd, &near_ind, num_iter);
        InputType q_near[2];
        q_near[0] = nearVertAndInd[0];
        q_near[1] = nearVertAndInd[1];

        // if close enough, then we're done! :)
        if (findNorm(q_near, qf) < max_step) {
          if (hasEdgeCollision(q_near, qf, obs, num_obs)) {
            continue;
          }
          route[num_iter][0] = qf[0];
          route[num_iter][1] = qf[1];
          edges[num_iter] = near_ind;
          num_iter++;
          break;
        }
        // compute this from q_near, q_rand, and max_step
        InputType q_new[2];
        InputType magnitude = findNorm(q_rand, q_near);
        if (magnitude > 0.0001) {
          if (magnitude > max_step) {
            q_new[0] = ((q_rand[0] - q_near[0])/magnitude)*max_step + q_near[0];
            q_new[1] = ((q_rand[1] - q_near[1])/magnitude)*max_step + q_near[1];
          } else {
            q_new[0] = q_rand[0];
            q_new[1] = q_rand[1];
          }
          // Don't push_back if we hit in process
          // !inObs(q_new, obs, NUMBER_OBS)
          if (!hasEdgeCollision(q_new, q_near, obs, num_obs)) {
            route[num_iter][0] = q_new[0];
            route[num_iter][1] = q_new[1];
            edges[num_iter] = near_ind;
            num_iter++;
          }
        }
        wait();
      }

      // Now that we've got the total graph, pick out a trajectory
      // num_iter total number of nodes
      if (num_iter <= max_iter) {
  	  // printf("Making traj \n");
        InputType trajectory;
        trajectory = route[num_iter-1][0];
        configurations[0].Push(trajectory);
        trajectory = route[num_iter-1][1];
        configurations[1].Push(trajectory);

        int next_ind = edges[num_iter-1];
        InputType q_curr[2];
        q_curr[0] = route[num_iter-1][0];
        q_curr[1] = route[num_iter-1][1];

        int i = 1;
        while (q_curr[0] != qs[0] || q_curr[1] != qs[1] ) {
          //pass out trajectories
  	    	// printf("Found the beginning \n");
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
        wait();
        //cout << "@" << sc_time_stamp() << " configs done." << endl;
        //cout << "number of obs: "<< NUMBER_OBS << endl;
      } //else {
  		  //cout << "Didn't get to the target" << endl;
        //push some value to tell that trajectory wasn't found
  	  //}
    }
  }
  //finding nearest node
  void findNearestVert(InputType q_curr[2], InputType route[MAX_NUMBER_NODES][2], InputType node[3], int* near_ind, int route_size) {
    InputType config[2];
    config[0] = route[0][0];
    config[1] = route[0][1];

    InputType min_dist = findNorm(config, q_curr);
    InputType dist;
    node[0] = route[0][0];
    node[1] = route[0][1];
    *near_ind = 0;

    for (int i = 0; i < route_size; i++) {
      config[0] = route[i][0];
      config[1] = route[i][1];

      dist = findNorm(config, q_curr);

      if (dist < min_dist) {
        min_dist = dist;
        node[0] = route[i][0];
        node[1] = route[i][1];
        *near_ind = i;
      }
      wait();
    }
  }

  //https://www.christianpinder.com/articles/pseudo-random-number-generation/
  //random num generator
  RandType rand_inp(RandType rnd_seed)
  {
    RandType k1;
    RandType ix = rnd_seed;

    k1 = ix / 127773;
    ix = 16807 * (ix - k1 * 127773) - k1 * 2836;
    if (ix < 0)
        ix += 2147483647;
    rnd_seed = ix;
    return rnd_seed;
  }
  //norm calculation
  InputType findNorm(InputType q1[2], InputType q2[2]) {
    InputType ret;
    InputType dist = (q1[0]-q2[0])*(q1[0]-q2[0]) + (q1[1]-q2[1])*(q1[1]-q2[1]);

    UnsignedType arg1, arg2;
    arg1 = (UnsignedType) dist;
    ac_math::ac_sqrt_pwl(arg1, arg2);
    ret = (InputType) arg2;
    return ret;
  }

  //checking for collisions in intermediate configurations between two nodes
  bool hasEdgeCollision(InputType q1[2], InputType q2[2], InputType obstacles[MAX_NUMBER_OBS][3], int NumberObs) {
    int resolution = EDGECOL_RES;
    InputType xdiff = q2[0] - q1[0];
    InputType m;
    if (xdiff != 0) {
      m = (q2[1] - q1[1])/(q2[0] - q1[0]);
    } else { //completely vertical
      m = q2[1] - q1[1];
    }

    InputType q_[2];
    q_[0] = q1[0];
    q_[1] = q1[1];

    for (int i = 1; i <= (resolution); i++) {
        q_[0] = q1[0] + i*(xdiff/resolution);
        if (xdiff != 0) {
          q_[1] = q1[1] + m*(q_[0]-q1[0]);
        } else {
          q_[1] = q1[1] + i*m/resolution;
        }

        if (inObs(q_ , obstacles, NumberObs) == true) {
          return true;
        }
        wait();
    }
    return false;
  }

  //obstacle checking function
  bool inObs(InputType q_curr[2], InputType obstacles[MAX_NUMBER_OBS][3], int NumberObs) {
    // Make sure that ee and joint don't hit!
    InputType L1 = 1;
    InputType L2 = 1;

    InputType tolerance = OBS_TOL; // parameter to judge how close the ee should ever come to obstacles
    InputType s1,s2,c1,c2;
    InputType pi = PI;
    InputType q_1 = q_curr[0]/pi;
    InputType q_2 = q_curr[1]/pi;
    ac_math::ac_sin_cordic(q_1, s1);
    ac_math::ac_cos_cordic(q_1, c1);
    ac_math::ac_sin_cordic(q_1 + q_2, s2);
    ac_math::ac_cos_cordic(q_1 + q_2, c2);

    InputType ax_ = L1*c1 + L2*c2;
    InputType ay_ = L1*s1 + L2*s2;
    InputType bx_ = L1*c1;
    InputType by_ = L1*s1;
    InputType cx_ = 0;
    InputType cy_ = 0;
    InputType ax, ay, bx, by, cx, cy, rx, ry, r, c, b, a, disc;
    //InputType pos1[2], pos2[2];
    InputType exp_ax, exp_ay, exp_r, exp_bx_ax, exp_by_ay, exp_b, exp_bx, exp_by, exp_cx_bx, exp_cy_by;

    for (int i = 0; i < NumberObs; i++) {
      // https://math.stackexchange.com/questions/275529/check-if-line-intersects-with-circles-perimeter
      rx = obstacles[i][0];
      ry = obstacles[i][1];
      r  = obstacles[i][2] + tolerance;

      // joint pos and ee pos
      ax = ax_ - rx;
      ay = ay_ - ry;
      bx = bx_ - rx;
      by = by_ - ry;

      exp_ax = ax * ax;
      exp_ay = ay * ay;
      exp_r = r*r;
      exp_bx_ax = (bx-ax)*(bx-ax);
      exp_by_ay = (by-ay)*(by-ay);

      c = exp_ax + exp_ay - exp_r;
      b = 2*(ax*(bx - ax) + ay*(by - ay));
      a = exp_bx_ax + exp_by_ay;
      exp_b = b*b;
      disc = exp_b - 4*a*c;
      if(disc >= 0) {
        // endpoint check
        if (ax >= bx) {
          if (0 <= ax && 0 >= bx) {
            return true;
          }
        } else {
          if (0 <= bx && 0 >= ax) {
            return true;
          }
        }
      }

      // base pos
      cx = cx_ - rx;
      cy = cy_ - ry;

      exp_bx = bx*bx;
      exp_by = by*by;
      exp_cx_bx = (cx-bx)*(cx-bx);
      exp_cy_by = (cy-by)*(cy-by);

      c = exp_bx + exp_by - exp_r;
      b = 2*(bx*(cx - bx) + by*(cy - by));
      a = exp_cx_bx + exp_cy_by;
      exp_b = b*b;
      disc = exp_b - 4*a*c;
      if(disc >= 0) {
        //endpoint check
        if (bx >= cx) {
          if (0 <= bx && 0 >= cx) {
            return true;
          }
        } else {
          if (0 <= cx && 0 >= bx) {
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
