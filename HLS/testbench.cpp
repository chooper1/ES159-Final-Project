/*
 * SysPE testbench for Harvard cs148/248 only
 */

#include "RRT.h"
#include <systemc.h>
#include <mc_scverify.h>
#include <nvhls_int.h>
#include <nvhls_connections.h>

#include <ac_int.h>
#include <ac_fixed.h>
#include <ac_float.h>
#include <ac_math.h>
#include <time.h>

#include <vector>

#define NVHLS_VERIFY_BLOCKS (RRT)
#include <nvhls_verify.h>
using namespace::std;

#include <testbench/nvhls_rand.h>

#ifndef CONSTANTS
#define CONSTANTS
#define MAX_NUMBER_OBS 200
#define MAX_NUMBER_NODES 5000
#define OBS_TOL 0.01
#define EDGECOL_RES 5
#define PI 3.1415926535
#endif

SC_MODULE (Source) {
  // I/O
  Connections::Out<RRT::InputType>  start_position[2];
  Connections::Out<RRT::InputType>  end_position[2];
  Connections::Out<RRT::InputType>  obstacles[3]; // need to read in obs 1 at a time
  Connections::Out<RRT::RandType>   rand_seed;
  Connections::In<RRT::OutputType>  configurations[2]; // need to read out configs 1 at a time

  sc_in <bool> clk;
  sc_in <bool> rst;

  int N = 1; // N == number of obstacles
  RRT::InputType start_position_list[2] {0,0};
  RRT::InputType end_position_list[2] {PI/2, 0};

  SC_CTOR(Source) {
    SC_THREAD(start);
    sensitive << clk.pos();
    async_reset_signal_is(rst, false);
    SC_THREAD(end);
    sensitive << clk.pos();
    async_reset_signal_is(rst, false);
  }

  void start() {
    // Wait for initial reset
    for (int i=0; i < 2; i++) {
      start_position[i].Reset();
      end_position[i].Reset();
    }
    for (int i=0; i < 3; i++) {
      obstacles[i].Reset();
    }
    rand_seed.Reset();
    //cout << "start loop\n"

    RRT::InputType obs_list[N][3];
    obs_list[0][0] = 1.5;
    obs_list[0][1] = 1.5;
    obs_list[0][2] = 0.25;

    for (int i = 1; i < N; i++) {
      obs_list[i][0] = 1.5;
      obs_list[i][1] = 1.5;
      obs_list[i][2] = 0.25;
    }

    // obs_list[1][0] = -1.5;
    // obs_list[1][1] = 1.5;
    // obs_list[1][2] = 0.25;
    //
    // obs_list[2][0] = -1.5;
    // obs_list[2][1] = -1.5;
    // obs_list[2][2] = 0.25;
    // //
    // obs_list[3][0] = -1.5;
    // obs_list[3][1] = -1.5;
    // obs_list[3][2] = 0.25;
    // //
    // obs_list[4][0] = -2;
    // obs_list[4][1] = 0;
    // obs_list[4][2] = 0.25;
    //
    ///////////////////////////////////////
    // obs_list[5][0] = 1.5;
    // obs_list[5][1] = 1.5;
    // obs_list[5][2] = 0.25;
    //
    // obs_list[6][0] = -1.5;
    // obs_list[6][1] = 1.5;
    // obs_list[6][2] = 0.25;
    //
    // obs_list[7][0] = -1.5;
    // obs_list[7][1] = -1.5;
    // obs_list[7][2] = 0.25;
    // //
    // obs_list[8][0] = -1.5;
    // obs_list[8][1] = -1.5;
    // obs_list[8][2] = 0.25;
    // //
    // obs_list[9][0] = -2;
    // obs_list[9][1] = 0;
    // obs_list[9][2] = 0.25;
    //
    // // obs_list[5][0] = 0;
    // // obs_list[5][1] = -2;
    // // obs_list[5][2] = 0.25;
    // //
    // // obs_list[6][0] = -1;
    // // obs_list[6][1] = 1;
    // // obs_list[6][2] = 0.25;
    // //
    // // obs_list[7][0] = -1;
    // // obs_list[7][1] = -1;
    // // obs_list[7][2] = 0.25;
    // // //
    // // obs_list[8][0] = 1;
    // // obs_list[8][1] = -1;
    // // obs_list[8][2] = 0.25;
    // // //
    // // obs_list[9][0] = -0.5;
    // // obs_list[9][1] = -0.5;
    // // obs_list[9][2] = 0.25;
    // // ///////////////////////////////////////
    // obs_list[10][0] = 1.5;
    // obs_list[10][1] = 1.5;
    // obs_list[10][2] = 0.25;
    //
    // obs_list[11][0] = -1.5;
    // obs_list[11][1] = 1.5;
    // obs_list[11][2] = 0.25;
    //
    // obs_list[12][0] = -1.5;
    // obs_list[12][1] = -1.5;
    // obs_list[12][2] = 0.25;
    // //
    // obs_list[13][0] = -1.5;
    // obs_list[13][1] = -1.5;
    // obs_list[13][2] = 0.25;
    // //
    // obs_list[14][0] = -2;
    // obs_list[14][1] = 0;
    // obs_list[14][2] = 0.25;
    //
    // // obs_list[10][0] = -0.5;
    // // obs_list[10][1] = 0.5;
    // // obs_list[10][2] = 0.25;
    // //
    // // obs_list[11][0] = 0.5;
    // // obs_list[11][1] = -0.5;
    // // obs_list[11][2] = 0.25;
    // //
    // // obs_list[12][0] = 1.75;
    // // obs_list[12][1] = 1.75;
    // // obs_list[12][2] = 0.25;
    // // //
    // // obs_list[13][0] = 1.75;
    // // obs_list[13][1] = 1.5;
    // // obs_list[13][2] = 0.25;
    // // //
    // // obs_list[14][0] = 1.5;
    // // obs_list[14][1] = 1.75;
    // // obs_list[14][2] = 0.25;
    // // ///////////////////////////////////////
    // obs_list[15][0] = 1.5;
    // obs_list[15][1] = 1.5;
    // obs_list[15][2] = 0.25;
    //
    // obs_list[16][0] = -1.5;
    // obs_list[16][1] = 1.5;
    // obs_list[16][2] = 0.25;
    //
    // obs_list[17][0] = -1.5;
    // obs_list[17][1] = -1.5;
    // obs_list[17][2] = 0.25;
    // //
    // obs_list[18][0] = -1.5;
    // obs_list[18][1] = -1.5;
    // obs_list[18][2] = 0.25;
    // //
    // obs_list[19][0] = -2;
    // obs_list[19][1] = 0;
    // obs_list[19][2] = 0.25;

    // obs_list[15][0] = 0.5;
    // obs_list[15][1] = 1.8;
    // obs_list[15][2] = 0.1;
    //
    // obs_list[16][0] = 1.8;
    // obs_list[16][1] = 0.5;
    // obs_list[16][2] = 0.1;
    //
    // obs_list[17][0] = -1;
    // obs_list[17][1] = 0;
    // obs_list[17][2] = 0.5;
    // //
    // obs_list[18][0] = 0;
    // obs_list[18][1] = -1;
    // obs_list[18][2] = 0.5;
    // //
    // obs_list[19][0] = -0.5;
    // obs_list[19][1] = -0.5;
    // obs_list[19][2] = 0.25;
    wait(200.0, SC_NS);
    wait();

    // Write wait to PE
    RRT::InputType tmp;
    tmp = start_position_list[0];
    start_position[0].Push(tmp);
    tmp = start_position_list[1];
    start_position[1].Push(tmp);
    tmp = end_position_list[0];
    end_position[0].Push(tmp);
    tmp = end_position_list[1];
    end_position[1].Push(tmp);

    srand (time(NULL));
    RRT::RandType temp = rand(); // more robust way?
    rand_seed.Push(temp);

    for (int i=0; i < N; i++) {
        tmp = obs_list[i][0];
        obstacles[0].Push(tmp);
        tmp = obs_list[i][1];
        obstacles[1].Push(tmp);
        tmp = obs_list[i][2];
        obstacles[2].Push(tmp);
        wait();
    }

    while(1) {
      wait(5);
    }
  }// void run()

  void end() {
    //configurations.Reset();
    for (int i=0; i < 2; i++) {
      configurations[i].Reset();
    }
    wait();
    //cout << "start configs\n";
    while (1) {
      RRT::OutputType tmp1, tmp2;
      tmp1 = configurations[0].Pop();
      cout << tmp1 << ", " ;
      tmp2 = configurations[1].Pop();
      cout << tmp2 << endl;
      //should write to a file here
      // RRT::InputType s1,s2,c1,c2;
      // RRT::InputType L1 = 1;
      // RRT::InputType L2 = 1;
      //
      // RRT::InputType pi = PI;
      // ac_math::ac_sin_cordic(tmp1/pi, s1);
      // ac_math::ac_cos_cordic(tmp1/pi, c1);
      // ac_math::ac_sin_cordic((tmp1+tmp2)/pi, s2);
      // ac_math::ac_cos_cordic((tmp1+tmp2)/pi, c2);
      //
      // RRT::InputType x = L1*c1 + L2*c2;
      // RRT::InputType y = L1*s1 + L2*s2;
      // cout << "x: " << x << ", y: " << y << endl;

      if(tmp1 == start_position_list[0] && tmp2 == start_position_list[1]) {
        cout << "@" << sc_time_stamp() << " Solution Reached" << endl ;
      }
      wait();
    }
  }// void pop_result()
};

SC_MODULE (testbench) {
  sc_clock clk;
  sc_signal<bool> rst;

  Connections::Combinational<RRT::InputType> start_position[2];
  Connections::Combinational<RRT::InputType> end_position[2];
  Connections::Combinational<RRT::InputType> obstacles[3];
  Connections::Combinational<RRT::RandType> rand_seed;
  Connections::Combinational<RRT::OutputType> configurations[2];

  NVHLS_DESIGN(RRT) RRT_MOD;
  Source src;

  SC_HAS_PROCESS(testbench);
  testbench(sc_module_name name)
  : sc_module(name),
    clk("clk", 1, SC_NS, 0.5,0,SC_NS,true),
    rst("rst"),
    RRT_MOD("RRT_MOD"),
    src("src")
  {
    RRT_MOD.clk(clk);
    RRT_MOD.rst(rst);
    RRT_MOD.rand_seed(rand_seed);
    for (int i=0; i < 2; i++) {
      RRT_MOD.start_position[i](start_position[i]);
      RRT_MOD.end_position[i](end_position[i]);
      RRT_MOD.configurations[i](configurations[i]);
    }
    for (int i=0; i < 3; i++) {
      RRT_MOD.obstacles[i](obstacles[i]);
    }

    src.clk(clk);
    src.rst(rst);
    src.rand_seed(rand_seed);
    for (int i=0; i < 2; i++) {
      src.start_position[i](start_position[i]);
      src.end_position[i](end_position[i]);
      src.configurations[i](configurations[i]);
    }
    for (int i=0; i < 3; i++) {
      src.obstacles[i](obstacles[i]);
    }

    SC_THREAD(run);
  }

  void run() {
    rst = 1;
    wait(10.5, SC_NS);
    rst = 0;
    cout << "@" << sc_time_stamp() << " Asserting Reset " << endl ;
    wait(1, SC_NS);
    cout << "@" << sc_time_stamp() << " Deasserting Reset " << endl ;
    rst = 1;

    wait(20000000,SC_NS);
    cout << "@" << sc_time_stamp() << " Stop " << endl ;
    sc_stop();
  }
};


int sc_main(int argc, char *argv[])
{
  nvhls::set_random_seed();
  testbench my_testbench("my_testbench");
  sc_start();
  //cout << "CMODEL PASS" << endl;
  return 0;
};
