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

#include <vector>

#define NVHLS_VERIFY_BLOCKS (RRT)
#include <nvhls_verify.h>
using namespace::std;

#include <testbench/nvhls_rand.h>

#ifndef CONSTANTS
#define CONSTANTS
#define MAX_NUMBER_OBS 20
#define MAX_NUMBER_NODES 15000
#define PI 3.1415926535
#endif

SC_MODULE (Source) {
  // I/O
  // typedef ac_fixed<30, 15, false, AC_RND, AC_SAT> InputType;
  // typedef ac_fixed<30, 15, false, AC_RND, AC_SAT> OutputType;
  Connections::Out<RRT::InputType>   start_position[2];
  Connections::Out<RRT::InputType>   end_position[2];
  Connections::Out<RRT::InputType>   obstacles[3]; // need to read in obs 1 at a time
  Connections::In<RRT::OutputType>  configurations[2]; // need to read out configs 1 at a time

  sc_in <bool> clk;
  sc_in <bool> rst;

  SC_CTOR(Source) {
    SC_THREAD(start);
    sensitive << clk.pos();
    async_reset_signal_is(rst, false);
    SC_THREAD(end);
    sensitive << clk.pos();
    async_reset_signal_is(rst, false);
  }

  void start() {
    for (int i=0; i < 2; i++) {
      start_position[i].Reset();
      end_position[i].Reset();
    }
    for (int i=0; i < 3; i++) {
      obstacles[i].Reset();
    }
    //cout << "start loop\n";
    RRT::InputType start_position_list[2];
    start_position_list[0] = 0;
    start_position_list[1] = 0;
    RRT::InputType end_position_list[2];
    end_position_list[0] = PI/2;
    end_position_list[1] = 0;
    int N = 20; // N == number of obstacles
    RRT::InputType obs_list[N][3];
    obs_list[0][0] = 1.5;
    obs_list[0][1] = 1.5;
    obs_list[0][2] = 0.25;

    obs_list[1][0] = -1.5;
    obs_list[1][1] = 1.5;
    obs_list[1][2] = 0.25;

    obs_list[2][0] = -1.5;
    obs_list[2][1] = -1.5;
    obs_list[2][2] = 0.25;
    //
    obs_list[3][0] = -1.5;
    obs_list[3][1] = -1.5;
    obs_list[3][2] = 0.25;
    //
    obs_list[4][0] = -2;
    obs_list[4][1] = 0;
    obs_list[4][2] = 0.25;
    ///////////////////////////////////////
    obs_list[5][0] = 1.5;
    obs_list[5][1] = 1.5;
    obs_list[5][2] = 0.25;

    obs_list[6][0] = -1.5;
    obs_list[6][1] = 1.5;
    obs_list[6][2] = 0.25;

    obs_list[7][0] = -1.5;
    obs_list[7][1] = -1.5;
    obs_list[7][2] = 0.25;
    //
    obs_list[8][0] = -1.5;
    obs_list[8][1] = -1.5;
    obs_list[8][2] = 0.25;
    //
    obs_list[9][0] = -2;
    obs_list[9][1] = 0;
    obs_list[9][2] = 0.25;

    // obs_list[5][0] = 0;
    // obs_list[5][1] = -2;
    // obs_list[5][2] = 0.25;
    //
    // obs_list[6][0] = -1;
    // obs_list[6][1] = 1;
    // obs_list[6][2] = 0.25;
    //
    // obs_list[7][0] = -1;
    // obs_list[7][1] = -1;
    // obs_list[7][2] = 0.25;
    // //
    // obs_list[8][0] = 1;
    // obs_list[8][1] = -1;
    // obs_list[8][2] = 0.25;
    // //
    // obs_list[9][0] = -0.5;
    // obs_list[9][1] = -0.5;
    // obs_list[9][2] = 0.25;
    // ///////////////////////////////////////
    obs_list[10][0] = 1.5;
    obs_list[10][1] = 1.5;
    obs_list[10][2] = 0.25;

    obs_list[11][0] = -1.5;
    obs_list[11][1] = 1.5;
    obs_list[11][2] = 0.25;

    obs_list[12][0] = -1.5;
    obs_list[12][1] = -1.5;
    obs_list[12][2] = 0.25;
    //
    obs_list[13][0] = -1.5;
    obs_list[13][1] = -1.5;
    obs_list[13][2] = 0.25;
    //
    obs_list[14][0] = -2;
    obs_list[14][1] = 0;
    obs_list[14][2] = 0.25;

    // obs_list[10][0] = -0.5;
    // obs_list[10][1] = 0.5;
    // obs_list[10][2] = 0.25;
    //
    // obs_list[11][0] = 0.5;
    // obs_list[11][1] = -0.5;
    // obs_list[11][2] = 0.25;
    //
    // obs_list[12][0] = 1.75;
    // obs_list[12][1] = 1.75;
    // obs_list[12][2] = 0.25;
    // //
    // obs_list[13][0] = 1.75;
    // obs_list[13][1] = 1.5;
    // obs_list[13][2] = 0.25;
    // //
    // obs_list[14][0] = 1.5;
    // obs_list[14][1] = 1.75;
    // obs_list[14][2] = 0.25;
    // ///////////////////////////////////////
    obs_list[15][0] = 1.5;
    obs_list[15][1] = 1.5;
    obs_list[15][2] = 0.25;

    obs_list[16][0] = -1.5;
    obs_list[16][1] = 1.5;
    obs_list[16][2] = 0.25;

    obs_list[17][0] = -1.5;
    obs_list[17][1] = -1.5;
    obs_list[17][2] = 0.25;
    //
    obs_list[18][0] = -1.5;
    obs_list[18][1] = -1.5;
    obs_list[18][2] = 0.25;
    //
    obs_list[19][0] = -2;
    obs_list[19][1] = 0;
    obs_list[19][2] = 0.25;

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

    // Wait for initial reset
    wait(20.0, SC_NS);
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

    for (int i=0; i< N; i++) {
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
      RRT::OutputType tmp;
      tmp = configurations[0].Pop();
      cout << tmp << ", " ;
      tmp = configurations[1].Pop();
      cout << tmp << endl;
      //should write to a file here
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
    src.clk(clk);
    src.rst(rst);

    for (int i=0; i < 2; i++) {
      RRT_MOD.start_position[i](start_position[i]);
      RRT_MOD.end_position[i](end_position[i]);
      RRT_MOD.configurations[i](configurations[i]);
      src.start_position[i](start_position[i]);
      src.end_position[i](end_position[i]);
      src.configurations[i](configurations[i]);
    }
    for (int i=0; i < 3; i++) {
      RRT_MOD.obstacles[i](obstacles[i]);
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
