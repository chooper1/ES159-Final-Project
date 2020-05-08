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
#define MAX_NUMBER_OBS 10
#define MAX_NUMBER_NODES 7500
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

    RRT::InputType start_position_list[2];
    start_position_list[0] = 0;
    start_position_list[1] = 0;
    RRT::InputType end_position_list[2];
    end_position_list[0] = PI;
    end_position_list[1] = 0;
    int N = 1; // N == number of obstacles
    RRT::InputType obs_list[N][3];
    obs_list[0][0] = 1;
    obs_list[0][1] = 1;
    obs_list[0][2] = 0.5;

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

    // for (int i=0; i< N; i++) {
    //     tmp = obs_list[i][0];
    //     obstacles[0].Push(tmp);
    //     tmp = obs_list[i][1];
    //     obstacles[1].Push(tmp);
    //     tmp = obs_list[i][2];
    //     obstacles[2].Push(tmp);
    //     wait();
    // }

    wait(5);
  }// void run()

  void end() {
    //configurations.Reset();
    for (int i=0; i < 2; i++) {
      configurations[i].Reset();
    }
    wait();
    while (1) {
      RRT::OutputType tmp;
      tmp = configurations[0].Pop();
      cout << tmp << " , " ;
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
    wait(1000000,SC_NS);
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
