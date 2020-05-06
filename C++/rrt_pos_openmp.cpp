#include <vector>
#include <math.h>
#include <stdlib.h>
#include <float.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <time.h>
#include <atomic>
#include <thread>
#include <chrono> 
#include <mutex>

using namespace std;

// findNearestVert takes find the closest configuration that we've seen before to q_curr
// It returns entries with that nearest vert vec(0:1), and entry with index vec(2)

// fill this in later!

#define Q1RANGE 90
#define Q2RANGE 90
#define Q1MIN 0
#define Q2MIN 0 
#define PI 3.14159

// Running many more threads than cores degrades performance

#define NUMCORES 4

float L1 = 1;
float L2 = 1;

bool colMemo[Q1RANGE + 1][Q2RANGE + 1]; 
bool colValid[Q1RANGE + 1][Q2RANGE + 1]; 

int numColsComp; 
int numColsCall; 

int num_obs_calc; 

atomic_int numThreadComp; 
atomic_int numThreadsRunning; 
atomic_bool hasHit; 

mutex g_display_mutex;

vector<float> inverseKin(vector<float> ee_pos) {
	float px = ee_pos[0];
  	float py = ee_pos[1];
	vector<float> q(2);
	float c2 = (pow(px, 2) + pow(py, 2) - pow(L1, 2) - pow(L2, 2)) / (2*L1*L2);
  	float s2 = sqrt(1-pow(c2, 2));
	q[0] = atan2(py,px) - atan2(L2*s2, L1 + L2*c2);
 	q[1] = atan2(s2, c2); //plus or minus
	// check joint limits?
	return q;
}

// If there is a hit, toggle the atomic boolean. 
void obsChecker(vector<float> base_pos, vector<float> joint_pos, vector<float> ee_pos, vector<float> currObs) {
	// https://math.stackexchange.com/questions/275529/check-if-line-intersects-with-circles-perimeter
    // thread::id this_id = this_thread::get_id();
 
    // g_display_mutex.lock();
    // cout << "thread: " << this_id << endl;
    // g_display_mutex.unlock();

	float ax = ee_pos[0];
	float ay = ee_pos[1];
	float bx = joint_pos[0];
	float by = joint_pos[1];

	float cx = currObs[0];
	float cy = currObs[1];
	float r  = currObs[2];

	ax = ax - cx;
	ay = ay - cy;
	bx = bx - cx;
	by = by - cy;

	float c = pow(ax, 2) + pow(ay, 2) - pow(r, 2);
	float b = 2*(ax*(bx - ax) + ay*(by - ay));
	float a = pow((bx - ax), 2) + pow((by - ay), 2);
	float disc = pow(b, 2) - 4*a*c;
	if(disc > 0) {
			//endpoint check
			if (ax > bx) {
					if (0 < ax && 0 > bx) {
							hasHit = true; 
							numThreadComp++; 
							numThreadsRunning--; 
							return;
					}
			} else {
					if (0 < bx && 0 > ax) {
							hasHit = true; 
							numThreadComp++; 
							numThreadsRunning--; 
							return;
				  }
			}
	}

	ax = joint_pos[0];
	ay = joint_pos[1];
	bx = base_pos[0];
	by = base_pos[1];
	// cx = obstacles[i][0]; //redundant
	// cy = obstacles[i][1]; //redundant
	// r  = obstacles[i][2]; //redundant
	ax = ax - cx;
	ay = ay - cy;
	bx = bx - cx;
	by = by - cy;

	c = pow(ax, 2) + pow(ay, 2) - pow(r, 2);
	b = 2*(ax*(bx - ax) + ay*(by - ay));
	a = pow((bx - ax), 2) + pow((by - ay), 2);
	disc = pow(b, 2) - 4*a*c;
	if(disc > 0) {
			//endpoint check
			if (ax > bx) {
					if (0 < ax && 0 > bx) {
							hasHit = true; 
							numThreadComp++; 
							numThreadsRunning--; 
							return;
					}
			} else {
					if (0 < bx && 0 > ax) {
							hasHit = true; 
							numThreadComp++; 
							numThreadsRunning--; 
							return;
				  }
			}
	}
	numThreadsRunning--; 
	numThreadComp++; 
	return; 
}

bool inObs(vector<float> p_curr, vector<float> q_curr, vector<vector<float> > obstacles) {
	// Make sure that ee and joint don't hit!
	
	
	vector<float> ee_pos(2);
	ee_pos[0] = L1*cos(q_curr[0]) + L2*cos(q_curr[0] + q_curr[1]);
	ee_pos[1] = L1*sin(q_curr[0]) + L2*sin(q_curr[0] + q_curr[1]);

	vector<float> joint_pos(2);
	joint_pos[0] = L1*cos(q_curr[0]);
	joint_pos[1] = L1*sin(q_curr[0]);

	vector<float> base_pos(2);
	base_pos[0] = 0;
	base_pos[1] = 0;

	hasHit = false; 
	numThreadComp = 0; 

	int numThreadsSpawned = 0; 
	numThreadsRunning = 0; 

	// while (numThreadsRunning < NUMCORES + 1 && numThreadsSpawned < obstacles.size() && !hasHit) {
	// 	vector<float> currObs = obstacles[numThreadsSpawned];
	// 	numThreadsSpawned++; 
	// 	thread currThread(obsChecker, base_pos, joint_pos, ee_pos, currObs);
	// 	numThreadsRunning++; 
	// 	currThread.detach(); 
	// }

	#pragma omp parallel for

	for (int i = 0; i < obstacles.size(); i++) {
		// spawn threads here!
		vector<float> currObs = obstacles[i];
	 	obsChecker(base_pos, joint_pos, ee_pos, currObs); 
	}

	while (!hasHit && numThreadComp < obstacles.size()) {
		// poll here!
	}

	return hasHit;
}

// fill this in later also!
bool hasEdgeCollision(vector<float> p1, vector<float> p2, vector<float> q_curr, vector<vector<float> > obstacles) {

	int resolution = 25;

	float m = (p2[1] - p1[1])/(p2[0] - p1[0]);
	float xdiff = p2[0] - p1[0];

	vector<float> p_(2);
  	p_[0] = p1[0];
	p_[1] = p1[1];

	for (int i = 2; i < (resolution); i++) {
	    p_[0] = p1[0] + i*(xdiff/resolution);
	    p_[1] = p1[1] + m*(p_[0]-p1[0]);

	    if (inObs(p_ , q_curr, obstacles) == true) {
	        return true;
			}
	}
	return false;
}


float findNorm(vector<float> q1, vector<float> q2) {
	float dist = sqrt(pow(q1[0]-q2[0], 2) + pow(q1[1]-q2[1], 2));
	return dist;
}


vector<float> findNearestVert(vector<float> p_curr, vector<vector<float> > route) {
	int num_nodes = route.size();
	//float min_dist = FLT_MAX;
	vector<float> ee_pos(2);
	float ee_pos_0 = L1*cos(route[0][0]) + L2*cos(route[0][0] + route[0][1]);
	float	ee_pos_1 = L1*sin(route[0][0]) + L2*sin(route[0][0] + route[0][1]);
	ee_pos.push_back(ee_pos_0);
	ee_pos.push_back(ee_pos_1);
	float min_dist = findNorm(ee_pos, p_curr);
	float dist;

	vector<float> ans(3);

	for (int i = 0; i < num_nodes; i++) {
		ee_pos.clear();
		ee_pos_0 = L1*cos(route[i][0]) + L2*cos(route[i][0] + route[i][1]);
		ee_pos_1 = L1*sin(route[i][0]) + L2*sin(route[i][0] + route[i][1]);
		ee_pos.push_back(ee_pos_0);
		ee_pos.push_back(ee_pos_1);
		dist = findNorm(ee_pos, p_curr);

		if (dist < min_dist) {
			min_dist = dist;
			ans[0] = route[i][0];
			ans[1] = route[i][1];
			ans[2] = i;
		}
	}
	return ans;
}

void rrt(vector<float> qs, vector<float> qf, vector<vector<float> > obs) {
	int num_iter = 0;
	int max_iter = 5000;
	float max_step = 0.1;

	num_obs_calc = 0; 

	float q1_max = PI/2;
	float q1_min = 0;

	float q2_max = PI/2;
	float q2_min = 0;

	vector<float> q_rand(2);
	vector<float> p_rand(2);
  	vector<float> pf(2);

	pf[0] = L1*cos(qf[0]) + L2*cos(qf[0] + qf[1]);
	pf[1] = L1*sin(qf[0]) + L2*sin(qf[0] + qf[1]);

	vector<vector<float> > route;
	route.push_back(qs);

	vector<int> edges;
	edges.push_back(1);

	int num_loops = 0; 

	while (num_iter <= max_iter) {
		// cout << "num_loops: " << num_loops << endl; 
		// cout << "num_iter: " << num_iter << endl; 
		num_loops++; 
		q_rand[0] = (q1_max - q1_min)*(rand() % 100)/100 + q1_min;
		q_rand[1] = (q2_max - q2_min)*(rand() % 100)/100 + q2_min;

		p_rand[0] = L1*cos(q_rand[0]) + L2*cos(q_rand[0] + q_rand[1]);
		p_rand[1] = L1*sin(q_rand[0]) + L2*sin(q_rand[0] + q_rand[1]);

		if (!inObs(p_rand, q_rand, obs)) {
			num_iter++;
			vector<float> nearVertAndInd = findNearestVert(p_rand, route);
			vector<float> q_near(2);
			vector<float> p_near(2);

			// move the top angle part to q_near
			copy(nearVertAndInd.begin(), nearVertAndInd.begin() + 2, q_near.begin());
			int near_ind = (int) nearVertAndInd[2];

			p_near[0] = L1*cos(q_near[0]) + L2*cos(q_near[0] + q_near[1]);
			p_near[1] = L1*sin(q_near[0]) + L2*sin(q_near[0] + q_near[1]);
			// if close enough, then we're done! :)

			if (findNorm(q_near, qf) < max_step) {

				if (hasEdgeCollision(p_near, pf, q_near, obs)) {
					num_iter--; 
					continue; 
				}
				route.push_back(qf);
				edges.push_back(near_ind);
				break;
			}

			// compute this from q_near, q_rand, and max_step
			vector<float> q_new(2);
			vector<float> temp(2);
			vector<float> p_new(2);
			float magnitude = findNorm(q_rand, q_near);

			if(magnitude > 0.001) {
				q_new[0] = ((q_rand[0] - q_near[0])/magnitude)*max_step + q_near[0];
				q_new[1] = ((q_rand[1] - q_near[1])/magnitude)*max_step + q_near[1];

				p_new[0] = L1*cos(q_new[0]) + L2*cos(q_new[0] + q_new[1]);
				p_new[1] = L1*sin(q_new[0]) + L2*sin(q_new[0] + q_new[1]); 

				// Don't push_back if we hit in process
				if (q_new[0] > q1_max || q_new[0] < q1_min) {
					// cout << "q1 oobs rrt" << endl;
					num_iter--; 
					continue; 
				}

				if (q_new[1] > q2_max || q_new[1] < q2_min) {
					// cout << "q2 oobs rrt" << endl; 
					num_iter--; 
					continue; 
				}

				if (isnan(q_new[0]) || isnan(q_new[1])) {
					num_iter--; 
					continue; 
				}

				int q1_ind = (int) roundf(q_new[0]*(PI/180)) - Q1MIN; 
				int q2_ind = (int) roundf(q_new[1]*(PI/180)) - Q2MIN; 

				if (hasEdgeCollision(p_new, p_near, q_new, obs)) {
					num_iter--; 
					continue; 
				}
				route.push_back(q_new);
				edges.push_back(near_ind);

			} else { // to avoid div by 0 errors
				num_iter--;
			}
		}

	}
	// Now that we've got the total graph, pick out a trajectory
	// num_iter total number of nodes
	if (num_iter <= max_iter) {
		vector<vector<float> > trajectory;
		trajectory.push_back(route[num_iter]);

		int next_ind = edges[num_iter];
		vector<float> q_curr = route[num_iter];

		while (q_curr != qs) {
			trajectory.push_back(route[next_ind]);
			q_curr = route[next_ind];
			next_ind = edges[next_ind];
		}

		// maybe write this to a .csv file?

		ofstream trajfile;
		trajfile.open("traj_log.csv");
		// x, y in a row. Lines denote separate points.

		int traj_len = trajectory.size();
		for (int j = traj_len-1; j > -1; j--) {
			trajfile << trajectory[j][0] << "," << trajectory[j][1] << "\n";
	 	}

		ofstream trajfile2;
		trajfile2.open("workspace_traj_log.csv");
		// x, y in a row. Lines denote separate points.
	  	float ee_pos_0;
		float ee_pos_1;
		for (int j = traj_len-1; j > -1; j--) {
			ee_pos_0 = L1*cos(trajectory[j][0]) + L2*cos(trajectory[j][0] + trajectory[j][1]);
			ee_pos_1 = L1*sin(trajectory[j][0]) + L2*sin(trajectory[j][0] + trajectory[j][1]);
			trajfile2 << ee_pos_0 << "," << ee_pos_1 << "\n";
	 	}
	 	cout << "num_obs_calc: " << num_obs_calc << endl; 
	 	cout << "num_loops: " << num_loops << endl; 
	 	cout << "Done" << endl; 
  } else {
		cout << "Final position not reached.\n";
	}
}

int main() {
	srand (time(NULL));
	vector<float> qs;
	qs.push_back(0);
	qs.push_back(0);

	vector<float> qf;
	qf.push_back(PI/4);
	qf.push_back(0);

	vector<vector<float> > obstacles;
	for (int i = 0; i < 500; i++) {
		vector<float> currObs; 
		currObs.push_back(2); 
		currObs.push_back(2); 
		currObs.push_back(0.25);
		obstacles.push_back(currObs);  
	}

	// Initialize all entries in the memo to be invalid
	for (int i = 0; i < Q1RANGE + 1; i++) {
		for (int j = 0; j < Q2RANGE + 1; j++) {
			colValid[i][j] = false; 
			colMemo[i][j] = false;
		}
	}

	auto start = chrono::high_resolution_clock::now(); 
	rrt(qs, qf, obstacles);
	auto stop = chrono::high_resolution_clock::now(); 

	auto duration = chrono::duration_cast<chrono::microseconds>(stop - start); 
  
    cout << "Time taken by function: " << duration.count() << " microseconds" << endl; 
	return 0;
}