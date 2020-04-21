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

using namespace std;

// findNearestVert takes find the closest configuration that we've seen before to q_curr
// It returns entries with that nearest vert vec(0:1), and entry with index vec(2)

// fill this in later!

#define Q1RANGE 360
#define Q2RANGE 180
#define Q1MIN -180
#define Q2MIN 0 
#define PI 3.14159

float L1 = 1;
float L2 = 1;

bool colMemo[Q1RANGE + 1][Q2RANGE + 1]; 
bool colValid[Q1RANGE + 1][Q2RANGE + 1]; 

int numColsComp; 
int numColsCall; 

atomic_int numThreadComp; 
atomic_bool hasHit; 

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
							return;
					}
			} else {
					if (0 < bx && 0 > ax) {
							hasHit = true; 
							numThreadComp++; 
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
							return;
					}
			} else {
					if (0 < bx && 0 > ax) {
							hasHit = true; 
							numThreadComp++; 
							return;
				  }
			}
	}
	numThreadComp++; 
	return; 
}

bool inObs(vector<float> p_curr, vector<float> q_curr, vector<vector<float> > obstacles) {
	// Make sure that ee and joint don't hit!

	// Passing the joint vector, don't need to call this!
	// vector<float> q_curr(2);
	// vector<float> temp(2);
	// temp = inverseKin(p_curr);
	// copy(temp.begin(), temp.begin() + 2, q_curr.begin());
	// int q1_ind = round(q_curr[0]*180/PI) - Q1MIN;
	// int q2_ind = round(q_curr[1]*180/PI) - Q2MIN; 

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

	for (int i = 0; i < obstacles.size(); i++) {
		// spawn threads here!
		vector<float> currObs = obstacles[i];
		thread currThread(obsChecker, base_pos, joint_pos, ee_pos, currObs); 
		currThread.detach(); 
	}

	while (!hasHit && numThreadComp < obstacles.size()) {
		// poll here!
	}

	return hasHit;
}

// fill this in later also!
bool hasEdgeCollision(vector<float> p1, vector<float> p2, vector<float> q_curr, vector<vector<float> > obstacles) {

	// convert to degrees
	// vector<float> q_deg(2);
	// q_deg[0] = q_curr[0]*(180/PI);
	// q_deg[1] = q_curr[1]*(180/PI);

	// // look in the memo
	// int q1_ind = round(q_deg[0]) - Q1MIN; 
	// int q2_ind = round(q_deg[1]) - Q2MIN; 

	// // This is really not fun
	// if (isnan(q_deg[0]) || isnan(q_deg[1])) 
	// 	return true; 

	// cout << "q1 in degrees: " << round(q_deg[0]) << endl; 
	// cout << "q2 in degrees: " << round(q_deg[1]) << endl; 

	// cout << "q1_ind: " << q1_ind << endl; 
	// cout << "q2_ind: " << q2_ind << endl; 
	// numColsCall++; 

	// if (colValid[q1_ind][q2_ind]) {
	// 	return colMemo[q1_ind][q2_ind];
	// }

	// numColsComp++; 

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
	float dist = sqrt((q1[0]-q2[0])*(q1[0]-q2[0]) + (q1[1]-q2[1])*(q1[1]-q2[1]));
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
	int max_iter = 7500;
	float max_step = 0.05; // everything will be in radians!

	float q1_max = (Q1RANGE + Q1MIN)*(PI/180);
	float q1_min = Q1MIN*(PI/180);

	float q2_max = (Q2RANGE + Q2MIN)*(PI/180);
	float q2_min = Q2MIN*(PI/180);

	vector<float> q_rand(2);
	vector<float> p_rand(2);
  	vector<float> pf(2);
	pf[0] = L1*cos(qf[0]) + L2*cos(qf[0] + qf[1]);
	pf[1] = L1*sin(qf[0]) + L2*sin(qf[0] + qf[1]);

	vector<vector<float> > route;
	route.push_back(qs);

	vector<int> edges;
	edges.push_back(1);

	while (num_iter <= max_iter) {
		q_rand[0] = (q1_max - q1_min)*(rand() % 100)/100 + q1_min;
		q_rand[1] = (q2_max - q2_min)*(rand() % 100)/100 + q2_min;
		p_rand[0] = L1*cos(q_rand[0]) + L2*cos(q_rand[0] + q_rand[1]);
		p_rand[1] = L1*sin(q_rand[0]) + L2*sin(q_rand[0] + q_rand[1]);

		// cout << p_rand[0] << "\n";
		// cout << p_rand[1] << "\n";

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
			if (findNorm(p_near, pf) < max_step) {
				if (hasEdgeCollision(p_near, pf, qf, obs)) {
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
			float magnitude = findNorm(p_rand, p_near);
			if(magnitude > 0.001) {
				p_new[0] = ((p_rand[0] - p_near[0])/magnitude)*max_step + p_near[0];
				p_new[1] = ((p_rand[1] - p_near[1])/magnitude)*max_step + p_near[1];

				temp = inverseKin(p_new);
				copy(temp.begin(), temp.begin() + 2, q_new.begin());
				//cout << p_new[0] << "," << p_new[1];
				//cout << q_new[0] << "," << q_new[1];
				// Don't push_back if we hit in process
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
		//trajfile.open("traj_log_no_obs.csv");
		// x, y in a row. Lines denote separate points.

		int traj_len = trajectory.size();
		for (int j = traj_len-1; j > -1; j--) {
			trajfile << trajectory[j][0] << "," << trajectory[j][1] << "\n";
	 	}

		ofstream trajfile2;
		trajfile2.open("workspace_traj_log.csv");
		//trajfile2.open("workspace_traj_log_no_obs.csv");
		// x, y in a row. Lines denote separate points.
	  float ee_pos_0;
		float ee_pos_1;
		for (int j = traj_len-1; j > -1; j--) {
			ee_pos_0 = L1*cos(trajectory[j][0]) + L2*cos(trajectory[j][0] + trajectory[j][1]);
			ee_pos_1 = L1*sin(trajectory[j][0]) + L2*sin(trajectory[j][0] + trajectory[j][1]);
			trajfile2 << ee_pos_0 << "," << ee_pos_1 << "\n";
	 	}
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
	qf.push_back(1.57);
	qf.push_back(0);

	vector<vector<float> > obstacles;
	vector<float> obs1;
	obs1.push_back(1);
	obs1.push_back(1);
	obs1.push_back(0.25);
	obstacles.push_back(obs1);

	numColsCall = 0; 
	numColsComp = 0; 
	// Initialize all entries in the memo to be invalid
	for (int i = 0; i < Q1RANGE + 1; i++) {
		for (int j = 0; j < Q2RANGE + 1; j++) {
			colValid[i][j] = false; 
			colMemo[i][j] = false;
		}
	}

	rrt(qs, qf, obstacles);
	cout << "numColsCall: " << numColsCall << endl; 
	cout << "numColsComp: " << numColsComp << endl; 
	return 0;
}
