#include <vector>
#include <math.h>
#include <stdlib.h>
#include <float.h>
#include <iostream>
#include <fstream>
#include <cmath>

using namespace std; 

// findNearestVert takes find the closest configuration that we've seen before to q_curr
// It returns entries with that nearest vert vec(0:1), and entry with index vec(2)

// fill this in later!
L1 = 1; 
L2 = 1; 

bool inObs(vector<float> q_curr, vector<vector<float> > obstacles) {
	// Make sure that ee and joint don't hit!

	vector<float> ee_pos(2);
	ee_pos[0] = L1*cos(q_curr[0]) + L2*cos(q_curr[0] + q_curr[1]);
	ee_pos[0] = L1*sin(q_curr[0]) + L2*sin(q_curr[0] + q_curr[1]);

	vector<float> joint_pos(2);
	joint_pos[0] = L1*cos(q_curr[0]); 
	joint_pos[1] = L1*sin(q_curr[0]); 

	for (int i = 0; i < obstacles.size(); i++) {
		vector<float> obs_pos(2); 
		copy(obstacles[i].begin(), obstacles[i].begin() + 2, obs_pos.begin());
		if (findNorm(ee_pos, obs_pos) < obstacles[i][2]) {
			return true; 
		} else if (findNorm(joint_pos, obs_pos) < obstacles[i][2]) {
			return true;
		}
	}
	return false; 
}

// fill this in later also!
bool hasEdgeCollision(vector<float> q1, vector<float> q2, vector<vector<float> > obstacles) {
	int resolution = 25; 

	vector<float> p1(2);
	vector<float> p2(2);

	p1[0] = 

	float m = (q2[1] - q1[1])/(q2[0] - q1[0]);
	float xdiff = q2()
	return false;
}

float findNorm(vector<float> q1, vector<float> q2) {
	float dist = sqrt((q1[0]-q2[0])*(q1[0]-q2[0]) + (q1[1]-q2[1])*(q1[1]-q2[1]));
	return dist; 
}


vector<float> findNearestVert(vector<float> q_curr, vector<vector<float> > route) {
	int num_nodes = route.size(); 
	float min_dist = FLT_MAX; 

	vector<float> ans(3);

	for (int i = 0; i < num_nodes; i++) {
		float dist = findNorm(route[i], q_curr);
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
	int max_iter = 2500; 
	float max_step = 0.035; // everything will be in radians!

	float q1_max = 90*(3.14/180);
	float q1_min = 0*(3.14/180);

	float q2_max = 90*(3.14/180);
	float q2_min = 0*(3.14/180);

	vector<float> q_rand(2); 

	vector<vector<float> > route; 
	route.push_back(qs);

	vector<int> edges; 
	edges.push_back(1);

	while (num_iter <= max_iter) {
		q_rand[0] = (q1_max - q1_min)*(rand() % 100)/100 + q1_min; 
		q_rand[1] = (q2_max - q2_min)*(rand() % 100)/100 + q2_min; 

		
		if (!inObs(q_rand, obs)) {
			num_iter++; 
			vector<float> nearVertAndInd = findNearestVert(q_rand, route);
			vector<float> q_near(2); 

			// move the top angle part to q_near
			copy(nearVertAndInd.begin(), nearVertAndInd.begin() + 2, q_near.begin());
			int near_ind = (int) nearVertAndInd[2];

			// if close enough, then we're done! :) 
			if (findNorm(q_near, qf) < max_step) {
				route.push_back(qf);
				edges.push_back(near_ind);
				break;  
			}

			// compute this from q_near, q_rand, and max_step
			vector<float> q_new(2);
			float magnitude = findNorm(q_rand, q_near);

			q_new[0] = ((q_rand[0] - q_near[0])/magnitude)*max_step + q_near[0];
			q_new[1] = ((q_rand[1] - q_near[1])/magnitude)*max_step + q_near[1]; 

			// Don't push_back if we hit in process
			if (hasEdgeCollision(q_new, q_near, obs)) {
				num_iter--; 
				continue; 
			}
			route.push_back(q_new); 
			edges.push_back(near_ind);
		}

	}
	// Now that we've got the total graph, pick out a trajectory
	// num_iter total number of nodes

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
	for (int j = 0; j < traj_len; j++) {
		trajfile << trajectory[j][0] << "," << trajectory[j][1] << "\n"; 
 	}
}

int main() {
	vector<float> qs; 
	qs.push_back(0);
	qs.push_back(0);

	vector<float> qf; 
	qf.push_back(0.698);
	qf.push_back(0.698); 

	vector<vector<float> > obstacles; 

	rrt(qs, qf, obstacles); 
	return 0; 
}