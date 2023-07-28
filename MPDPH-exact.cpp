//Devaraja Vignesh Radha Krishnan
#include "gurobi_c++.h"
#include <cstdlib>
#include <cmath>
#include <sstream>
#include <fstream>
#include <iostream>
#include <time.h>
#include<string>
#include<vector>
#include<algorithm>

using namespace std;
using std::ios;
string itos(int i) { stringstream s; s << i; return s.str(); }
double viol;

void position_indexing(vector<int> path, vector<int> &position, vector<int> &index) {
	int pathlen = 0, i;
	for (i = 0; i < path.size(); i++) {
		position[path[i]] = pathlen;
		index[pathlen] = path[i];
		pathlen++;
	}
}

double obj_fn(vector<vector<double>> fs, vector<vector<double>> cost, vector<int> position, vector<int> orders_on_path, vector<int> path, int orders, int hc) {
	int i, j;
	double obfn = 0;
	for (i = 0; i < path.size() - 1; i++)
		for (j = 0; j < fs[path[i]].size(); j++)
			if (fs[path[i]][j] == path[i + 1]) {
				obfn += cost[path[i]][j];
				break;
			}

	for (i = 0; i < orders_on_path.size(); i++)
		for (j = 0; j < orders_on_path.size(); j++)
			if (i != j)
				if ((position[orders_on_path[i]] < position[orders_on_path[j]]) && (position[orders_on_path[j]] < position[orders + orders_on_path[i]]) && (position[orders + orders_on_path[i]] < position[orders + orders_on_path[j]]))
					obfn += hc;
	return obfn;
}

bool capacity_acceptable(double qv, vector<int> vpath, vector<double> qs) {
	int i, checker = 0;
	double weight_in_vehicle = 0;
	//Capacity feasibility
	for (i = 0; i < vpath.size(); i++) {
		weight_in_vehicle += qs[vpath[i]];
		//cout << "The node is " << vpath[i] << endl;
		//cout << "Weight in vehicle is " << weight_in_vehicle<<endl;
		if (weight_in_vehicle > qv)
			checker++;
	}

	if (checker == 0)
		return true;
	else
		return false;
}

bool time_acceptable(vector<int> vpath, double vt, vector<vector<double>> fs, vector<vector<double>> tij, vector<double> ai, vector<double> bi, vector<double> qs) {
	int i, j, k, checker = 0;
	double time_on_vehicle = 0;
	//Time window feasibility
	for (i = 0; i < vpath.size() - 1; i++) {
		for (j = 0; j < fs[vpath[i]].size(); j++)
			if (fs[vpath[i]][j] == vpath[i + 1]) {
				time_on_vehicle += tij[vpath[i]][j];
				//cout << "The arc is "<< vpath[i] << " " << vpath[i + 1] << endl;
				//cout << "Time " << time_on_vehicle << endl;
				//system("pause");
			}
		if ((time_on_vehicle > vt) || (time_on_vehicle > bi[vpath[i + 1]])) {
			checker++;
			//cout << "Time on vehicle " << time_on_vehicle << endl;
			//cout << "Time on path " << bi[vpath[i+1]] << endl;
		}
	}
	//cout << "checker "<<checker << endl;
	if (checker == 0)
		return true;
	else
		return false;
}

void juggle(vector<vector<double>> fs, vector<vector<double>> cij, vector<vector<double>> tij, vector<double> qs, vector<double> ai, vector<double> bi, vector<int>& org_path, vector<vector<int>>& violations, bool& new_path, int orders, int nodes, int hc, double qv, int new_order) {
	vector<int> pathpos(nodes), pathindex(nodes);
	double ob;
	int i, j, k;
	vector<int> orders_on_path;

	vector<int> path(org_path.size());
	path = org_path;
	path.push_back(new_order);
	path.push_back(new_order + orders);

	//Find out what customer requests are on the path
	for (i = 0; i < path.size(); i++)
		if (path[i] <= orders)
			orders_on_path.push_back(path[i]);

	/*cout << "The path after insertion is " << endl;
	for (i = 0; i < path.size(); i++)
		cout << path[i] << " ";
	cout << endl;*/

	fill(pathpos.begin(), pathpos.end(), 300);
	fill(pathindex.begin(), pathindex.end(), 300);
	position_indexing(path, pathpos, pathindex);
	//ob = obj_fn(fs, cij, pathpos, orders_on_path, path, orders, hc);
	ob = 100000;
	//cout << "Objective function is " << ob << endl;
	//system("pause");

	for (i = 0; i < orders_on_path.size(); i++) {
		int t1, t2, t3;
		vector<int> best_i_path(path.size());
		best_i_path = path;
		int itr_size = best_i_path.size();
		//Remove the order nodes from the path
		best_i_path.erase(remove(best_i_path.begin(), best_i_path.end(), orders_on_path[i]), best_i_path.end());
		best_i_path.erase(remove(best_i_path.begin(), best_i_path.end(), orders_on_path[i] + orders), best_i_path.end());

		fill(pathpos.begin(), pathpos.end(), 300);
		fill(pathindex.begin(), pathindex.end(), 300);
		position_indexing(best_i_path, pathpos, pathindex);
		//ob = obj_fn(fs, cij, pathpos, orders_on_path,path, orders, hc);

		for (t1 = 0; t1 < itr_size - 1; t1++)
			for (t2 = t1 + 1; t2 < itr_size; t2++) {
				best_i_path.insert(best_i_path.begin() + t1, orders_on_path[i]);
				best_i_path.insert(best_i_path.begin() + t2, orders_on_path[i] + orders);

				//cout << "Active order is: " << orders_on_path[i] << endl;
				//cout << "t1: " <<t1<< endl;
				//cout << "t2: " <<t2<< endl;

				fill(pathpos.begin(), pathpos.end(), 300);
				fill(pathindex.begin(), pathindex.end(), 300);
				position_indexing(best_i_path, pathpos, pathindex);

				double new_ob = obj_fn(fs, cij, pathpos, orders_on_path, best_i_path, orders, hc);

				/*cout << "The new path within loop is " << endl;
				for (j = 0; j < best_i_path.size(); j++)
					cout << best_i_path[j] << " ";
				cout << endl;
				cout << "Old objective function is " << ob << endl;
				cout << "Corresponding objective function is " << new_ob << endl;
				cout << "Is capacity acceptable? " << capacity_acceptable(qv, best_i_path, qs) << endl;
				cout << "Is time acceptable? " << time_acceptable(best_i_path, bi[nodes - 1], fs, tij, ai, bi, qs) << endl;
				system("pause");*/

				if ((capacity_acceptable(qv, best_i_path, qs)) && (new_ob < ob) && (time_acceptable(best_i_path, bi[nodes - 1], fs, tij, ai, bi, qs))) {
					org_path = best_i_path;
					path = best_i_path;
					ob = new_ob;
					new_path = true;
				}
				best_i_path.erase(remove(best_i_path.begin(), best_i_path.end(), orders_on_path[i]), best_i_path.end());
				best_i_path.erase(remove(best_i_path.begin(), best_i_path.end(), orders_on_path[i] + orders), best_i_path.end());
			}
	}

	fill(pathpos.begin(), pathpos.end(), 300);
	fill(pathindex.begin(), pathindex.end(), 300);
	position_indexing(org_path, pathpos, pathindex);
	violations.clear();

	k = 0;
	for (i = 0; i < orders_on_path.size(); i++)
		for (j = 0; j < orders_on_path.size(); j++)
			if (i != j)
				if ((pathpos[orders_on_path[i]] < pathpos[orders_on_path[j]]) && (pathpos[orders_on_path[j]] < pathpos[orders + orders_on_path[i]]) && (pathpos[orders + orders_on_path[i]] < pathpos[orders + orders_on_path[j]])) {
					violations.push_back(vector<int>());
					violations[k].push_back(orders_on_path[i]);
					violations[k].push_back(orders_on_path[j]);
					k++;
				}

}

void warm_start(vector<vector<int>> &pth, vector<vector<double>> fs, vector<vector<double>> dij, vector<vector<double>> tij, vector<vector<double>> &sav, double n, double ns, double qv, double nv, double hc, vector<double> qs, vector<double> ai, vector<double> bi)
{
	vector<vector<int>> violations;
	string dum;
	int viol = 0;
	double i, j, k, l;
	int x, y, z, m = 0;
	vector<bool> seen(ns + 1, false);
	
	//Sorting savings list
	for (i = 0; i < sav.size() - 1; i++)
		for (j = i + 1; j < sav.size(); j++)
			if (sav[j][0] > sav[i][0]) {
				double temp1, temp2, temp3;
				temp1 = sav[j][0];
				temp2 = sav[j][1];
				temp3 = sav[j][2];
				sav[j][0] = sav[i][0];
				sav[j][1] = sav[i][1];
				sav[j][2] = sav[i][2];
				sav[i][0] = temp1;
				sav[i][1] = temp2;
				sav[i][2] = temp3;
			}

	//Sorting and grouping best candidates for pickup by savings
	vector<vector<int>> order_candidates(ns + 1);
	vector<vector<double>> order_savings(ns + 1);
	for (i = 0; i < sav.size(); i++) {
		if ((sav[i][1] >= 1) && (sav[i][1] <= ns) && (sav[i][2] >= 1) && (sav[i][2] <= ns))//If head of the arc is a pickup & tail is also a pickup
			if (find(order_candidates[sav[i][1]].begin(), order_candidates[sav[i][1]].end(), sav[i][2]) == order_candidates[sav[i][1]].end()) {
				order_candidates[sav[i][1]].push_back(sav[i][2]);
				order_savings[sav[i][1]].push_back(sav[i][0]);
			}
		if ((sav[i][1] >= ns + 1) && (sav[i][1] <= 2 * ns) && (sav[i][2] >= ns + 1) && (sav[i][2] <= 2 * ns))//If head of the arc is a delivery & tail is also a delivery
			if (find(order_candidates[sav[i][1] - ns].begin(), order_candidates[sav[i][1] - ns].end(), sav[i][2] - ns) == order_candidates[sav[i][1] - ns].end()) {
				order_candidates[sav[i][1] - ns].push_back(sav[i][2] - ns);
				order_savings[sav[i][1] - ns].push_back(sav[i][0]);
			}
		if ((sav[i][1] >= 1) && (sav[i][1] <= ns) && (sav[i][2] >= ns + 1) && (sav[i][2] <= 2 * ns))//If head of the arc is a pickup & tail is a delivery
			if (find(order_candidates[sav[i][1]].begin(), order_candidates[sav[i][1]].end(), sav[i][2] - ns) == order_candidates[sav[i][1]].end()) {
				order_candidates[sav[i][1]].push_back(sav[i][2] - ns);
				order_savings[sav[i][1]].push_back(sav[i][0]);
			}
		if ((sav[i][1] >= ns + 1) && (sav[i][1] <= 2 * ns) && (sav[i][2] >= 1) && (sav[i][2] <= ns))//If head of the arc is a delivery & tail is a pickup
			if (find(order_candidates[sav[i][1] - ns].begin(), order_candidates[sav[i][1] - ns].end(), sav[i][2]) == order_candidates[sav[i][1] - ns].end()) {
				order_candidates[sav[i][1] - ns].push_back(sav[i][2]);
				order_savings[sav[i][1] - ns].push_back(sav[i][0]);
			}
	}
	for (i = 0; i < order_candidates.size(); i++)
		order_candidates[i].erase(remove(order_candidates[i].begin(), order_candidates[i].end(), i), order_candidates[i].end());

	/*for (i = 0; i < order_candidates.size(); i++) {
		cout << "Out of " << i << ": ";
		for (j = 0; j < order_candidates[i].size(); j++)
			cout << order_candidates[i][j] << " ";
		cout << endl;
	}

	system("pause");*/

	//initiate a path
	pth[0].push_back(1);
	pth[0].push_back(ns + 1);
	seen[1] = true;
	double flag = 20000;


	/*for (j = 0; j < pth[0].size(); j++)
		cout << pth[0][j] << " ";
	cout << endl;

	system("pause");*/

	for (k = 0; k < nv; k++) {
		vector<int> orders_on_path;

		//See what shipments are on the path
	rerun_orders:for (x = 0; x < pth[k].size(); x++)
		if (pth[k][x] <= ns)
			orders_on_path.push_back(pth[k][x]);

	orders_on_path.erase(unique(orders_on_path.begin(), orders_on_path.end()), orders_on_path.end());

	/*cout << "Orders on path " << k << " are:";
	for (i = 0; i < orders_on_path.size(); i++)
			cout << orders_on_path[i] << " ";
	cout << endl;
	cout << endl<<"The path is"<<endl;
	for (j = 0; j < pth[k].size(); j++)
		cout << pth[k][j] << " ";
	cout << endl;
	system("pause");*/

	//Choose an unselected shipment to be appended to the path based on highest savings
	for (x = 0; x < orders_on_path.size(); x++)
		for (y = 0; y < order_candidates[orders_on_path[x]].size(); y++)
			if (seen[order_candidates[orders_on_path[x]][y]] == false) {
				bool chk = false;
				//cout << "Here: "<<order_candidates[orders_on_path[x]][y] << endl;
				//system("pause");
				juggle(fs, dij, tij, qs, ai, bi, pth[k], violations, chk, ns, n, hc, qv, order_candidates[orders_on_path[x]][y]);
				if (chk == true) {
					seen[order_candidates[orders_on_path[x]][y]] = true;
					//cout << "Right Here: " << order_candidates[orders_on_path[x]][y] << endl;
					//system("pause");
					goto rerun_orders;
				}
			}
	//int shipment_check = 0;

	//Initiate next vehicle path
	for (x = 1; x <= ns; x++)
		if ((seen[x] == false) && (k < nv - 1)) {
			pth[k + 1].push_back(x);
			pth[k + 1].push_back(ns + x);
			seen[x] = true;
			break;
		}
	}

	/*for (k = 0; k < nv; k++) {
		cout << endl << "The path for " << k << " is" << endl;
		for (j = 0; j < pth[k].size(); j++)
			cout << pth[k][j] << " ";
		cout << endl;
		system("pause");
	}*/
}

int main(int argc, char* argv[])
{
	vector<string> filename;
	ifstream fileforin;
	string fn;
	int itr;
	fileforin.open("Filelist.txt");/*identifying the external file to be read*/
	while (!fileforin.eof()) {
		fileforin >> fn;
		filename.push_back(fn);
	}
	fileforin.close();
	ofstream outfile;
	outfile.open("Resultsexact1.csv");
	outfile << "Filename,htime, hobjective,gap,opt,objective,model time,solving time,nodes,Trucks,cuts,actcost,modcost\n";

	for (itr = 0; itr < filename.size(); itr++) {
		cout << "Starting work on " << filename[itr]<<endl;
		double n, h, ns, qv, nv, dist, tim, tim1, tim2, nos,hc,act_cost=0;
		vector<vector<double>> fs, dij, tij, sav;
		vector<double> os, ts, qs, qlb, qub, ai, bi;
		clock_t mst, mfin, optst, optfin,hst,hfin;
		GRBEnv *env = NULL;
		ifstream infile;
		string dum;
		viol = 0;
		double i, j, k, l,m = 0;
		int x, y, z;
		mst = clock();
		infile.open(filename[itr] + ".txt");
		cout << filename[itr] << endl;
		while (!infile.eof()) {
			infile >> dum;
			if (dum == "d") {
				infile >> i >> j >> dist >> tim;
				fs[i].push_back(j);
				dij[i].push_back(dist);
				tij[i].push_back(tim);
				sav.push_back(vector<double>());
				sav[m].push_back(-dist);
				sav[m].push_back(i);
				sav[m].push_back(j);
				sav[m].push_back(tim);
				if ((j - i == ns) && (i > 0) && (j <= 2 * ns))
					act_cost += (dist*1.38) + 272;
				m++;
				//cout << "d";
			}
			else if (dum == "t") {
				infile >> i >> tim1 >> tim2;
				ai[i] = tim1;
				bi[i] = tim2;
				//cout << "t";
			}
			else if (dum == "s") {
				infile >> i >> j >> dist;
				os.push_back(i);
				ts.push_back(j);
				qs[i] = dist;
				qs[j] = -dist;
				//cout << "s";
			}
			else if (dum == "nv") {
				infile >> nv;
				//cout << "nv";
			}
			else if (dum == "ns") {
				infile >> ns;
				//cout << "ns";
			}
			else if (dum == "#")
				getline(infile, dum);
			else if (dum == "h")
				infile >> hc;
			else if (dum == "n") {
				infile >> n;
				for (i = 0; i < n; i++) {
					fs.push_back(vector<double>());
					dij.push_back(vector<double>());
					tij.push_back(vector<double>());
					qs.push_back(0);
					qlb.push_back(0);
					qub.push_back(0);
					ai.push_back(0);
					bi.push_back(0);
				}
				//system("pause");
				//cout << "n";
			}
			else if (dum == "q")
				infile >> qv;
			else if (dum == "nos")
				infile >> nos;
		}
		try
		{
			//cout << "Done reading, here goes!" << endl;
			env = new GRBEnv();
			GRBModel CGSP = GRBModel(*env);
			vector<vector<vector<GRBVar>>> xijk, zijk, bijk,hijk;
			vector<vector<vector<vector<GRBVar>>>> f1, f2, f3;
			vector<vector<GRBVar>> qik, bik;
			vector<vector<GRBLinExpr>> vflowout(fs.size(), vector<GRBLinExpr>(nv)), vflowin(fs.size(), vector<GRBLinExpr>(nv));
			vector<GRBLinExpr> flowout(n), flowin(n), sflowout(nv);
			vector<GRBConstr> CGSP_const, CGSP_lin_const;

			// Must set LazyConstraints parameter when using lazy constraints
			CGSP.getEnv().set(GRB_IntParam_LazyConstraints, 1);
			CGSP.getEnv().set(GRB_DoubleParam_TimeLimit, 7200); //Two hour time limit
			CGSP.getEnv().set(GRB_DoubleParam_MIPGap, 1e-4);
			CGSP.getEnv().set(GRB_IntParam_OutputFlag, 0);
			CGSP.update();

			for (i = 0; i < fs.size(); i++) {
				qlb[i] = max(0.0, qs[i]);
				qub[i] = min(qv, qv + qs[i]);
				vector<GRBVar> jpush, bpush;
				for (k = 0; k < nv; k++) {
					jpush.push_back(CGSP.addVar(qlb[i], qub[i], 0, GRB_CONTINUOUS, "q_" + itos(i) + "_" + itos(k)));
					bpush.push_back(CGSP.addVar(ai[i], bi[i], 0, GRB_CONTINUOUS, "b_" + itos(i) + "_" + itos(k)));
				}
				qik.push_back(jpush);
				bik.push_back(bpush);
			}

			for (i = 0; i < fs.size(); i++) {
				vector<vector<GRBVar>> ipush, zipush, bipush;
				vector<vector<vector<GRBVar>>> f1ipush, f2ipush, f3ipush;
				for (j = 0; j < fs[i].size(); j++) {
					vector<GRBVar> jpush, zjpush, bjpush;
					vector<vector<GRBVar>> f1jpush, f2jpush, f3jpush;
					for (k = 0; k < nv; k++) {
						vector<GRBVar> f1kpush, f2kpush, f3kpush;
						jpush.push_back(CGSP.addVar(0.0, 1.0, dij[i][j], GRB_BINARY, "x_" + itos(i) + "_" + itos(fs[i][j]) + "_" + itos(k)));
						zjpush.push_back(CGSP.addVar(min(0.0, qlb[i]), qub[i], 0, GRB_CONTINUOUS, "z_" + itos(i) + "_" + itos(fs[i][j]) + "_" + itos(k)));
						bjpush.push_back(CGSP.addVar(min(0.0, ai[i]), bi[i], 0, GRB_CONTINUOUS, "b_" + itos(i) + "_" + itos(fs[i][j]) + "_" + itos(k)));
						for (l = 0; l < ns; l++) {
							f1kpush.push_back(CGSP.addVar(0.0, 1.0, 0, GRB_BINARY, "f1_" + itos(i) + "_" + itos(fs[i][j]) + "_" + itos(k) + "_" + itos(l)));
							f2kpush.push_back(CGSP.addVar(0.0, 1.0, 0, GRB_BINARY, "f2_" + itos(i) + "_" + itos(fs[i][j]) + "_" + itos(k) + "_" + itos(l)));
							f3kpush.push_back(CGSP.addVar(0.0, 1.0, 0, GRB_BINARY, "f3_" + itos(i) + "_" + itos(fs[i][j]) + "_" + itos(k) + "_" + itos(l)));
						}
						f1jpush.push_back(f1kpush);
						f2jpush.push_back(f2kpush);
						f3jpush.push_back(f3kpush);
					}
					ipush.push_back(jpush);
					zipush.push_back(zjpush);
					bipush.push_back(bjpush);
					f1ipush.push_back(f1jpush);
					f2ipush.push_back(f2jpush);
					f3ipush.push_back(f3jpush);
				}
				xijk.push_back(ipush);
				zijk.push_back(zipush);
				bijk.push_back(bipush);
				f1.push_back(f1ipush);
				f2.push_back(f2ipush);
				f3.push_back(f3ipush);
			}
			//hc = 2000;
			for (i = 0; i < ns; i++) {
				vector<vector<GRBVar>> hipush;
				for (j = 0; j < ns; j++) {
					vector<GRBVar> hjpush;
					for (k = 0; k < nv; k++)
						hjpush.push_back(CGSP.addVar(0.0, 1.0, hc, GRB_BINARY, "h_" + itos(i) + "_" + itos(j) + "_" + itos(k)));
					hipush.push_back(hjpush);
				}
				hijk.push_back(hipush);
			}
			 
			CGSP.update();

			vector<vector<GRBLinExpr>> f1flowout(fs.size(), vector<GRBLinExpr>(ns)), f1flowin(fs.size(), vector<GRBLinExpr>(ns)), f2flowout(fs.size(), vector<GRBLinExpr>(ns)), f2flowin(fs.size(), vector<GRBLinExpr>(ns)), f3flowout(fs.size(), vector<GRBLinExpr>(ns)), f3flowin(fs.size(), vector<GRBLinExpr>(ns));
			vector<vector<vector<GRBLinExpr>>> f2flowin9(fs.size(), vector<vector<GRBLinExpr>>(n, vector<GRBLinExpr>(nv))), f2flowout9(fs.size(), vector<vector<GRBLinExpr>>(n, vector<GRBLinExpr>(nv)));
			
			for (i = 0; i < fs.size(); i++)
				for (j = 0; j < fs[i].size(); j++) {
					for (k = 0; k < nv; k++) {
						vflowout[i][k] += xijk[i][j][k];
						vflowin[fs[i][j]][k] += xijk[i][j][k];
						flowout[i] += xijk[i][j][k];
						flowin[fs[i][j]] += xijk[i][j][k];
						for (l = 0; l < ns; l++) {
							f1flowout[i][l] += f1[i][j][k][l];
							f1flowin[fs[i][j]][l] += f1[i][j][k][l];
							f2flowout[i][l] += f2[i][j][k][l];
							f2flowin[fs[i][j]][l] += f2[i][j][k][l];
							f3flowout[i][l] += f3[i][j][k][l];
							f3flowin[fs[i][j]][l] += f3[i][j][k][l];
							if ((i != 0) && (fs[i][j] != 0) && (i != n - 1) && (fs[i][j] != n - 1)) {
								f2flowout9[i][l][k] += f2[i][j][k][l];
								f2flowin9[fs[i][j]][l][k] += f2[i][j][k][l];
							}
						}

						CGSP_lin_const.push_back(CGSP.addConstr(zijk[i][j][k], GRB_GREATER_EQUAL, qlb[i] * xijk[i][j][k], "CGSP_lin_const2z_" + itos(i) + "_" + itos(fs[i][j]) + "_" + itos(k)));
						CGSP_lin_const.push_back(CGSP.addConstr(bijk[i][j][k], GRB_GREATER_EQUAL, ai[i] * xijk[i][j][k], "CGSP_lin_const2b_" + itos(i) + "_" + itos(fs[i][j]) + "_" + itos(k)));
						CGSP_lin_const.push_back(CGSP.addConstr(zijk[i][j][k], GRB_LESS_EQUAL, qub[i] * xijk[i][j][k], "CGSP_lin_const3z_" + itos(i) + "_" + itos(fs[i][j]) + "_" + itos(k)));
						CGSP_lin_const.push_back(CGSP.addConstr(bijk[i][j][k], GRB_LESS_EQUAL, bi[i] * xijk[i][j][k], "CGSP_lin_const3b_" + itos(i) + "_" + itos(fs[i][j]) + "_" + itos(k)));
						CGSP_lin_const.push_back(CGSP.addConstr(zijk[i][j][k], GRB_GREATER_EQUAL, qik[i][k] - ((1 - xijk[i][j][k])*qub[i]), "CGSP_lin_const4z_" + itos(i) + "_" + itos(fs[i][j]) + "_" + itos(k)));
						CGSP_lin_const.push_back(CGSP.addConstr(bijk[i][j][k], GRB_GREATER_EQUAL, bik[i][k] - ((1 - xijk[i][j][k])*bi[i]), "CGSP_lin_const4b_" + itos(i) + "_" + itos(fs[i][j]) + "_" + itos(k)));
						CGSP_lin_const.push_back(CGSP.addConstr(zijk[i][j][k], GRB_LESS_EQUAL, qik[i][k] - ((1 - xijk[i][j][k])*qlb[i]), "CGSP_lin_const5z_" + itos(i) + "_" + itos(fs[i][j]) + "_" + itos(k)));
						CGSP_lin_const.push_back(CGSP.addConstr(bijk[i][j][k], GRB_LESS_EQUAL, bik[i][k] - ((1 - xijk[i][j][k])*ai[i]), "CGSP_lin_const5b_" + itos(i) + "_" + itos(fs[i][j]) + "_" + itos(k)));
						CGSP_lin_const.push_back(CGSP.addConstr(zijk[i][j][k], GRB_LESS_EQUAL, qik[i][k] + ((1 - xijk[i][j][k])*qub[i]), "CGSP_lin_const6z_" + itos(i) + "_" + itos(fs[i][j]) + "_" + itos(k)));
						CGSP_lin_const.push_back(CGSP.addConstr(bijk[i][j][k], GRB_LESS_EQUAL, bik[i][k] + ((1 - xijk[i][j][k])*bi[i]), "CGSP_lin_const6b_" + itos(i) + "_" + itos(fs[i][j]) + "_" + itos(k)));
					}
				}
			CGSP.update();
			//cout << "Done flow counts and linearization!" << endl;

			for (i = 0; i < fs.size(); i++)
				for (k = 0; k < ns; k++) {
					if (i == 0)
						CGSP.addConstr(f1flowout[i][k] - f1flowin[i][k], GRB_EQUAL, 1, "Const4_" + itos(i) + "_" + itos(k));
					else if (i == k+1)
						CGSP.addConstr(f1flowout[i][k] - f1flowin[i][k], GRB_EQUAL, -1, "Const4_" + itos(i) + "_" + itos(k));
					else
						CGSP.addConstr(f1flowout[i][k] - f1flowin[i][k], GRB_EQUAL, 0, "Const4_" + itos(i) + "_" + itos(k));

					if ((i != 0)&&(i != n-1)) {
					if (i == k+1)
						CGSP.addConstr(f2flowout[i][k] - f2flowin[i][k], GRB_EQUAL, 1, "Const5_" + itos(i) + "_" + itos(k));
					else if (i == ns + k+1)
						CGSP.addConstr(f2flowout[i][k] - f2flowin[i][k], GRB_EQUAL, -1, "Const5_" + itos(i) + "_" + itos(k));
					else
						CGSP.addConstr(f2flowout[i][k] - f2flowin[i][k], GRB_EQUAL, 0, "Const5_" + itos(i) + "_" + itos(k));
					}

					if (i == ns + k+1)
						CGSP.addConstr(f3flowout[i][k] - f3flowin[i][k], GRB_EQUAL, 1, "Const6_" + itos(i) + "_" + itos(k));
					else if (i == n - 1)
						CGSP.addConstr(f3flowout[i][k] - f3flowin[i][k], GRB_EQUAL, -1, "Const6_" + itos(i) + "_" + itos(k));
					else
						CGSP.addConstr(f3flowout[i][k] - f3flowin[i][k], GRB_EQUAL, 0, "Const6_" + itos(i) + "_" + itos(k));
				}
			//cout << "Fin1" << endl;
			for (i = 0; i < fs.size(); i++)
				for (j = 0; j < fs[i].size(); j++)
					for (k = 0; k < nv; k++) {
						GRBLinExpr sumexpr1, sumexpr2, sumexpr3;
						for (l = 0; l < ns; l++) {
							sumexpr1 += f1[i][j][k][l];
							sumexpr2 += f2[i][j][k][l];
							sumexpr3 += f3[i][j][k][l];
						}
						if ((i == 0) || (fs[i][j] == 0) || (i == n - 1) || (fs[i][j] == n - 1))
							CGSP.addConstr(sumexpr1 + sumexpr3, GRB_LESS_EQUAL, ns*xijk[i][j][k], "Const7_" + itos(i) + "_" + itos(fs[i][j]) + "_" + itos(k));
						else
							CGSP.addConstr(sumexpr1 + sumexpr2 + sumexpr3, GRB_LESS_EQUAL, ns*xijk[i][j][k], "Const8_" + itos(i) + "_" + itos(fs[i][j]) + "_" + itos(k));
					}
			//cout << "Fin2" << endl;
			
			for (i = 0; i < ns; i++)
				for (j = 0; j < ns; j++)
					for (k = 0; k < nv; k++)
					if (i != j)
						CGSP.addConstr(f2flowin9[j+1][i][k] - f2flowout9[j + ns+1][i][k], GRB_LESS_EQUAL, hijk[i][j][k], "Const9_" + itos(i) + "_" + itos(j) + "_" + itos(k));

			CGSP.update();
			//cout << "Fin3" << endl;
			//system("pause");
			for (k = 0; k < nv; k++)
				for (i = 0; i < fs[0].size(); i++)
					sflowout[k] += xijk[0][i][k];
			CGSP.update();

			for (k = 0; k < nv; k++)
				CGSP_const.push_back(CGSP.addConstr(sflowout[k], GRB_EQUAL, 1, "CGSP_const4_" + itos(k)));
			for (i = 0; i < os.size(); i++) {
				CGSP_const.push_back(CGSP.addConstr(flowout[os[i]], GRB_EQUAL, 1, "CGSP_const2_" + itos(os[i])));
				for (k = 0; k < nv; k++) {
					CGSP_const.push_back(CGSP.addConstr(vflowout[os[i]][k] - vflowout[ts[i]][k], GRB_EQUAL, 0, "CGSP_const3_" + itos(i) + "_" + itos(k)));
					CGSP_const.push_back(CGSP.addConstr(vflowout[os[i]][k] - vflowin[os[i]][k], GRB_EQUAL, 0, "CGSP_const5_" + itos(os[i]) + "_" + itos(k)));
					CGSP_const.push_back(CGSP.addConstr(vflowout[ts[i]][k] - vflowin[ts[i]][k], GRB_EQUAL, 0, "CGSP_const5_" + itos(ts[i]) + "_" + itos(k)));
					//cout << os[i] << " " << ts[i] << " "<<k<<" "<<tij[os[i]][ts[i]]<<endl;
					for (j = 0; j < fs[os[i]].size(); j++)
						if (fs[os[i]][j] - os[i] == ns)
							CGSP_const.push_back(CGSP.addConstr(bik[os[i]][k] + tij[os[i]][j], GRB_LESS_EQUAL, bik[ts[i]][k], "CGSP_const9_" + itos(os[i]) + "_" + itos(ts[i]) + "_" + itos(k)));
				}
			}
			//cout << "Check1" << endl;
			for (i = 0; i < fs.size(); i++)
				for (j = 0; j < fs[i].size(); j++)
					for (k = 0; k < nv; k++) {
						CGSP_const.push_back(CGSP.addConstr(qik[fs[i][j]][k], GRB_GREATER_EQUAL, zijk[i][j][k] + (qs[fs[i][j]] * xijk[i][j][k]), "CGSP_const7z_" + itos(i) + "_" + itos(fs[i][j]) + "_" + itos(k)));
						CGSP_const.push_back(CGSP.addConstr(bik[fs[i][j]][k], GRB_GREATER_EQUAL, bijk[i][j][k] + (tij[i][j] * xijk[i][j][k]), "CGSP_const7b_" + itos(i) + "_" + itos(fs[i][j]) + "_" + itos(k)));
					}

			for (i = 0; i < os.size(); i++) {
				GRBLinExpr symt;
				for (k = 0; k <= i; k++)
					symt += vflowin[os[i]][k];
				CGSP_const.push_back(CGSP.addConstr(symt, GRB_EQUAL, 1, "symmetry_const_" + itos(os[i])));
			}

			for (k = 0; k < nv; k++)
				for (i = 1; i <= ns; i++)
					for (j = 1; j <= ns; j++)
						if (i != j) {
							GRBLinExpr c1, c2, c3;
							for (x = 0; x < fs[i].size(); x++)
								if (fs[i][x] == j)
									c1 = xijk[i][x][k];
							//cout << "Part 1"<<endl;
							for (y = 0; y < fs[j].size(); y++)
								if (fs[j][y] == ns + i)
									c2 = xijk[j][y][k];
							//cout << "Part 2" << endl;
							for (z = 0; z < fs[ns + i].size(); z++)
								if (fs[ns + i][z] == ns + j)
									c3 = xijk[ns + i][z][k];
							//cout << "Part 3" << endl;
							CGSP.addConstr(hijk[i - 1][j - 1][k] >= c1 + c2 - 1);
							CGSP.addConstr(hijk[i - 1][j - 1][k] >= c2 + c3 - 1);
							CGSP.addConstr(hijk[i - 1][j - 1][k] >= c1 + c3 - 1);
						}

			//Warm start
			vector<vector<int>> pth(nv);

			double hobj=0;

			hst = clock();
			warm_start(pth, fs, dij, tij, sav, n, ns, qv, nv, hc, qs, ai, bi);
			hfin = clock() - hst;
			//double hobj= obj_fn(fs, cij, pathpos, orders_on_path, best_i_path, orders, hc);

			for (k = 0; k < nv; k++) {
				pth[k].insert(pth[k].begin(), 0);
				pth[k].push_back(n - 1);
			}

			for (k = 0; k < nv; k++) {
				vector<int> pathpos(n, -1), pathindex(n, -1);
				position_indexing(pth[k], pathpos, pathindex);

				//Loading h values
				for (i = 1; i <= ns; i++)
					for (j = 1; j <= ns; j++) {
						if ((i != j) && (pathpos[i] < pathpos[j]) && (pathpos[ns + i] > pathpos[j]) && (pathpos[ns + j] > pathpos[ns + i])) {
							hijk[i - 1][j - 1][k].set(GRB_DoubleAttr_Start, 1);
							hobj += hc;
						}
						else
							hijk[i - 1][j - 1][k].set(GRB_DoubleAttr_Start, 0);
					}

				//Loading x values
				for (i = 0; i < fs.size(); i++)
					for (j = 0; j < fs[i].size(); j++)
						if (pathpos[i] == pathpos[fs[i][j]] - 1) {
							xijk[i][j][k].set(GRB_DoubleAttr_Start, 1);
							hobj += dij[i][j];
						}
						else
							xijk[i][j][k].set(GRB_DoubleAttr_Start, 0);
			}

			//cout << "Check2" << endl;
			CGSP.update();
			cout << "Going to optimize!" << endl;
			CGSP.write("jcheck.lp");

			// Set callback function
			optst = clock();
			//cycleelim cb = cycleelim(xijk,hijk,fs,nv,ns,n);
			//CGSP.setCallback(&cb);
			//cycleelim cb = cycleelim(xij, yij, cij, v, p, n, fs, pathpos, viol_matrix);
			//model.setCallback(&cb);

			CGSP.optimize();
			optfin = clock() - optst;
			//system("pause");

			mfin = clock() - mst;
			double mtime = mfin / (double)CLOCKS_PER_SEC;
			double optime = optfin / (double)CLOCKS_PER_SEC;
			double htime = hfin / (double)CLOCKS_PER_SEC;
			double nodecnt = CGSP.get(GRB_DoubleAttr_NodeCount);
			double igap = CGSP.get(GRB_DoubleAttr_MIPGap);
			double nt = 0;
			int pos=0;

			for (j = 0; j < fs[0].size(); j++)
				if (fs[0][j] == n - 1) {
					pos = j;
					break;
				}
			for (k = 0; k < nv; k++)
				if (xijk[0][pos][k].get(GRB_DoubleAttr_X) < 0.3)
					nt++;

			double o = CGSP.get(GRB_DoubleAttr_ObjVal);
			double modcost=(o*1.38)+(nt*272);
			/*ofstream checkfile;
			checkfile.open(filename[itr]+".csv");
			checkfile << o << ",\n";
			for (i = 0; i < fs.size(); i++)
				for (j = 0; j < fs[i].size(); j++)
					for (k = 0; k < nv; k++)
						if (xijk[i][j][k].get(GRB_DoubleAttr_X) > 0.3)
							checkfile << xijk[i][j][k].get(GRB_StringAttr_VarName)<<","<< xijk[i][j][k].get(GRB_DoubleAttr_X) << ",\n";
			cout << "fvalues" << endl;
			for (i = 0; i < fs.size(); i++)
				for (j = 0; j < fs[i].size(); j++)
					for (k = 0; k < nv; k++)
						for (l = 0; l < ns; l++)
						if (f1[i][j][l][k].get(GRB_DoubleAttr_X) > 0.3)
							checkfile << f1[i][j][l][k].get(GRB_StringAttr_VarName) << "," << f1[i][j][l][k].get(GRB_DoubleAttr_X) << ",\n";

			for (i = 0; i < fs.size(); i++)
				for (j = 0; j < fs[i].size(); j++)
					for (k = 0; k < nv; k++)
						for (l = 0; l < ns; l++)
							if (f2[i][j][l][k].get(GRB_DoubleAttr_X) > 0.3)
								checkfile << f2[i][j][l][k].get(GRB_StringAttr_VarName) << "," << f2[i][j][l][k].get(GRB_DoubleAttr_X) << ",\n";
			checkfile.close();*/
			if (CGSP.get(GRB_IntAttr_Status) == 2) {
				outfile << filename[itr] << "," <<htime<<","<<hobj<<","<< igap << "," << "yes" << "," << o << "," << mtime << "," << optime << "," << nodecnt << "," << nt<< "," << viol <<","<< act_cost << "," << modcost << ",\n";
			}
			else
				outfile << filename[itr] << "," << htime << "," << hobj << ","<< igap << "," << "no" << "," << o << "," << mtime << "," << optime << "," << nodecnt << "," <<nt<< "," << viol << "," << act_cost << "," << modcost << ",\n";

			cout << "Ended work on " << filename[itr] << endl;
		}

		/*Checking if there are exceptions in optimizations*/
		catch (GRBException e) {
			//system("pause");
			cout << "Error code = " << e.getErrorCode() << endl;
			cout << e.getMessage() << endl;
			outfile << e.getMessage() << ",\n";
		}
		catch (...) {
			cout << "Exception during optimization" << endl;
			//system("pause");
			outfile << "Exception during optimization" << ",\n";
		}
	}
	outfile.close();
	getchar();
	return 0;

}