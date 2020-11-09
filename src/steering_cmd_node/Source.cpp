/**
* Created by: TRAN Anh Tuan 2019/11/18
*/

#include "EKF.h"
#include "Pedestrian_model.h"
#include <iostream>
#include <vector>
#include <string>
#include "boost/variant.hpp"
#include <unsupported/Eigen/CXX11/Tensor>
#include "MultiDimensionalMatrix.h"
#include "boost/multi_array.hpp"
#include <cassert>
#include <Eigen/Dense>
#include <fstream>
#include <stdlib.h>
#include <string>
#include <stdio.h>
#include "FrenetCoordinate.h"
#include "SimpleCSVLoader.h"
#include "Header.h"
#include <map>


using namespace RTCLib;
using namespace std;

class A {
	double i = 1.9;

};


int main() {
	A bb;
	int simStep = 100;
	double T = 0.01;
	MatrixXf args = MatrixXf::Zero(3,1);
	args << 200, 200, 200;
	int dim_x = 8;

	// Create Model set
	MatrixXf q(3, 1);
	MatrixXf r(6, 1);
	q<< 1e-5, 1e-5, 1e-5;
	r<< 1e-6,1e-6,0.9,0.9,1e-6,0.1;

	DiscreteSensorModel cs(q, r);
	EKF<DiscreteSensorModel> ekf(cs);


	//MatrixXd Y(6, 1);
	//MatrixXd X(6, 2);
	//Y << 1, 2, 3, 4, 5, 6;
	//X << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12;
	//cout << X << endl;

	//MatrixXd W;
	//W = LeastMeanSquare(Y, X);
	//cout << W << endl;
	// Initialize

	SimpleCSVLoader Data;
	Data.LoadPathFromCSV("Data_ident.csv");
	
	double u_norm = 1;
	int data_radius = 3;	// nearby data point number
	int find_index;
	cout << Data.data[0].size() << endl;

	DataIdent dataIdent;
	dataIdent.LoadDataFromCSV("Data_ident.csv");
	dataIdent.GetDataLocal(u_norm, data_radius);
	MatrixXd mdata_local = dataIdent.data_local_matrix;
	MatrixXd Yyedot = mdata_local.middleCols(5, 1) + mdata_local.middleCols(10,1).cwiseProduct(mdata_local.middleCols(10, 1)).cwiseProduct(mdata_local.middleCols(11, 1));
	
	MatrixXd Xyedot1 = -mdata_local.middleCols(4, 1).cwiseQuotient(mdata_local.middleCols(10, 1)) + mdata_local.middleCols(6, 1);
	MatrixXd Xyedot2 = mdata_local.middleCols(7, 1).cwiseQuotient(mdata_local.middleCols(10, 1)) + mdata_local.middleCols(11, 1);
	MatrixXd Xyedot3 = mdata_local.middleCols(13, 1);
	int Xyedot_ncols = 3;
	MatrixXd Xyedot(data_radius*2+1, Xyedot_ncols);
	Xyedot<< Xyedot1, Xyedot2, Xyedot3;

	MatrixXd Wyedot;
	Wyedot = LeastMeanSquare(Yyedot, Xyedot);
	double a11 = Wyedot(0);
	double a12 = Wyedot(1);
	double b1 = Wyedot(2);

	MatrixXd YthetaEdotdot = mdata_local.middleCols(8, 1);
	MatrixXd XthetaEdotdot = Xyedot;

	MatrixXd WthetaEdotdot;
	WthetaEdotdot = LeastMeanSquare(YthetaEdotdot, XthetaEdotdot);
	double a21 = WthetaEdotdot(0);
	double a22 = WthetaEdotdot(1);
	double b2 = WthetaEdotdot(2);

	std::map < std::string, int >  salary;

	salary["John"] = 1400;
	salary["Tom"] = 1000;
	salary["Harry"] = 0;

	int  a = salary["John"];  // 1400 
	int  b = salary["Tom"];  // 1000 
	int  c = salary["Harry"];  // 0

	//for (int i = 0; i < Data.data.size(); i++) {
	//	if (Data.data[i][0] - u_norm>0) {
	//		find_index = i;
	//		break;
	//	}
	//}

	MatrixXf x0(dim_x, 1), P0(dim_x, dim_x);
	x0 << 35, 1.5, 0, 0,0,0,0,0;							// Initial condition of the pedestrian

	P0 = MatrixXf::Identity(dim_x, dim_x);			
	cout << P0 << endl;

	MatrixXf z = MatrixXf::Zero(6, 1);				// Measurements
	MatrixXf u0 = MatrixXf::Zero(1, 1);
	z << 39, 1.5,  0.1, -0.1, 0, 0;							// Initial condition of the pedestrian

	// LOOOOOOOPPPP O_o !!!
	for (int k = 1; k < simStep; k++) {
		cout << "=================" << endl;
		cout << "Loop " << k << endl;

		tie(x0, P0) = ekf.Predict(z, x0, P0, u0, args, T);

		cout << x0 << endl;

	}

	return 0;
}