#include "ros/ros.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include "mpc.h"

// Class Car Params
CarParams::CarParams() {
	a11 = 7.1;
	a12 = 0.9;
	a21 = -45.8;
	a22 = -2.9;
	b1 = 6.9;
	b2 = 15.5;
	T = 0.1;
	v0 = 1;
	nx = 4;
	nu = 1;
	N = 50;
};

void CarParams::SetParams(vector<double> vec_params) {
	a11 = vec_params[0];
	a12 = vec_params[1];
	a21 = vec_params[2];
	a22 = vec_params[3];
	b1 = vec_params[4];
	b2 = vec_params[5];
};


// Class DataIdent
DataIdent::DataIdent() {};

void DataIdent::LoadDataFromCSV(std::string fn) {
	data_all.LoadPathFromCSV(fn);
}

void DataIdent::FindClosestPoint(double u_current) {
	// cout << "prev_index: "<<prev_index<<endl;
	// cout << "u_current: "<< u_current<<endl;
	if (data_all.data[prev_index][0] < u_current) {
		for (int i = 0; i < (int)data_all.data.size() - prev_index; i++) {
			if (data_all.data[i][0] >= u_current) {
				prev_index = i;
				// cout << "prev_index: "<<prev_index<<endl;
				break;
			}
		}
	}
	else {
		for (int i = prev_index; i > 0; i--) {
			if (data_all.data[i][0] <= u_current) {
				prev_index = i;
				// cout << "prev_index: "<<prev_index<<endl;
				break;
			}
		}
	}
}

MatrixXd DataIdent::ConvertVector2Matrix(vector<vector<double>> data_local) {
	int n_rows = data_local.size();
	int n_cols = data_local[0].size();
	MatrixXd temp(n_rows, n_cols);
	for (int i = 0; i < n_rows; i++) {				// Is there any faster way to copy Vector to Matrix?
		for (int j = 0; j < n_cols; j++) {
			temp(i, j) = data_local[i][j];
		}
	}
	return temp;
}

MatrixXd DataIdent::GetDataLocal(double u_current, int radius) {
	// cout << "Here 2.5"<<endl;
	FindClosestPoint(u_current);
	// cout << "Here3"<<endl;
	int data_length = data_all.data.size();
		// cout << "Here1"<<endl;
	vector<vector<double> > data_local;
	for (int i = 0; i < 2 * radius + 1; i++) {
		if (prev_index + i - radius > data_length-1) {
			data_local.push_back(data_all.data[prev_index + i - radius - data_length]);
		}
		else if (prev_index + i - radius < 0) {
			data_local.push_back(data_all.data[prev_index + i - radius + data_length]);
		}
		else {
			data_local.push_back(data_all.data[prev_index + i - radius]);
		}
	}
		// cout << "Here2"<<endl;

	return ConvertVector2Matrix(data_local);
};



vector<double> DataIdent::ParamIdentCalc(double u_norm, int data_radius) {
	MatrixXd mdata_local = GetDataLocal(u_norm, data_radius);
	// cout << "prev index: "<< prev_index<<endl;
	// cout <<mdata_local<<endl;
	MatrixXd Yyedot = mdata_local.middleCols(5, 1) + mdata_local.middleCols(10, 1).cwiseProduct(mdata_local.middleCols(10, 1)).cwiseProduct(mdata_local.middleCols(11, 1));
	// cout << "HERE1"<<endl;

	MatrixXd Xyedot1 = -mdata_local.middleCols(4, 1).cwiseQuotient(mdata_local.middleCols(10, 1)) + mdata_local.middleCols(6, 1);
	MatrixXd Xyedot2 = mdata_local.middleCols(7, 1).cwiseQuotient(mdata_local.middleCols(10, 1)) + mdata_local.middleCols(11, 1);
	MatrixXd Xyedot3 = mdata_local.middleCols(13, 1);
	int Xyedot_ncols = 3;
	// cout << "HERE2"<<endl;
	// cout << Xyedot2<<endl;
	MatrixXd Xyedot(data_radius * 2 + 1, Xyedot_ncols);
	Xyedot << Xyedot1, Xyedot2, Xyedot3;
	// cout << "HERE3"<<endl;

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

	vector<double> vec{ a11,a12,a21,a22,b1,b2 };

	return vec;
}

// Least mean square
MatrixXd LeastMeanSquare (MatrixXd Y, MatrixXd X){
	return (X.transpose()*X).inverse()*X.transpose()*Y;
}


// Convert PWM to Steer angle and vice versa
double pwm2steerDeg(double pwm) {
	return -(-0.00000004849623188520 * pow(pwm, 3) + 0.00021440245188645400 * pow(pwm, 2) - 0.26402119043951200000 * pwm + 77.88901947127730000000);
}

double steerDeg2pwm(double steer) {
	return -0.01557388636656220000*pow(steer, 3) + 0.04869585096726950000*pow(steer, 2) - 18.16298336641920000000*steer + 1486.63500938688000000000;
}

double unwrap_u(double u, double u_last, double course_length, int loop) {
	double delta = u - u_last;

	if (abs(delta) <= 0.9*course_length) {	// when the car passes the origin
	}
	else if (delta > 0) {
		loop -= 2;
	}
	else {
		loop += 2;
	}
	return loop * course_length + u;
}


// Continuous-time Right-Hand-Side equations
// Koga-san's thesis, Page 48
SX func_f(vector<SX> x_vec, vector<SX> u_vec, CarParams args) {

	double a11 = args.a11;
	double a12 = args.a12;
	double a21 = args.a21;
	double a22 = args.a22;
	double b1 = args.b1;
	double b2 = args.b2;
	double T = args.T;
	double v0 = args.v0;

	SX rhs1 = x_vec[1];
	SX rhs2 = -a11 / v0 * x_vec[1] + a11 * x_vec[2] + a12 * x_vec[3] / v0 + b1 * u_vec[0];
	SX rhs3 = x_vec[3];
	SX rhs4 = -a21 * x_vec[1] / v0 + a21 * x_vec[2] + a22 * x_vec[3] / v0 + b2 * u_vec[0];

	SX rhs = SX::vertcat({ rhs1,rhs2,rhs3,rhs4 });

	return rhs;
}

// Discrete-time Right-Hand-Side equations
vector<double> func_f_discrete(vector<double> x_vec, vector<double> u_vec, CarParams args) {

	double a11 = args.a11;
	double a12 = args.a12;
	double a21 = args.a21;
	double a22 = args.a22;
	double b1 = args.b1;
	double b2 = args.b2;
	double T = args.T;
	double v0 = args.v0;

	double rhs1 = x_vec[0] + T * x_vec[1];
	double rhs2 = x_vec[1] + T * (-a11 / v0 * x_vec[1] + a11 * x_vec[2] + a12 * x_vec[3] / v0 + b1 * u_vec[0]);
	double rhs3 = x_vec[2] + T * x_vec[3];
	double rhs4 = x_vec[3] + T * (-a21 * x_vec[1] / v0 + a21 * x_vec[2] + a22 * x_vec[3] / v0 + b2 * u_vec[0]);

	vector<double> rhs{ rhs1,rhs2,rhs3,rhs4 };

	return rhs;
}


SXDict mpcFormulation(CarParams Cparams) {

	int N = Cparams.N;
	int nx = Cparams.nx;
	int nu = Cparams.nu;
	double T = Cparams.T;
	

	// State & Input definition
	SX x1 = SX::sym("y_e");
	SX x2 = SX::sym("y_e_dot");
	SX x3 = SX::sym("theta_e");
	SX x4 = SX::sym("theta_e_dot");
	vector<SX> x_vec{ x1,x2,x3,x4 };
	// cout<<x_vec[0]<<endl;
	// SX x_vec = SX::vertcat({x1,x2,x3,x4,x5,x6});

	SX u1 = SX::sym("steer_com");
	vector<SX> u_vec{ u1 };
	// SX u_vec = SX::vertcat({u1,u2});

	// Predicted Input series U = {u1, ..., uN} in N steps
	SX U = SX::sym("U", nu*N);

	// Parameter P that contains the initial state value at each MPC loop, Reference, Obstacle's position, etc.
	SX P = SX::sym("P", nx + nx);

	// Predicted state in N+1 steps
	SX X = SX::sym("X", nx*(N + 1));

	// Cost value
	SX obj = 0;

	// Weighting Matrices
	double q1 = 1000, q2 = 1, q3 = 1, q4 = 1;
	double r1 = 100;
	double s1 = 10000, s2 = 10, s3 = 100, s4 = 10;
	vector<SX> Q{ q1,q2,q3,q4 };
	vector<SX> R{ r1 };
	vector<SX> Sf{ s1,s2,s3,s4 };

	// Vector that contains equality & inequality constraints
	SX g;
	for (int i = 0; i < nx; i++) {
		g = vertcat(g, X(i) - P(i));
	}
	// cout << g <<endl;

	vector<SX> st_next(nx), st(nx), con(nu);
	SX f_value = SX::sym("f_value");

	// Calculate cost function
	for (int k = 0; k < N; k++) {
		// Sum of X'*Q*X
		for (int i = 0; i < nx; i++) {
			obj = obj + (X(k*nx + i) - P(nx + i))*Q[i] * (X(k*nx + i) - P(nx + i));
			st[i] = X(k*nx + i);
		}
		// cout<<st<<endl;

		// Sum of U'*R*U
		for (int j = 0; j < nu; j++) {
			obj = obj + U(k*nu + j)*R[j] * U(k*nu + j);
			con[j] = U(k*nu + j);
		}
		// cout <<con <<endl;

		// Next state value
		f_value = func_f(st, con, Cparams);
		for (int i = 0; i < nx; i++) {
			st_next[i] = st[i] + T * f_value(i);
			g = vertcat(g, X((k + 1)*nx + i) - st_next[i]); 	// Equality constraints: st_next = st + T * f_value
		}
	}

	// Terminal cost
	for (int i = 0; i < nx; i++) {
		obj = obj + (X(N*nx + i) - P(nx + i))*Sf[i] * (X(N*nx + i) - P(nx + i));
	}

	// Optimization (decision) variable which contains X and U
	SX OPT_variables;
	OPT_variables = SX::vertcat({ X,U });
	// cout <<OPT_variables<<endl;

	// Formulate nonilnear optimization problem
	SXDict nlp = { {"x",OPT_variables}, {"f",obj}, {"g",g}, {"p",P} };

	return nlp;
}



std::map<std::string, DM> mpcConstraints(CarParams Cparams) {
	int N = Cparams.N;
	int nx = Cparams.nx;
	int nu = Cparams.nu;

	// Input constraints
	double steer_dot_max = 20 * M_PI / 180, steer_dot_min = -steer_dot_max;
	double x1min = -3, x1max = 3, x5min = -20 * M_PI / 180, x5max = 20 * M_PI / 180;

	// Bound on g
	vector<double> lbg(nx*(N + 1)), ubg(nx*(N + 1));
	lbg = set_value_vector(lbg, 0.0, 0, nx*(N + 1) - 1, 1);			// constraints on g
	ubg = set_value_vector(ubg, 0.0, 0, nx*(N + 1) - 1, 1);			// vector<T> set_value_vector(vector<T> x, T value, int start, int end, int interval)

	// Bound on OPT_varibles
	vector<double> lbx(nx*(N + 1) + nu * N), ubx(nx*(N + 1) + nu * N);	// vector of constraints on x&u in the prediction horizon
	lbx = set_value_vector(lbx, x1min, 0, nx*(N + 1) - 1, nx);		// state x1
	ubx = set_value_vector(ubx, x1max, 0, nx*(N + 1) - 1, nx);
	lbx = set_value_vector(lbx, -inf, 1, nx*(N + 1) - 1, nx);		// state x2
	ubx = set_value_vector(ubx, inf, 1, nx*(N + 1) - 1, nx);
	lbx = set_value_vector(lbx, -inf, 2, nx*(N + 1) - 1, nx);		// state x3
	ubx = set_value_vector(ubx, inf, 2, nx*(N + 1) - 1, nx);
	lbx = set_value_vector(lbx, -inf, 3, nx*(N + 1) - 1, nx);		// state x4
	ubx = set_value_vector(ubx, inf, 3, nx*(N + 1) - 1, nx);

	lbx = set_value_vector(lbx, steer_dot_min, nx*(N + 1) + 0, nx*(N + 1) + nu * N - 1, nu);	// u1
	ubx = set_value_vector(ubx, steer_dot_max, nx*(N + 1) + 0, nx*(N + 1) + nu * N - 1, nu);

	// cout <<lbx<<endl;
	// cout <<ubx<<endl;	
	std::map<std::string, DM> args;
	args["lbg"] = lbg;
	args["ubg"] = ubg;
	args["lbx"] = lbx;
	args["ubx"] = ubx;

	return args;
	// ============= FINISHED PROBLEM SETUP ==============
}

vector<double> mpcCalculation(vector<double> stateMPC, CarParams Cparams, std::map<std::string, DM> args, SXDict nlp, Dict opts) {
	//======================================

	// clock_t c_start = clock();

	Function solver = nlpsol("solver", "ipopt", nlp, opts);

	double y_e = stateMPC[0];
	double y_e_dot = stateMPC[1];
	double theta_e = stateMPC[2];
	double theta_e_dot = stateMPC[3];

	int nx = Cparams.nx;
	int nu = Cparams.nu;
	int N = Cparams.N;

	vector<double> x0{ y_e, y_e_dot, theta_e, theta_e_dot };	// Initial value, x = [ye, yedot, theta_e, theta_edot, delta, v]
	vector<double> xs{ 0., 0, 0, 0, };		// Reference

	vector<double> Xpred0(nx*(N + 1), 0);	// Predicted states
	vector<double> Upred0(nu*N, 0);		// Predicted control input
	vector<double> xu0, p0;				// Concatenated xu and parameter P

	// Initialize Xpred0
	for (int i = 0; i < (N + 1); i++) {
		for (int j = 0; j < nx; j++) {
			Xpred0[nx*i + j] = x0[j];
		}
	}

	// vector<double> sol_x;
	vector<double> u_opt(nu);

	xu0 = Xpred0;
	xu0.insert(xu0.end(), Upred0.begin(), Upred0.end());
	args["x0"] = xu0;
	// cout <<xu0<<endl;

	// Set initial value of the calution 
	p0 = x0;
	p0.insert(p0.end(), xs.begin(), xs.end());
	args["p"] = p0;
	// cout<< p0 <<endl;
	std::map<std::string, DM>  sol;

	// Solve the optimization problem
	sol = solver(args);
	// cout << sol.at("x") <<endl;

	// Get the optimal solution which includes Xpred and Upred
	// sol_x =  sol.at("x");
	vector<double> sol_x(sol.at("x"));	// How to set the value of sol_x, like sol_x = sol.at("x"), without declaring  sol_x at every step.
  // cout<< sol_x <<endl;

  // Initiate the initial guess of Upred0 and Xpred0 for the next calculation step. 
  // Basically, we get the optimal trajectory XoptPred and optimal control input series UoptPred from sol_x.
  // Then the inital guess for the next calculation step will be 
  // Xpred0 = [XoptPred(2:end),XoptPred(end)] and Upred0 = [UoptPred(2:end),UoptPred(end)];		
  // Xpred0
	for (int i = 0; i < N + 1; i++) {
		if (i != N) {
			for (int j = 0; j < nx; j++) {
				Xpred0[nx*i + j] = sol_x.at(nx*(i + 1) + j);
			}
		}
		else {
			for (int j = 0; j < nx; j++) {
				Xpred0[nx*i + j] = sol_x.at(nx*i + j);
			}
		}

	}
	// Upred0
	for (int i = 0; i < N; i++) {
		if (i != N - 1) {
			for (int j = 0; j < nu; j++) {
				Upred0[i*nu + j] = sol_x.at(nx*(N + 1) + nu * (i + 1) + j);
			}
		}
		else {
			for (int j = 0; j < nu; j++) {
				Upred0[i*nu + j] = sol_x.at(nx*(N + 1) + nu * i + j);
			}
		}

	}
	// cout<<Upred0<<endl;
	// cout<<Xpred0<<endl;
	// cout <<sol_x<< endl;

	// Optimal control input
	for (int i = 0; i < nu; i++) {
		u_opt[i] = sol_x.at(nx*(N + 1) + i);
	}
	// cout <<u_opt_pred<<endl;
	// cout << xu0;
	// cout << sol;
	// cout<<sol_x<<endl;
	// cout<<u_opt_pred<<endl<<endl;
	// cout<< x_opt_pred<<endl;
	// cout << u_opt <<endl;
	// clock_t c_end = clock();
	// double time_elapsed_ms = 1000.0*(c_end - c_start) / CLOCKS_PER_SEC;
	// cout << "CPU time used: " <<time_elapsed_ms <<" ms" <<endl;	

	return u_opt;


}
