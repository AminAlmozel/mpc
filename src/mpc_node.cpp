
// Set a 2 second time limit
//m->set(GRB_DoubleParam_TimeLimit, 2);

#include "gurobi_c++.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <Eigen/Dense>

#include "ros/ros.h"

#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseArray.h"

#define n_con 4
#define n_st 12

using namespace std;

#define N 20
#define f 10.0

class mpc {
public:
    mpc(ros::NodeHandle* node) {
        model.set(GRB_IntParam_OutputFlag, 0);
        model.update();
        
        Eigen::Matrix<GRBVar, 4, 4> test;

        //sub = node->subscribe("gtp", 1, &mpc::mpcCallback, this);
        sub_gtp = node->subscribe("gtp", 1, &mpc::callback, this);
        pubControl = node->advertise<geometry_msgs::Quaternion>("control", 2);
        //pubTrajectory = node->advertise<geometry_msgs::PoseArray>("gtp", 2);
        
        double llb = 0;
        double lub = 1;
        // The zero used to NULL
        GRBVar dummy = model.addVar(llb, lub, 0, GRB_INTEGER, "l"); // Dummy variable, needed as a placeholder

        // Setting up the variable type (continuous, integer, ...) and the variable constraints
        // TODO: give proper values for the boundaries
        double rp_angle = 30 * M_PI / 180; // 20 degress
        double y_angle = 180 * M_PI / 180;
        double rp_rate = 2.90; // Roll rate, pitch rate bound
        rp_rate = 0.7;
        double y_rate = 3.793;
        y_rate = GRB_INFINITY;
        //y_rate = 0.4;
        double rp_accel = 2.90; // Roll rate, pitch rate bound
        rp_accel = 0.4; // To stay within the linear region
        double y_accel = 3.793; // Yaw rate bound, through experimentation
        y_accel = 0.4;

        double low[n_st] = { -GRB_INFINITY, -GRB_INFINITY, -GRB_INFINITY, -GRB_INFINITY, -GRB_INFINITY, -GRB_INFINITY, -rp_angle, -rp_angle, -GRB_INFINITY, -rp_rate, -rp_rate, -y_rate };
        double high[n_st] = { GRB_INFINITY, GRB_INFINITY, GRB_INFINITY, GRB_INFINITY, GRB_INFINITY, GRB_INFINITY, rp_angle, rp_angle, GRB_INFINITY, rp_rate, rp_rate, y_rate };
        memcpy(xlb0, low, sizeof(double) * n_st);
        memcpy(xub0, high, sizeof(double) * n_st);


        // For the linear model (Thrust, roll rate, pitch rate, yaw rate)
        for (int i = 0; i < n_st * (N + 1); i++) {
            xtype[i] = GRB_CONTINUOUS;
            xlb[i] = xlb0[i % n_st];
            xub[i] = xub0[i % n_st];
        }

        // To impose no constraints on the initial state
        for (int i = 0; i < n_st; i++) {
            xtype[i] = GRB_CONTINUOUS;
            xlb[i] = -GRB_INFINITY;
            xub[i] = GRB_INFINITY;
        }

        double ulb0[n_con] = { 0, -rp_accel, -rp_accel, -y_accel };
        double uub0[n_con] = { 1, rp_accel, rp_accel, y_accel };

        for (int i = 0; i < n_con * N; i++) {
            utype[i] = GRB_CONTINUOUS;
            ulb[i] = ulb0[i % n_con];
            uub[i] = uub0[i % n_con];
        }

        for (int i = 0; i < 3 * N; i++) {
            ptype[i] = GRB_CONTINUOUS;
            plb[i] = -GRB_INFINITY;
            pub[i] = GRB_INFINITY;
        }

        // Adding the variables, and setting constraints on the variables
        x = model.addVars(xlb, xub, NULL, xtype, NULL, (int)n_st * (N + 1));
        u = model.addVars(ulb, uub, NULL, utype, NULL, (int)n_con * N);
        p = model.addVars(plb, pub, NULL, ptype, NULL, (int)3 * N);


        // Computing the objective, in terms of the parameter p
        double Q[3][3] =
        { {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1} };
        double R[n_con][n_con] =
        { {0.01, 0, 0, 0},
        {0, 0.01, 0, 0},
        {0, 0, 0.01, 0},
        {0, 0, 0, 0.01} };

        obj.clear();

        GRBLinExpr temp[3] = { 0, 0, 0 };

        // Constructing the objective
        // (x[n * n_st + i] - path.poses[i].pose.x)*Q[i*cols+j]*(x[j * n_st + 0] - path.poses[j].pose.x)

        for (int n = 0; n < N; n++) { // For each time step
            temp[0] = x[(n + 1) * n_st + 0] - p[n * 3 + 0];
            temp[1] = x[(n + 1) * n_st + 1] - p[n * 3 + 1];
            temp[2] = x[(n + 1) * n_st + 2] - p[n * 3 + 2];
            // Quad part (in the form of xT*Q*x), 3 for x, y, z states
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    if (Q[i][j] != 0) {
                        obj += Q[i][j] * temp[i] * temp[j];
                    }
                }
            }
        }

        // uT*R*u
        for (int n = 0; n < N; n++) { // For each time step
            // Quad part (in the form of uT*R*u), 4 for u inputs
            for (int i = 0; i < n_con; i++) {
                for (int j = 0; j < n_con; j++) {
                    if (R[i][j] != 0) {
                        obj += R[i][j] * u[n * n_con + i] * u[n * n_con + j];
                    }
                }

            }
        }

        for (int n = 0; n < N; n++) {
            //obj += 0.1 * x[n * n_st + 11] * x[n * n_st + 11];
        }
        
        add_soft_constraint();

        model.setObjective(obj, GRB_MINIMIZE);

        pHandle = new GRBConstr[3 * N];
        state_transition_handle = new GRBConstr[n_st * N];


        // Initializing the parameters
        for (int i = 0; i < 3 * N; i++) {
            pHandle[i] = model.addConstr(p[i] == 0);
        }

        for (int i = 0; i < n_st; i++) {
            initial_st[i] = model.addConstr(x[i] == 0);
        }

        // Initializing the state transition constraints
        for (int i = 0; i < n_st * N; i++) {
            state_transition_handle[i] = model.addConstr(dummy == 0);
        }

        parameters = new double[3 * N];

        // Initialize the control inputs to hover
        double u_hover[] = { u_bar, 0, 0, 0 };
        for (int n = 0; n < N; n++) {
            for (int i = 0; i < n_con; i++) {
                u0[n][i] = u_hover[i % n_con];
            }
        }
        // Initialize the initial state to 0 (x, y, z, and yaw need not be zero)
        for (int i = 0; i < n_st; i++) {
            x0[i] = 0;
        }

        // Setup the constraints by calling lquadModel (linearizing around equilibrium, because of the previous initializations)
        //cout << "0 iteration\n";
        state_transition_constraints();
        
        gen_path();

        ROS_INFO_STREAM("Initialized MPC node");
    }

    // Class methods
    void callback(const geometry_msgs::PoseArray::ConstPtr& path);
    void callback();
    void state_transition_constraints(); // Linear model
    void collision(mpc* opponent); // Initializing collision constraints
    void set_initial_state(const geometry_msgs::PoseArray::ConstPtr& path);
    void set_reference_trajectory(const geometry_msgs::PoseArray::ConstPtr& path);
    void solveMPC();
    void pubTraj();
    void pub_cont();
    void linearize_trajectory(double* x0, const double* u0);
    void linearize_point(int n, double* x0, const double* u0);
    void nl_model(const double* x, const double* u, double* x_next) const;
    void nl_model_xdot(const double* x, const double* u, double* xdot) const;
    double univariate_c();
    double score_traj(const double* u0);
    void gen_path();
    void add_soft_constraint();
    void remove_soft_constraint();
    

    double x0[n_st];
    double u0[N][n_con];
    ros::Publisher pubControl;


private:
    // Class attributes
    double dt = 1 / f;
    /*
    const int64_t n_st = 12;
    const int64_t n_con = 4;
    */

    //GRBEnv* env = new GRBEnv("tune.prm");
    GRBEnv* env = new GRBEnv();
    GRBModel model = GRBModel(*env);

    GRBVar* x;
    GRBVar* u;
    GRBVar* p;
    GRBVar* s;
    GRBConstr* pHandle;
    GRBConstr* state_transition_handle;

    GRBQuadExpr obj;
    /* Add variables to the model */
    double* xlb = new double[(N + 1) * n_st]{ -GRB_INFINITY };
    double* xub = new double[(N + 1) * n_st]{ GRB_INFINITY };
    char* xtype = new char[(N + 1) * n_st]{ GRB_CONTINUOUS };

    double xlb0[n_st];
    double xub0[n_st];

    double* ulb = new double[N * n_con]{ 0 };
    double* uub = new double[N * n_con]{ 1 };
    char* utype = new char[N * n_con]{ GRB_CONTINUOUS };

    double* plb = new double[N * 3]{ -GRB_INFINITY };
    double* pub = new double[N * 3]{ GRB_INFINITY };
    char* ptype = new char[N * 3]{ GRB_CONTINUOUS };

    double sub0 = GRB_INFINITY;
    const int n_v = 6;
    const int s_size = n_v * (int(N) + 1);
    double* slb = new double[s_size]{ 0 };
    double* sub = new double[s_size]{ sub0 };
    char* stype = new char[s_size]{ GRB_CONTINUOUS };

    // Array to store the constraints to remove them quickly
    GRBQConstr* colli_con = new GRBQConstr[N];
    GRBConstr* initial_st = new GRBConstr[n_st];

    // Soft constraints
    bool soft_constraint = false;
    bool first_iteration = true;
    GRBConstr* soft_con = new GRBConstr[2 * (N + 1) * n_v];

    bool converged = 1;
    double* parameters;
    
    double A[N][n_st][n_st];
    double B[N][n_st][n_con];
    double x_bar[N + 1][n_st];

    static const int path_len = 6 * 3 * N;
    double path[path_len];

    // Constants
    double u_bar = 0.64; // From experimentation

    double g = 9.81;
    double m = 1; // Taken from AirLib / include / vehicles / multirotor / firmwares / mavlink / Px4MultiRotorParams.hpp
    double l = 0.2275; // Taken from AirLib / include / vehicles / multirotor / firmwares / mavlink / Px4MultiRotorParams.hpp
    //AirLib / include / vehicles / multirotor / firmwares / mavlink / Px4MultiRotorParams.hpp
    //AirLib / include / vehicles / multirotor / MultiRotorParams.hpp
    // and plugging the two links into matlab
    double Ix = 0.0066;
    double Iy = 0.0079;
    double Iz = 0.0143;
    double Jr = 6e-5;
    double cT = 4.179446268; // Max thrust taken from AirSim / blob / master / AirLib / include / vehicles / multirotor / RotorParams.hpp
    double cQ = 1.3 / 4; // Drag coefficient, taken from AirSim / blob / master / AirLib / include / vehicles / multirotor / RotorParams.hpp

    ros::NodeHandle* n;
    
    ros::Publisher pubTrajectory;
    ros::Subscriber sub_gtp;

    geometry_msgs::PoseArray path_msg;
    int nn = 0;
};

void mpc::remove_soft_constraint() {
    // Adding the constraints
    for (int n = 1; n < N; n++) {
        for (int i = 6; i < n_st; i++) { // Impose the constraints on the last 6 states (angles, and angle rates)
            model.remove(soft_con[2 * (n * n_v + i - 6) + 0]);
            model.remove(soft_con[2 * (n * n_v + i - 6) + 1]);
        }
    }

    // Removing the state constraints initialized in the constructor
    for (int n = 0; n < N; n++) {
        for (int i = 6; i < n_st; i++) {
            x[n * n_st + i].set(GRB_DoubleAttr_UB, xub0[i]);
            x[n * n_st + i].set(GRB_DoubleAttr_LB, xlb0[i]);
        }
    }
    soft_constraint = false;
}

void mpc::add_soft_constraint() {
    if (first_iteration) {
        // Initializing the variable s for soft constraint
        double sb0 = GRB_INFINITY;
        double slb0[6] = { 0, 0, 0, 0, 0, 0 };
        double sub0[6] = { sb0, sb0, sb0, sb0, sb0, sb0 }; // CHANGE THIS TO INFINITY

        for (int i = 0; i < n_v * (N + 1); i++) {
            stype[i] = GRB_CONTINUOUS;
            slb[i] = slb0[i % n_v];
            sub[i] = sub0[i % n_v];
        }
        s = model.addVars(slb, sub, NULL, stype, NULL, (int)(n_v * (N + 1)));
    }

    for (int n = 1; n < (N + 1); n++) {
        for (int i = 6; i < n_st; i++) { // Impose the constraints on the last 6 states (angles, and angle rates)
            soft_con[2 * (n * n_v + i - 6) + 0] = model.addConstr(x[n * n_st + i] - s[n * n_v + i - 6] - xub0[i] <= 0);
            soft_con[2 * (n * n_v + i - 6) + 1] = model.addConstr(x[n * n_st + i] + s[n * n_v + i - 6] - xlb0[i] >= 0);
            //soft_con[2 * (n * n_v + i - 6) + 0] = model.addConstr(x[n * n_st + i] - xub0[i] <= 0);
            //soft_con[2 * (n * n_v + i - 6) + 1] = model.addConstr(x[n * n_st + i] - xlb0[i] >= 0);
        }
    }

    // Removing the state constraints initialized in the constructor
    for (int n = 1; n < (N + 1); n++) {
        for (int i = 6; i < n_st; i++) {
            x[n * n_st + i].set(GRB_DoubleAttr_UB, GRB_INFINITY);
            x[n * n_st + i].set(GRB_DoubleAttr_LB, -GRB_INFINITY);
        }
    }

    double c = 1000;
    double W[6][6] =
    { {c, 0, 0, 0, 0, 0},
    {0, c, 0, 0, 0, 0},
    {0, 0, c, 0, 0, 0},
    {0, 0, 0, c, 0, 0},
    {0, 0, 0, 0, c, 0},
    {0, 0, 0, 0, 0, c} };
    // Minimizing the constraint violation by putting the soft constraint parameter in the objective
    // sT*W*s
    // Quad part (in the form of sT*W*s), 6 for s soft constraints
    for (int n = 0; n < N; n++) {
        for (int i = 0; i < n_v; i++) {
            for (int j = 0; j < n_v; j++) {
                if (W[i][j] != 0) {
                    obj += W[i][j] * s[n * n_v + i] * s[n * n_v + j];
                }
            }

        }
    }
    soft_constraint = true;
}

void mpc::gen_path() {
    const int path_len = 6 * 3 * N;
    double p[path_len];

    ofstream myfile;
    myfile.open("path.txt");
    double k = 1;
    double c = 1.5;
    for (int n = 0; n < 6 * N; n++) {
        double t = 6.0 * M_PI / k * double(n) / (6.0 * double(N));
        p[3 * n + 0] = c * (sin(k * t) + cos(k * t) - cos(2 * k * t));
        p[3 * n + 1] = c * (cos(k * t) - cos(2 * k * t) - cos(2.0 / 3.0 * k * t) - 1.0);
        p[3 * n + 2] = c * (cos(2 * k * t) + sin(k * t) - 1.0);
    }

    for (int i = 0; i < 3; i++){
        for (int n = 0; n < 6 * N; n++) {
            myfile << p[3 * n + i] << " ";
        }
        myfile << endl;
    }
    myfile.close();

    //double p[path_len];
    ifstream mypath;
    mypath.open("path.txt");
    //double a;
    for (int i = 0; i < 3; i++) {
        for (int n = 0; n < 6 * N; n++) {
            mypath >> path[3 * n + i];
        }
    }
}

void mpc::collision(mpc* opponent) {
    /*
        double d = 0.8;
        GRBQuadExpr colli_constraint = 0;

        if (!firstIteration) {
            for (int i = 0; i < N; i++) {
                model.remove(colli_con[i]);
            }
            firstIteration = 0;
        }



        for (int i = 0; i < N; i++) { // The x[0], x[1], and x[2] are x, y, z coords
            colli_constraint =
                (x[i * n_st + 0] - opponent->p[i * 3 + 0]) * (x[i * n_st + 0] - opponent->p[i * 3 + 0]) +
                (x[i * n_st + 1] - opponent->p[i * 3 + 1]) * (x[i * n_st + 1] - opponent->p[i * 3 + 1]) +
                (x[i * n_st + 2] - opponent->p[i * 3 + 2]) * (x[i * n_st + 2] - opponent->p[i * 3 + 2]);

            colli_con[i] = mpc::model.addQConstr(colli_constraint >= d*d); // Using d*d instead of taking the square root
        }
    */
}

void mpc::pub_cont() {
    // Throttle, RPY rates
    // Publish a 4d vector (Overloaded as a quaternion message for convenience) as the 4 control inputs to the quadcopter

    /*
    std::cout << "xf: ";
    for(int i = 0; i < n_st; i++){
        std::cout << x[(N - 1) * n_st + i].get(GRB_DoubleAttr_X) << ", ";
        if ((i + 1) % 3 == 0){std::cout << std::endl;}
    }

    std::cout << "u[i]: ";
    for(int i = 0; i < N; i++){
        std::cout << u[i* n_con + 0].get(GRB_DoubleAttr_X) << ", ";
        //if ((i + 1) % 3 == 0){std::cout << std::endl;}
    }

    std::cout << "p[i]: ";
    for(int i = 0; i < 3 * N; i++){
        std::cout << p[i].get(GRB_DoubleAttr_X) << ", ";
        if ((i + 1) % 3 == 0){std::cout << std::endl;}
    }
    */
    geometry_msgs::Quaternion cont;
    try {
        // u0 from the model
        double U[4] = {
            u0[0][0],
            u0[0][1],
            u0[0][2],
            u0[0][3]
        };

        // // Trouttle angle
        // double U[4] = {
        //     u0[0][0],
        //     x[n_st + 6].get(GRB_DoubleAttr_X), // Roll
        //     x[n_st + 7].get(GRB_DoubleAttr_X), // Pitch
        //     x[n_st + 8].get(GRB_DoubleAttr_X)  // Yaw
        // };

        // // Trouttle angle rates
        // double U[4] = {
        //     u0[0][0],
        //     x[n_st + 9].get(GRB_DoubleAttr_X), // Roll
        //     x[n_st + 10].get(GRB_DoubleAttr_X), // Pitch
        //     x[n_st + 11].get(GRB_DoubleAttr_X)  // Yaw
        // };
            // Filling up the message
        cont.x = U[0];
        cont.y = U[1];
        cont.z = U[2];
        cont.w = U[3];
    }

    catch (GRBException e)
    {
        double U[4] = { u_bar, 0, 0, 0 };

        // Filling up the message
        cont.x = U[0];
        cont.y = U[1];
        cont.z = U[2];
        cont.w = U[3];
    }
    catch (...)
    {
        cout << "Error during optimization" << endl;
    }

    // Publishing the message
    pubControl.publish(cont);

    ros::spinOnce();
}

void mpc::pubTraj() {
    /*
    // Input a list of pose messages as a pointer
    geometry_msgs::PoseArray p;
    geometry_msgs::Pose pos;

    // Save ego trajectory in p
    for (int i = 0; i < N; i++) {
        pos.position.x = x[n_st * i + 0].get(GRB_DoubleAttr_X);
        pos.position.y = x[n_st * i + 1].get(GRB_DoubleAttr_X);
        pos.position.z = x[n_st * i + 2].get(GRB_DoubleAttr_X);

        // Maybe add later, remember orientation is in quaternion, and coordinates used here are RPY

        //pos.orientation.x = x[n_st * i + 0].get(GRB_DoubleAttr_X);
        //pos.orientation.y = x[n_st * i + 1].get(GRB_DoubleAttr_X);
        //pos.orientation.z = x[n_st * i + 2].get(GRB_DoubleAttr_X);


        p.poses.push_back(pos);
    }
    std::string frame_id = "world";
    p.header.frame_id = frame_id;
    pubTrajectory.publish(p);

    ros::spinOnce();
    */

}

double mpc::score_traj(const double* u) {
    double Q[3][3] =
    { {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1} };
    double R[n_con][n_con] =
    { {0.01, 0, 0, 0},
    {0, 0.01, 0, 0},
    {0, 0, 0.01, 0},
    {0, 0, 0, 0.01} };
    double temp[3];
    double obj = 0;
    for (int n = 0; n < N; n++) { // For each time step
        temp[0] = x_bar[n + 1][0] - parameters[n * 3 + 0];
        temp[1] = x_bar[n + 1][1] - parameters[n * 3 + 1];
        temp[2] = x_bar[n + 1][2] - parameters[n * 3 + 2];
        // Quad part (in the form of xT*Q*x), 3 for x, y, z states
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                if (Q[i][j] != 0) {
                    obj += Q[i][j] * temp[i] * temp[j];
                }
            }
        }
    }

    // uT*R*u
    for (int n = 0; n < N; n++) { // For each time step
        // Quad part (in the form of uT*R*u), 4 for u inputs
        for (int i = 0; i < n_con; i++) {
            for (int j = 0; j < n_con; j++) {
                if (R[i][j] != 0) {
                    obj += R[i][j] * u[n * n_con + i] * u[n * n_con + j];
                }
            }

        }
    }
    //cout << obj << endl;
    return obj;
}

double mpc::univariate_c() {
    double u1[N][n_con];
    double u_c[N][n_con];
    double s, s1, s2, c, c1, c2;
    double c_poly[8];
    double s_poly[8];
    for (int n = 0; n < N; n++) {
        for (int i = 0; i < n_con; i++) {
            u1[n][i] = u[n * n_con + i].get(GRB_DoubleAttr_X);
        }
    }
    linearize_trajectory(x0, u0[0]);
    s1 = score_traj(u0[0]);
    c_poly[0] = 0;
    s_poly[0] = s1;
    linearize_trajectory(x0, u1[0]);
    s2 = score_traj(u1[0]);
    c_poly[1] = 1;
    s_poly[1] = s2;
    c1 = 0;
    c2 = 1;
    for (int j = 0; j < 6; j++) {
        c = (c1 + c2) / 2;
        for (int n = 0; n < N; n++) {
            for (int i = 0; i < n_con; i++) {
                u_c[n][i] = (1 - c) * u0[n][i] + (c) * u1[n][i];
            }
        }
        linearize_trajectory(x0, u_c[0]);
        s = score_traj(u_c[0]);
        c_poly[j + 2] = c;
        s_poly[j + 2] = s;
        if (s1 < s2) {
            c2 = c;
            s2 = s;
        }
        else {
            c1 = c;
            s1 = s;
        }
    }
    //for (int i = 0; i < 8; i++) {
    //    cout << c_poly[i] << ", ";
    //}
    //cout << endl;
    //for (int i = 0; i < 8; i++) {
    //    cout << s_poly[i] << ", ";
    //}
    //cout << endl;
    //cout << "c: " << c << endl;
    return c;

}

void mpc::nl_model_xdot(const double* x, const double* u, double* xdot) const {
    // IMPORTANT
    double cT = g * m / u_bar;

    //xd
    xdot[0] = x[3];
    // yd
    xdot[1] = x[4];
    // zd
    xdot[2] = x[5];
    // xdd, horizontal
    xdot[3] = (cT * (u[0]) / m) * (cos(x[6]) * sin(x[7]) * cos(x[8]) + sin(x[6]) * sin(x[8]));
    // ydd, horizontal
    xdot[4] = (cT * (u[0]) / m) * (cos(x[6]) * sin(x[7]) * sin(x[8]) - sin(x[6]) * cos(x[8]));
    // MAKE SURE OF THE - g
    // zdd, height
    xdot[5] = (cT * (u[0]) / m) * (cos(x[6]) * cos(x[7])) - g;
    // Rolld, phi
    xdot[6] = x[9];
    // Pitchd, theta
    xdot[7] = x[10];
    // Yawd, psi
    xdot[8] = x[11];
    // Rolldd, phi
    xdot[9] = (Iy - Iz) / Ix * x[10] * x[11] + cT * l * (u[1]) / Ix;
    // Pitchdd, theta
    xdot[10] = (Iz - Ix) / Iy * x[9] * x[11] + cT * l * (u[2]) / Iy;
    // Yawdd, psi
    xdot[11] = (Ix - Iy) / Iz * x[9] * x[10] + cQ * (u[3]) / Iz;
}

void mpc::nl_model(const double* x, const double* u, double* x_next) const {
    double xdot[n_st];

    nl_model_xdot(x, u, xdot);

    for (int i = 0; i < n_st; i++) {
        x_next[i] = x[i] + xdot[i] * 1 / f;
    }
}

void mpc::linearize_point(int n, double* x0, const double* u0) {
    // Consider changing the value of cT to 1
    /*
    double A0[12][12] = { // Make sure of the g's, now x. is g*pitch, y. is -g*roll
    {0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0 },
    {0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0 },
    {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0 },
    {0, 0, 0, 0, 0, 0, 0, g, 0, 0, 0, 0 },
    {0, 0, 0, 0, 0, 0, -g, 0, 0, 0, 0, 0 },
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0 },
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0 },
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 },
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 } };

    double B0[12][4] = {
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {1 / m, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, l / Ix, 0, 0},
    {0, 0, l / Iy, 0},
    {0, 0, 0, l / Iz} };
    */
    for (int i = 0; i < n_st; i++) {
        if (abs(x0[i]) > xub[i]) {
            x0[i] = x0[i] / abs(x0[i]) * xub0[i];
            initial_st[i].set(GRB_DoubleAttr_RHS, x0[i]);
            //cout << "Happened\n";

        }
    }
    
    double A0[12][12] = { // Make sure of the g's, now x. is g*pitch, y. is -g*roll
    {0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0 },
    {0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0 },
    {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0 },
    {0, 0, 0, 0, 0, 0, 0, g, 0, 0, 0, 0 },
    {0, 0, 0, 0, 0, 0, -g, 0, 0, 0, 0, 0 },
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0 },
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0 },
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 },
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, (Iy - Iz) / Ix * x0[11], (Iy - Iz) / Ix * x0[10] },
    {0, 0, 0, 0, 0, 0, 0, 0, 0, (Iz - Ix) / Iy * x0[11], 0, (Iz - Ix) / Iy * x0[9] },
    {0, 0, 0, 0, 0, 0, 0, 0, 0, (Ix - Iy) / Iz * x0[10], (Ix - Iy) / Iz * x0[9], 0 } };
    // xdd
    A0[3][6] = (cT * (u0[0]) / m) * (-sin(x0[6]) * sin(x0[7]) * cos(x0[8]) + cos(x0[6]) * sin(x0[8]));
    A0[3][7] = (cT * (u0[0]) / m) * (cos(x0[6]) * cos(x0[7]) * cos(x0[8]) + sin(x0[6]) * sin(x0[8]));
    A0[3][8] = (cT * (u0[0]) / m) * (-cos(x0[6]) * sin(x0[7]) * sin(x0[8]) + sin(x0[6]) * cos(x0[8]));

    // ydd
    A0[4][6] = (cT * (u0[0]) / m) * (-sin(x0[6]) * sin(x0[7]) * sin(x0[8]) - cos(x0[6]) * cos(x0[8]));
    A0[4][7] = (cT * (u0[0]) / m) * (cos(x0[6]) * cos(x0[7]) * sin(x0[8]) - sin(x0[6]) * cos(x0[8]));
    A0[4][8] = (cT * (u0[0]) / m) * (cos(x0[6]) * sin(x0[7]) * cos(x0[8]) + sin(x0[6]) * sin(x0[8]));

    //zdd
    A0[5][6] = (cT * (u0[0]) / m) * (-sin(x0[6]) * cos(x0[7]));
    A0[5][7] = (cT * (u0[0]) / m) * (-cos(x0[6]) * sin(x0[7]));



    double B0[12][4] = {
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {(cT / m) * (cos(x0[6]) * sin(x0[7]) * cos(x0[8]) + sin(x0[6]) * sin(x0[8])), 0, 0, 0},
    {(cT / m) * (cos(x0[6]) * sin(x0[7]) * sin(x0[8]) - sin(x0[6]) * cos(x0[8])), 0, 0, 0},
    {(cT / m) * (cos(x0[6]) * cos(x0[7])), 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, 0, 0, 0},
    {0, cT * l / Ix, 0, 0},
    {0, 0, cT * l / Iy, 0},
    {0, 0, 0, cQ * l / Iz} };
    
    for (int i = 0; i < n_st; i++) {
        for (int j = 0; j < n_st; j++) {
            //A[i * n_st + j] = (*A0)[i * n_st + j];
            A[n][i][j] = A0[i][j];
        }
        for (int j = 0; j < n_con; j++) {
            B[n][i][j] = B0[i][j];
        }
    }

}

void mpc::linearize_trajectory(double* x0, const double* u) {

    double xi[n_st]; // Will hold the current state
    double x_next[n_st];
    double ui[n_con]; // Will hold the current input

    for (int i = 0; i < n_st; i++) {
        xi[i] = x0[i]; // Setting the current state to be the initial state
        x_bar[0][i] = x0[i];
    }

    for (int n = 0; n < N; n++) {
        for (int i = 0; i < n_con; i++) {
            ui[i] = u[n * n_con + i];
        }
        linearize_point(n, xi, ui);
        nl_model(xi, ui, x_next); // Use the nonlinear model to get the next state using the current state and input

        for (int k = 0; k < n_st; k++) {
            xi[k] = x_next[k];
            x_bar[n + 1][k] = x_next[k];
        }
    }
}

void mpc::state_transition_constraints() {
    //ROS_WARN_STREAM("Using the linear model!");

    linearize_trajectory(x0, u0[0]);

    /*
    for (int n = 0; n < N; n++) {
        for (int i = 0; i < n_st; i++) {
            for (int j = 0; j < n_st; j++) {
                //cout << A[n][i][j] << ',';
            }

            for (int k = 0; k < n_con; k++) {
                //cout << B[n][i][k] << ',';
            }
            cout << endl;
        }
        cout << endl;
    }
    */
    

    double xdot[n_st];

    GRBLinExpr dx = 0;
    for (int n = 0; n < N; n++) {
        nl_model_xdot(x_bar[n], u0[n], xdot);
        for (int i = 0; i < n_st; i++) { // Rows
            dx.clear(); //dx = 0;
            for (int j = 0; j < n_st; j++) { // Cols
                if (A[n][i][j] != 0) {
                    dx += A[n][i][j] * (mpc::x[n * n_st + j] - x_bar[n][j]);
                }
            }
            for (int k = 0; k < n_con; k++) { // Cols
                if (B[n][i][k] != 0) {
                    dx += B[n][i][k] * (mpc::u[n * n_con + k] - u0[n][k]);
                }
            }
            dx += xdot[i];
            // x_t+1 = x_t + (A*x_t + B*u_t) * dt
            //          x_t + (A*x_t + B*u_t) * dt - x_t+1 == 0
            model.remove(state_transition_handle[n * n_st + i]);
            state_transition_handle[n * n_st + i] = model.addConstr(mpc::x[n * n_st + i] + dx * mpc::dt - mpc::x[(n + 1) * n_st + i] == 0);
        }
    }
}

void mpc::solveMPC() {
    //model.set(GRB_IntParam_TuneResults, 1);
    //// Tune the model
    //model.tune();
    //// Get the number of tuning results
    //int resultcount = model.get(GRB_IntAttr_TuneResultCount);
    //if (resultcount > 0) {
    //    // Load the tuned parameters into the model's environment
    //    model.getTuneResult(0);
    //    // Write tuned parameters to a file
    //    model.write("tune.prm");
    //}
    //model.set(GRB_IntParam_Presolve, 0);
    //model.set(GRB_IntParam_NonConvex, 2);
    try {
        // Optimize
        model.update();
        model.optimize();
        double epsilon = 1e-10;
        double conv = 5e-2; // If the norm is less than this, then it has converged
        double temp_u;
        double temp;
        double norm = 0;
        double c = univariate_c();
        for (int n = 0; n < N; n++) {
            for (int i = 0; i < n_con; i++) {
                temp_u = u[n * n_con + i].get(GRB_DoubleAttr_X);
                // Consider making a univariate optimization in c to converge faster
                //double c = 0.1;
                temp = temp_u - u0[n][i];
                norm += temp * temp;
                u0[n][i] = c * temp_u + (1 - c) * u0[n][i];
            }
        }
        if (norm <= conv) {
            converged = true;
            //cout << "Conv\n";
        }
    }
    catch (GRBException e)
    {
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
        
        if (!soft_constraint) {
            cout << "Big problem boss" << endl;
            //add_soft_constraint();
        }
        cout << model.get(GRB_IntAttr_Status) << endl;
        model.set(GRB_IntParam_DualReductions, 0);
    }
    catch (...)
    {
        cout << "Error during optimization" << endl;
    }
}

void mpc::set_reference_trajectory(const geometry_msgs::PoseArray::ConstPtr& path) {

    // int index = 0;
    // double dist[3];
    // double min = 1e5;
    // double sum;
    // // Finding the point closest to the initial position
    // for (int i = 0; i < path_len / 3; i++) {
    //     dist[0] = path[3 * i + 0] - x0[0];
    //     dist[1] = path[3 * i + 1] - x0[1];
    //     dist[2] = path[3 * i + 2] - x0[2];
    //     sum = dist[0] * dist[0] + dist[1] * dist[1] + dist[2] * dist[2];
    //     if (sum < min) {
    //         min = sum;
    //         index = i;
    //     }
    // }
    // index++;
    //cout << index << endl;

    //std::cout << "Reference: " << p_i[0] << ", " << p_i[1] << ", " << p_i[2] << std::endl;
    //double p_i[3] = {0.5, -9.7, -0.78};
    //double p_i[3] = {-2, 1.65, 0.1};
    //double p_i[3] = { 0, 3, 10 };
    double p_i[3] = { 1.1, 0, 1 };
    // Getting the parameters (reference path) from the received message, and using them as equality constraints    
    for (int n = 0; n < N; n++) {
        // parameters[n * 3 + 0] = path[(3 * (n + index) + 0) % path_len];
        // parameters[n * 3 + 1] = path[(3 * (n + index) + 1) % path_len];
        // parameters[n * 3 + 2] = path[(3 * (n + index) + 2) % path_len];

        // parameters[n * 3 + 0] = p_i[0];
        // parameters[n * 3 + 1] = p_i[1];
        // parameters[n * 3 + 2] = p_i[2];

        // +2 Because the first two poses are the initial state
        parameters[n * 3 + 0] = path->poses[n + 2].position.x;
        parameters[n * 3 + 1] = path->poses[n + 2].position.y;
        parameters[n * 3 + 2] = path->poses[n + 2].position.z;
    }

    for (int i = 0; i < 3 * N; i++) {
        pHandle[i].set(GRB_DoubleAttr_RHS, parameters[i]);
    }
}

void mpc::set_initial_state(const geometry_msgs::PoseArray::ConstPtr& path) {
    double x0[n_st] = {
        path->poses[0].position.x,
        path->poses[0].position.y,
        path->poses[0].position.z,
        path->poses[1].position.x,
        path->poses[1].position.y,
        path->poses[1].position.z,
        path->poses[0].orientation.x,
        path->poses[0].orientation.y,
        path->poses[0].orientation.z,
        path->poses[1].orientation.x,
        path->poses[1].orientation.y,
        path->poses[1].orientation.z
    };
    for (int i = 0; i < n_st; i++) {
        mpc::x0[i] = x0[i];
    }
    for (int i = 0; i < n_st; i++) {
        initial_st[i].set(GRB_DoubleAttr_RHS, x0[i]);
    }
}

void mpc::callback(const geometry_msgs::PoseArray::ConstPtr& path) {
    //ROS_INFO_STREAM("Recieved Pose Array!");
    int L = 15;
    // TODO: Add collision constraints
    //collision(); // Adding collision constraints
    set_initial_state(path);
    set_reference_trajectory(path);
    converged = false;
    if (soft_constraint) {
        //remove_soft_constraint();
    }

    for (int i = 0; i < L; i++) {
        // Solve the problem with the given path, then take the solution, and store the control inputs
        solveMPC();
        // Update the state transition constraints by generating A and B matricies around the new trajectory
        state_transition_constraints();
        if (converged == true) {
            //break;
        }
    }

    pub_cont();
    //pubTraj();

    // Printing

    std::cout << "x[" << nn  << "]:";
    for (int i = 0; i < n_st; i++) {
        std::cout << x0[i] << ", ";
    }
    std::cout << std::endl;
    std::cout << "u[" << nn << "]:";
    for (int i = 0; i < n_con; i++) {
        std::cout << u0[0][i] << ", ";
    }
    std::cout << std::endl;
    std::cout << "r[" << nn  << "]:";
    for (int i = 0; i < 3; i++) {
        std::cout << parameters[i] << ", ";
    }
    std::cout << std::endl;
    nn++;
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "mpc");
    ros::NodeHandle n;

    mpc ego = mpc(&n);

    geometry_msgs::Quaternion cont;
    double U[4] = {
        0.64,
        0,
        0,
        0
    };
    
    // Filling up the message
    cont.x = U[0];
    cont.y = U[1];
    cont.z = U[2];
    cont.w = U[3];
    ros::Duration(1.0).sleep();
    ego.pubControl.publish(cont);

    //x[20]:1.02383, 0.23585, 1.00095, 0.184719, -0.518006, 0.0487062, -0.0657238, 0.122437, 4.93842, 0.69976, -0.700007, 2.80343,
    ros::spin();
    return 0;
}
