/* Model Predictive Control */
// Give the nonlinear model a warm start
// Set a 2 second time limit
//m->set(GRB_DoubleParam_TimeLimit, 2);

#include "ros/ros.h"
#include "gurobi_c++.h"
//#include <nlopt.hpp>
#include <Eigen/Dense>

#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseArray.h"

#include <iostream>
#include <sstream>

//#include <valarray>
using namespace std;

#define N 20

#define f 10.0

class mpc {
public:
    mpc(ros::NodeHandle* node){

        Eigen::Matrix<GRBVar, 4, 4> test;
        
        sub = node->subscribe("gtp", 1, &mpc::mpcCallback, this);
        pubControl = node->advertise<geometry_msgs::Quaternion>("control", 2);
        pubTrajectory = node->advertise<geometry_msgs::PoseArray>("mpc_trajectory", 2);

        // Setting up the variable type (continuous, integer, ...) and the variable constraints
        double rpAngle = 30 * M_PI / 180; // 30 degress
        double yAngle = 180 * M_PI / 180; // 30 degress
        double xlb0[n_st] = {-GRB_INFINITY, -GRB_INFINITY, -GRB_INFINITY, -GRB_INFINITY, -GRB_INFINITY, -GRB_INFINITY, -GRB_INFINITY, -GRB_INFINITY, -GRB_INFINITY, -GRB_INFINITY, -GRB_INFINITY, -GRB_INFINITY};
        double xub0[n_st] = {GRB_INFINITY, GRB_INFINITY, GRB_INFINITY, GRB_INFINITY, GRB_INFINITY, GRB_INFINITY, GRB_INFINITY, GRB_INFINITY, GRB_INFINITY, GRB_INFINITY, GRB_INFINITY, GRB_INFINITY};

        if (~linearModel){ u_bar = 0;}
        double rpb = 2.90; // Roll rate, pitch rate bound
        rpb = 0.3; // To stay within the linear region
        double yb = 3.793; // Yaw bound, through experimentation
        yb = 0.3;

        // TODO: give proper values for the boundaries
        // For the nonlinear model (motor values)
        /*
        ulb0[n_con] = {0, 0, 0, 0};
        uub0[n_con] = {1, 1, 1, 1};
        */

        // For the linear model (Thrust, roll rate, pitch rate, yaw rate)
        double ulb0[n_con] = {0 - u_bar, -rpb, -rpb, -yb};
        double uub0[n_con] = {1 - u_bar, rpb, rpb, yb};

        for (int i = 0; i < n_st * N; i++) {
            xtype[i] = GRB_CONTINUOUS;
            xlb[i] = xlb0[ i % n_st ];
            xub[i] = xub0[ i % n_st ];
        }

        for (int i = 0; i < n_con * N; i++) {
            utype[i] = GRB_CONTINUOUS;
            ulb[i] = ulb0[ i % n_con ];
            uub[i] = uub0[ i % n_con ];
        }

        for (int i = 0; i < 3 * N; i++) {
            ptype[i] = GRB_CONTINUOUS;
            plb[i] = -GRB_INFINITY;
            pub[i] = GRB_INFINITY;
        }

        // Adding the variables, and setting constraints on the variables
        x = model.addVars(xlb, xub, NULL, xtype, NULL, (int)n_st * N);
        u = model.addVars(ulb, uub, NULL, utype, NULL, (int)n_con * N);
        p = model.addVars(plb, pub, NULL, ptype, NULL, (int)3 * N);
        
        if (~linearModel){ // 0 For the nonlinear model
            // Auxiliary variables for use in the nonlinear model
            cx = model.addVars(xlb, xub, NULL, xtype, NULL, 3 * N);
            sx = model.addVars(xlb, xub, NULL, xtype, NULL, 3 * N);
            T = model.addVars(xlb, xub, NULL, xtype, NULL, N);
            c6s7 = model.addVars(xlb, xub, NULL, xtype, NULL, N);
            c6s7c8 = model.addVars(xlb, xub, NULL, xtype, NULL, N);
            s6s8 = model.addVars(xlb, xub, NULL, xtype, NULL, N);
            c6s7s8 = model.addVars(xlb, xub, NULL, xtype, NULL, N);
            s6c8 = model.addVars(xlb, xub, NULL, xtype, NULL, N);
            c6c7 = model.addVars(xlb, xub, NULL, xtype, NULL, N);
        }

        // Computing the objective, in terms of the parameter p
        double Q[3][3] = 
        {{1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}};
        double R[n_con][n_con] = 
        {{0.01, 0, 0, 0},
        {0, 0.01, 0, 0},
        {0, 0, 0.01, 0},
        {0, 0, 0, 0.01}};

        obj = 0;

        GRBLinExpr temp[3] = {0, 0, 0};

        // Constructing the objective
        // (x[n * n_st + i] - path.poses[i].pose.x)*Q[i*cols+j]*(x[j * n_st + 0] - path.poses[j].pose.x)

        double r[] = {-2, 1.65, 0.1 + 10};
        for (int n = 0; n < N; n++){ // For each time step
            temp[0] = x[n * n_st + 0] - p[n * 3 + 0];
            temp[1] = x[n * n_st + 1] - p[n * 3 + 1];
            temp[2] = x[n * n_st + 2] - p[n * 3 + 2];
            // Quad part (in the form of xT*Q*x), 3 for x, y, z states
            for (int i = 0; i < 3; i++){ 
                for (int j = 0; j < 3; j++){
                    if (Q[i][j] != 0){
                        obj += Q[i][j]*temp[i]*temp[j];
                    }
                }
            }
        }

        // Constructing the objective
        
        // uT*R*u
        for (int n = 0; n < N; n++){ // For each time step
            // Quad part (in the form of uT*R*u), 4 for u inputs
            for (int i = 0; i < n_con; i++){ 
                for (int j = 0; j < n_con; j++){
                    if (R[i][j] != 0){
                        obj += R[i][j]*u[n * n_con + i]*u[n * n_con + j];
                    }
                }
            }
        }
        

        model.setObjective(obj, GRB_MINIMIZE);

        pHandle = new GRBConstr[3 * N];
        

        // Initializing the parameters
        for(int i = 0; i < 3 * N; i++){
            pHandle[i] = model.addConstr(p[i] == 0);
        }

        for(int i = 0; i < n_st; i++){
            initial_st[i] = model.addConstr(x[i] == 0);
        }
        
        // Adding the model constraints

        //kineticModel();
        lquadModel();
        //nlquadModel();
        
        //pHandle = model.addConstrs(p == 0);
        
        parameters = new double[3 * N];

        model.set(GRB_IntParam_OutputFlag, 0);
        model.update();

        ROS_INFO_STREAM("Initialized MPC node");
    }

    // Class methods
    void mpcCallback(const geometry_msgs::PoseArray::ConstPtr& msg);
    void kineticModel(); // Simple acceleration model
    void nlquadModel(); // Nonlinear model
    void lquadModel(); // Linear model
    void collision(mpc* opponent); // Initializing collision constraints
    void mpcSetup(const geometry_msgs::PoseArray::ConstPtr& path);
    void pubTraj();
    void pubCont();

private:

    // Class attributes
    double dt = 1/f;
    const int64_t n_st = 12;
    const int64_t n_con = 4;
    GRBEnv* env = new GRBEnv();
    GRBModel model = GRBModel(*env);

    GRBVar* x;
    GRBVar* u;
    GRBVar* p;
    GRBConstr* pHandle;

    GRBQuadExpr obj;
    /* Add variables to the model */
    double* xlb = new double[N * n_st]{ -GRB_INFINITY };
    double* xub = new double[N * n_st]{ GRB_INFINITY };
    char* xtype = new char[N * n_st]{ GRB_CONTINUOUS };

    double* ulb = new double[N * n_con]{ 0 };
    double* uub = new double[N * n_con]{ 1 };
    char* utype = new char[N * n_con]{ GRB_CONTINUOUS };

    double* plb = new double[N * 3]{ -GRB_INFINITY };
    double* pub = new double[N * 3]{ GRB_INFINITY };
    char* ptype = new char[N * 3]{ GRB_CONTINUOUS };

    // Auxiliary variables for use in the nonlinear model
    GRBVar* cx;
    GRBVar* sx;
    GRBVar* T;
    GRBVar* c6s7;
    GRBVar* c6s7c8;
    GRBVar* s6s8;
    GRBVar* c6s7s8;
    GRBVar* s6c8;
    GRBVar* c6c7;

    // Array to store the constraints to remove them quickly
    GRBQConstr* colli_con = new GRBQConstr[N];
    GRBConstr* initial_st = new GRBConstr[n_st];



    bool firstIteration = 1;
    bool linearModel = 1; // 0 for nonlinear model
    double* parameters;

    //ros::NodeHandle* n;
    ros::Publisher pubControl;
    ros::Publisher pubTrajectory;
    ros::Subscriber sub;

    geometry_msgs::PoseArray path;

    // Constants
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
    double cQ = 1.3/4; // Drag coefficient, taken from AirSim / blob / master / AirLib / include / vehicles / multirotor / RotorParams.hpp

    double u_bar = 0.64; // From experimentation

};

void mpc::nlquadModel() {
    ROS_WARN_STREAM("Using the nonlinear model!");
    // INCOMPELETE
    linearModel = 0;
    //GRBQuadExpr xdot[3] = 0;
    GRBLinExpr xdotLin[6] = 0;
    GRBQuadExpr xdotQuad[6] = 0;

    // u_bar is used for the linear model, so setting it to 0 here if the non linear model is used
    u_bar = 0;

    // Constants are defined as private class attributes 
    
    // Change the lower bound, upper bound, and type
    // Working around the nonlinearities by defining new variables 
    /*
    GRBVar* cx = model.addVars(xlb, xub, NULL, xtype, NULL, 3 * N);
    GRBVar* sx = model.addVars(xlb, xub, NULL, xtype, NULL, 3 * N);
    GRBVar* T = model.addVars(xlb, xub, NULL, xtype, NULL, N);
    GRBVar* c6s7 = model.addVars(xlb, xub, NULL, xtype, NULL, N);
    GRBVar* c6s7c8 = model.addVars(xlb, xub, NULL, xtype, NULL, N);
    GRBVar* s6s8 = model.addVars(xlb, xub, NULL, xtype, NULL, N);
    GRBVar* c6s7s8 = model.addVars(xlb, xub, NULL, xtype, NULL, N);
    GRBVar* s8c8 = model.addVars(xlb, xub, NULL, xtype, NULL, N);
    GRBVar* c6c7 = model.addVars(xlb, xub, NULL, xtype, NULL, N);
    */

    for(int n = 0; n < N; n++){
        
        model.addGenConstrSin(x[n * n_st + 6], sx[3 * n + 0]);
        model.addGenConstrSin(x[n * n_st + 7], sx[3 * n + 1]);
        model.addGenConstrSin(x[n * n_st + 8], sx[3 * n + 2]);

        model.addGenConstrCos(x[n * n_st + 6], cx[3 * n + 0]);
        model.addGenConstrCos(x[n * n_st + 7], cx[3 * n + 1]);
        model.addGenConstrCos(x[n * n_st + 8], cx[3 * n + 2]); 
        model.addQConstr(T[n] - (u[n * n_con + 0] * u[n * n_con + 0] + u[n * n_con + 1] * u[n * n_con + 1] + u[n * n_con + 2] * u[n * n_con + 2] + u[n * n_con + 3] * u[n * n_con + 3]) == 0);
        model.addQConstr(c6s7[n] - cx[3 * n + 0] * sx[3 * n + 1] == 0);
        model.addQConstr(c6s7c8[n] - c6s7[n] * cx[3 * n + 2] == 0);
        model.addQConstr(s6s8[n] - sx[3 * n + 0] * sx[3 * n + 2] == 0);
        model.addQConstr(c6s7s8[n] - c6s7[n] * sx[3 * n + 2] == 0);
        model.addQConstr(s6c8[n] - sx[3 * n + 0] * cx[3 * n + 2] == 0);
        model.addQConstr(c6c7[n] - cx[3 * n + 0] * cx[3 * n + 1] == 0);
        
    }



    for(int n = 0; n < N - 1; n++){ // The last time step is constraint to the time step before it
        //xd
        xdotLin[0] = x[n * n_st + 3];
        // yd
        xdotLin[1] = x[n * n_st + 4];
        // zd
        xdotLin[2] = x[n * n_st + 5];
        // xdd, horizontal
        //xdot[3] = (cT * (u[n * n_con + 0] * u[n * n_con + 0] + u[n * n_con + 1] * u[n * n_con + 1] + u[n * n_con + 2] * u[n * n_con + 2] + u[n * n_con + 3] * u[n * n_con + 3]) / m) * (cos(x[n * n_st + 6]) * sin(x[n * n_st + 7]) * cos(x[n * n_st + 8]) + sin(x[n * n_st + 6]) * sin(x[n * n_st + 8]));
        xdotQuad[0] = cT * T[n] / m * (c6s7c8[n] + s6s8[n]);
        // ydd, horizontal
        //xdot[4] = (cT * (u[n * n_con + 0] * u[n * n_con + 0] + u[n * n_con + 1] * u[n * n_con + 1] + u[n * n_con + 2] * u[n * n_con + 2] + u[n * n_con + 3] * u[n * n_con + 3]) / m) * (cos(x[n * n_st + 6]) * sin(x[n * n_st + 7]) * sin(x[n * n_st + 8]) - sin(x[n * n_st + 6]) * cos(x[n * n_st + 8]));
        xdotQuad[1] = cT * T[n] / m * (c6s7s8[n] + s6c8[n]);
        // MAKE SURE OF THE - g
        // zdd, height
        //xdot[5] = (cT * (u[n * n_con + 0] * u[n * n_con + 0] + u[n * n_con + 1] * u[n * n_con + 1] + u[n * n_con + 2] * u[n * n_con + 2] + u[n * n_con + 3] * u[n * n_con + 3]) / m) * (cos(x[n * n_st + 6]) * cos(x[n * n_st + 7])) - g;
        xdotQuad[2] = cT * T[n] / m * c6c7[n] - g;
        // Rolld, phi
        xdotLin[3] = x[n * n_st + 9];
        // Pitchd, theta
        xdotLin[4] = x[n * n_st + 10];
        // Yawd, psi
        xdotLin[5] = x[n * n_st + 11];
        // Rolldd, phi
        xdotQuad[3] = (Iy - Iz) / Ix * x[n * n_st + 10] * x[n * n_st + 11] + cT * l * (u[n * n_con + 3] * u[n * n_con + 3] - u[n * n_con + 1] * u[n * n_con + 1]) / Ix;
        // Pitchdd, theta
        xdotQuad[4] = (Iz - Ix) / Iy * x[n * n_st + 9] * x[n * n_st + 11] + cT * l * (u[n * n_con + 2] * u[n * n_con + 2] - u[n * n_con + 0] * u[n * n_con + 0]) / Iy;
        // Yawdd, psi
        xdotQuad[5] = (Ix - Iy) / Iz * x[n * n_st + 9] * x[n * n_st + 10] + cQ * (-u[n * n_con + 0] * u[n * n_con + 0] + u[n * n_con + 1] * u[n * n_con + 1] - u[n * n_con + 2] * u[n * n_con + 2] + u[n * n_con + 3] * u[n * n_con + 3]) / Iz;

        int k = 0;
        // Adding linear constraints
        for(int i = 0; i < 6; i++){
            GRBLinExpr modelConstraint = 0;
            if (i > 2){ k = 3;} // To match the order of the states and xdot, matching states 0, 1, 2, 6, 7, 8
            // x(t+1) = x(t) + xdot*dt
            modelConstraint = x[n * n_st + i + k] + xdotLin[i] * dt - x[(n + 1) * n_st + i + k];
            mpc::model.addConstr(modelConstraint == 0);
        }

        k = 3;
        // Adding quadractic constraints
        for(int i = 0; i < 6; i++){
            GRBQuadExpr modelConstraint = 0;
            if (i > 2){ k = 6;} // To match the order of the states and xdot, matching states 3, 4, 5, 9, 10, 11
            // x(t+1) = x(t) + xdot*dt
            modelConstraint = x[n * n_st + i + k] + xdotQuad[i] * dt - x[(n + 1) * n_st + i + k];
            mpc::model.addQConstr(modelConstraint == 0);
        }

    }

}

void mpc::lquadModel() {
    ROS_WARN_STREAM("Using the linear model!");
    linearModel = 1;
    /*
    // Since it's a linear model, try to keep the yaw around 0
    GRBQuadExpr yaw = 0;
    for (int n = 0; n < N; n++){
        yaw += 100 * x[n * n_st + 6] * x[n * n_st + 6];
    }
    model.setObjective(model.getObjective() + yaw, GRB_MINIMIZE);
    */

    // Constants are defined as private class attributes 

    double A[12][12] = { // Make sure of the g's, now x. is g*pitch, y. is -g*roll
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

    double B[12][4] = {
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {1/m, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, l/Ix, 0, 0},
        {0, 0, l/Iy, 0},
        {0, 0, 0, l/Iz} };

    GRBLinExpr dx = 0;
    for (int n = 0; n < N - 1; n++) {
        for (int i = 0; i < mpc::n_st; i++) {
            dx = 0;
            for (int j = 0; j < mpc::n_st; j++) {
                if (A[i][j] != 0)
                    dx += A[i][j] * mpc::x[n * n_st + j];
            }
            for (int k = 0; k < mpc::n_con; k++) {
                if (B[i][k] != 0)
                    dx += B[i][k] * mpc::u[n * n_con + k];
            }
            // x_t+1 = x_t + (A*x_t + B*u_t) * dt
            //          x_t + (A*x_t + B*u_t) * dt - x_t+1 == 0

            mpc::model.addConstr(mpc::x[n * n_st + i] + dx * mpc::dt - mpc::x[(n + 1) * n_st + i]  == 0);

            /*
            printf("%f *", modelConstraint.getCoeff(0));
            cout << modelConstraint.getVar(0).get(GRB_StringAttr_VarName) << endl;
            printf("%f *", modelConstraint.getCoeff(1));
            cout << modelConstraint.getVar(1).get(GRB_StringAttr_VarName) << endl;
            */



        }

    }

}

void mpc::kineticModel() {
    // WARNING: If you want to use this model be sure to change the n_st = 6, and n_con = 3, and change the state boundaries to limit the velocity, and input to limit acceleration

    double A[6][6] = { // Missing 2 g's somewhere
        {0, 0, 0, 1, 0, 0 },
        {0, 0, 0, 0, 1, 0 },
        {0, 0, 0, 0, 0, 1 },
        {0, 0, 0, 0, 0, 0 },
        {0, 0, 0, 0, 0, 0 },
        {0, 0, 0, 0, 0, 0 } };

    double B[6][3] = {
        {0, 0, 0},
        {0, 0, 0},
        {0, 0, 0},
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1} };

    if (mpc::n_st != 6 or mpc::n_con != 3){
        std::cout << "The number of states is not 6, or the number of controls is not 3" << endl;
    }

    GRBLinExpr dx = 0;
    // x_t+1 = A*x_t + B*u_t ???
    for (int n = 0; n < N - 1; n++) {
        for (int i = 0; i < mpc::n_st; i++) {
            dx = 0;
            for (int j = 0; j < mpc::n_st; j++) {
                if (A[i][j] != 0)
                    dx += A[i][j] * mpc::x[n * n_st + j];
            }
            for (int k = 0; k < mpc::n_con; k++) {
                if (B[i][k] != 0)
                    dx += B[i][k] * mpc::u[n * n_con + k];
            }
            // Make sure of this 
            // x_t+1 = x_t + (A*x_t + B*u_t) * dt
            //          x_t + (A*x_t + B*u_t) * dt - x_t+1 == 0
            mpc::model.addConstr(mpc::x[n * n_st + i] + dx * mpc::dt - mpc::x[(n + 1) * n_st + i]  == 0);

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

void mpc::mpcSetup(const geometry_msgs::PoseArray::ConstPtr& path){

    // Assuming the initial state is 12 states that are appended as the last 2 poses in the path message
    double x0[n_st] = {
        path->poses[N].position.x,
        path->poses[N].position.y,
        path->poses[N].position.z,
        path->poses[N+1].position.x,
        path->poses[N+1].position.y,
        path->poses[N+1].position.z,
        path->poses[N].orientation.x,
        path->poses[N].orientation.y,
        path->poses[N].orientation.z,
        path->poses[N+1].orientation.x,
        path->poses[N+1].orientation.y,
        path->poses[N+1].orientation.z
    };

    // Transforming the coords (rotation about z) to align it with the yaw (nessecary for the linear model to work)
    Eigen::Matrix3d rot;
    Eigen::Vector3d v1, v2;
    rot = Eigen::AngleAxis<double>(-path->poses[N].orientation.z, Eigen::Vector3d::UnitZ());

    // Position
    v1(0) = path->poses[N].position.x;
    v1(1) = path->poses[N].position.y;
    v1(2) = path->poses[N].position.z;

    // Velocity
    v2(0) = path->poses[N+1].position.x;
    v2(1) = path->poses[N+1].position.y;
    v2(2) = path->poses[N+1].position.z;
    
    // Transforming the position and yaw of the initial state
    v1 = rot * v1; // Position
    v2 = rot * v2; // Velocity
    
    x0[0] = v1(0);
    x0[1] = v1(1);
    x0[2] = v1(2);

    x0[3] = v2(0);
    x0[4] = v2(1);
    x0[5] = v2(2);

    // Setting the yaw to 0, since we're alligning the x axis with it
    x0[8] = 0;
    
   //std::cout << "Rotation by: " << -path->poses[N].orientation.z << std::endl;
   //std::cout << "v: (" << v(0) << ", " << v(1) << ", " << v(2) << ") -> (" << v_rot(0) << ", " << v_rot(1) << ", " << v_rot(2) << ")\n";

    // Transforming the position of the gates

    //std::cout << "Reference: " << p_i[0] << ", " << p_i[1] << ", " << p_i[2] << std::endl;
    //double p_i[3] = {0.5, -9.7, -0.78};
    double p_i[3] = {-2, 1.65, 0.1 + 10};
    // Getting the parameters (reference path) from the received message, and using them as equality constraints    
    for (int n = 0; n < N; n++){
        
        v1(0) = path->poses[n].position.x;
        v1(1) = path->poses[n].position.y;
        v1(2) = path->poses[n].position.z;
        v1 = rot * v1;
        
        parameters[n * 3 + 0] = p_i[0];
        parameters[n * 3 + 1] = p_i[1];
        parameters[n * 3 + 2] = p_i[2];
        /*
        parameters[n * 3 + 0] = v1(0);
        parameters[n * 3 + 1] = v1(1);
        parameters[n * 3 + 2] = v1(2);
        */

    }

    std::cout << "r0: " << path->poses[0].position.x << ", " << path->poses[0].position.y << ", " << path->poses[0].position.z << std::endl;

    std::cout << "x0: " << path->poses[N].position.x << ", " << path->poses[N].position.y << ", " << path->poses[N].position.z << std::endl;

    std::cout << "r0: " << v1(0) << ", " << v1(1) << ", " << v1(2) << std::endl;

    std::cout << "x0: ";
    for(int i = 0; i < n_st; i++){
        std::cout << x0[i] << ", ";
        if ((i + 1) % 3 == 0){std::cout << std::endl;}
    }
    std::cout << std::endl;
    
    for(int i = 0; i < 3 * N; i++){
        pHandle[i].set(GRB_DoubleAttr_RHS, parameters[i]);
    }

    for(int i = 0; i < n_st; i++){
        initial_st[i].set(GRB_DoubleAttr_RHS, x0[i]);
    }

    model.update();


/*
    if(linearModel){ // 1 for linear model
        double A[12][12] = { // Make sure of the g's, now x. is g*pitch, y. is -g*roll
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

        double B[12][4] = {
            {0, 0, 0, 0},
            {0, 0, 0, 0},
            {0, 0, 0, 0},
            {0, 0, 0, 0},
            {0, 0, 0, 0},
            {1/m, 0, 0, 0},
            {0, 0, 0, 0},
            {0, 0, 0, 0},
            {0, 0, 0, 0},
            {0, l/Ix, 0, 0},
            {0, 0, l/Iy, 0},
            {0, 0, 0, l/Iz} };
        // Adding the new ones
        GRBLinExpr dx = 0;
        for (int i = 0; i < mpc::n_st; i++) {
            int n = 0;
            dx = 0;
            for (int j = 0; j < mpc::n_st; j++) {
                if (A[i][j] != 0)
                    dx += A[i][j] * x0[j];
            }
            for (int k = 0; k < mpc::n_con; k++) {
                if (B[i][k] != 0)
                    dx += B[i][k] * mpc::u[k];
            }


            // x_t+1 = x_t + (A*x_t + B*u_t) * dt
            //          x_t + (A*x_t + B*u_t) * dt - x_t+1 == 0
            initial_st[i] = mpc::model.addQConstr(x0[i] + dx * mpc::dt - mpc::x[0 * n_st + i]  == 0);

        }

    }
    else{ // For nonlinear model
        int n = 0;
        GRBQuadExpr xdot[12] = 0;
        //xd
        xdot[0] = x0[n * n_st + 3];
        // yd
        xdot[1] = x0[n * n_st + 4];
        // zd
        xdot[2] = x0[n * n_st + 5];
        // xdd, horizontal
        xdot[3] = (cT * (u[n * n_con + 0] * u[n * n_con + 0] + u[n * n_con + 1] * u[n * n_con + 1] + u[n * n_con + 2] * u[n * n_con + 2] + u[n * n_con + 3] * u[n * n_con + 3]) / m) * (cos(x0[n * n_st + 6]) * sin(x0[n * n_st + 7]) * cos(x0[n * n_st + 8]) + sin(x0[n * n_st + 6]) * sin(x0[n * n_st + 8]));
        //xdotQuad[0] = cT * T[n] / m * (c6s7c8[n] + s6s8[n]);
        // ydd, horizontal
        xdot[4] = (cT * (u[n * n_con + 0] * u[n * n_con + 0] + u[n * n_con + 1] * u[n * n_con + 1] + u[n * n_con + 2] * u[n * n_con + 2] + u[n * n_con + 3] * u[n * n_con + 3]) / m) * (cos(x0[n * n_st + 6]) * sin(x0[n * n_st + 7]) * sin(x0[n * n_st + 8]) - sin(x0[n * n_st + 6]) * cos(x0[n * n_st + 8]));
        //xdotQuad[1] = cT * T[n] / m * (c6s7s8[n] + s8c8[n]);
        // MAKE SURE OF THE - g
        // zdd, height
        xdot[5] = (cT * (u[n * n_con + 0] * u[n * n_con + 0] + u[n * n_con + 1] * u[n * n_con + 1] + u[n * n_con + 2] * u[n * n_con + 2] + u[n * n_con + 3] * u[n * n_con + 3]) / m) * (cos(x0[n * n_st + 6]) * cos(x0[n * n_st + 7])) - g;
        //xdotQuad[2] = cT * T[n] / m * c6c7[n] - g;
        // Rolld, phi
        xdot[6] = x0[n * n_st + 9];
        // Pitchd, theta
        xdot[7] = x0[n * n_st + 10];
        // Yawd, psi
        xdot[8] = x0[n * n_st + 11];
        // Rolldd, phi
        xdot[9] = (Iy - Iz) / Ix * x0[n * n_st + 10] * x0[n * n_st + 11] + cT * l * (u[n * n_con + 3] * u[n * n_con + 3] - u[n * n_con + 1] * u[n * n_con + 1]) / Ix;
        // Pitchdd, theta
        xdot[10] = (Iz - Ix) / Iy * x0[n * n_st + 9] * x0[n * n_st + 11] + cT * l * (u[n * n_con + 2] * u[n * n_con + 2] - u[n * n_con + 0] * u[n * n_con + 0]) / Iy;
        // Yawdd, psi
        xdot[11] = (Ix - Iy) / Iz * x0[n * n_st + 9] * x0[n * n_st + 10] + cQ * (-u[n * n_con + 0] * u[n * n_con + 0] + u[n * n_con + 1] * u[n * n_con + 1] - u[n * n_con + 2] * u[n * n_con + 2] + u[n * n_con + 3] * u[n * n_con + 3]) / Iz;

        // Adding linear constraints
        for(int i = 0; i < n_st; i++){
            GRBQuadExpr modelConstraint = 0;
            // x(t+1) = x(t) + xdot*dt
            modelConstraint = x0[i] + xdot[i] * dt - x[i];
            initial_st[i] = mpc::model.addQConstr(modelConstraint == 0);
        }
    }
    */

    // Adding the collision constraints to the optimization problem
    //collision(opponent);
    try{

        //model.set(GRB_IntParam_Presolve, 0);
        model.set(GRB_IntParam_NonConvex, 2);
        // Optimize
        model.update();
        //model.reset(0);
        model.optimize();
    }
    catch (GRBException e)
    {
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    }
    catch (...)
    {
        cout << "Error during optimization" << endl;
    }

}

void mpc::pubCont() {
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
    double U[4] = {
    u[0].get(GRB_DoubleAttr_X), 
    u[1].get(GRB_DoubleAttr_X), 
    u[2].get(GRB_DoubleAttr_X), 
    u[3].get(GRB_DoubleAttr_X)};

    // Getting the control input from the solution of the optimization problem
    if(linearModel){ // linearModel = 1 for the linear model
        cont.x = U[0] + u_bar;
        cont.y = U[1];
        cont.z = U[2];
        cont.w = U[3];
    }
    else{
        cont.x = cT * (U[0] * U[0] + U[1] * U[1] + U[2] * U[2] + U[3] * U[3]);
        cont.y = cT * l * (- U[1] * U[1] + U[3] * U[3]);
        cont.z = cT * l * (- U[0] * U[0] + U[2] * U[2]);
        cont.w = cT * l * (- U[0] * U[0] + U[1] * U[1] - U[2] * U[2] + U[3] * U[3]);
    }

    pubControl.publish(cont);

    ros::spinOnce();
}

void mpc::pubTraj() {
    // Input a list of pose messages as a pointer
    geometry_msgs::PoseArray p;
    geometry_msgs::Pose pos;

    // Save ego trajectory in p
    for (int i = 0; i < N; i++) {
        pos.position.x = x[n_st * i + 0].get(GRB_DoubleAttr_X);
        pos.position.y = x[n_st * i + 1].get(GRB_DoubleAttr_X);
        pos.position.z = x[n_st * i + 2].get(GRB_DoubleAttr_X);

        // Maybe add later, remember orientation is in quaternion, and coordinates used here are RPY
        /*
        pos.orientation.x = x[n_st * i + 0].get(GRB_DoubleAttr_X);
        pos.orientation.y = x[n_st * i + 1].get(GRB_DoubleAttr_X);
        pos.orientation.z = x[n_st * i + 2].get(GRB_DoubleAttr_X);
        */

        p.poses.push_back(pos);
    }
    std::string frame_id = "world";
    p.header.frame_id = frame_id;
    pubTrajectory.publish(p);

    ros::spinOnce();

}

void mpc::mpcCallback(const geometry_msgs::PoseArray::ConstPtr& msg){
    ROS_INFO_STREAM("Recieved Pose Array!");

    // TODO: Add collision constraints
    //collision(); // Adding collision constraints

    mpcSetup(msg);
    pubCont();
    //pubTraj();

}

int main(int argc, char* argv[]){

    int L = 5;
    double alpha = 1;

    ros::init(argc, argv, "mpc");
    ros::NodeHandle n;

    mpc ego = mpc(&n);


    ros::spin();

    return 0;
}