/* Game Theory Planner */

// Set a 2 second time limit
//m->set(GRB_DoubleParam_TimeLimit, 2);

#include "ros/ros.h"
#include "gurobi_c++.h"

#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseArray.h"

#include <iostream>
#include <sstream>

//#include <valarray>
using namespace std;

#define N 20

#define f 10.0

class mpc {       // Iterative Best Response
public: // Access specifier
    mpc(ros::NodeHandle* node){
      
        sub = node->subscribe("gtp", 10, &mpc::mpcCallback, this);
        pub = node->advertise<geometry_msgs::Quaternion>("control", 2);
        pubTrajectory = node->advertise<geometry_msgs::PoseArray>("mpc_trajectory", 2);

        // Adding the optimization variables
        
        // Setting up the variable type (continuous, integer, ...) and the variable constraints

        for (int i = 0; i < n_st * N; i++) {
            xtype[i] = GRB_CONTINUOUS;
            xlb[i] = -GRB_INFINITY;
            xub[i] = GRB_INFINITY;
        }


        // TODO: give proper values for the boundaries
        for (int i = 0; i < n_con * N; i++) {
            utype[i] = GRB_CONTINUOUS;
            if (i % n_con == 1) {
                ulb[i] = 0;
                uub[i] = 1;
            }
            if (i % n_con == 2) {
                ulb[i] = 0;
                uub[i] = 1;
            }
            if (i % n_con == 3) {
                ulb[i] = 0;
                uub[i] = 1;
            }
            if (i % n_con == 0) {
                ulb[i] = 0;
                uub[i] = 1;
            }

        }

        // Adding the variables, and setting constraints on the variables
        x = model.addVars(xlb, xub, NULL, xtype, NULL, (int)n_st * N);
        u = model.addVars(ulb, uub, NULL, utype, NULL, (int)n_con * N);

        // Adding the model constraints
        //nlquadModel();
        lquadModel();
        //kineticModel();

        // Pre-computing part of the objective
        double R[n_con][n_con] = 
        {{1, 0, 0, 0},
        {0, 1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1}};
        int cols = 4;

        // Constructing the objective
        // (x[n * n_st + i] - path.poses[i].pose.x)*Q[i*cols+j]*(x[j * n_st + 0] - path.poses[j].pose.x)
        for (int n = 0; n < N; n++){ // For each time step
            // Quad part (in the form of uT*R*u), 4 for u inputs
            for (int i = 0; i < n_con; i++){ 
                for (int j = 0; j < n_con; j++){
                    if (R[i][j] != 0){
                        contObj += R[i][j]*x[n * n_st + i]*x[n * n_st + j];
                    }
                }
            }
        }
        
        /*
        // Initilizing to 0's for the first iteration
        for (int i = 0; i < 3 * N; i++) {
            p[i] = 0;
        }
        for (int i = 0; i < N; i++) {
            mu[i] = 0;
        }
        */
        ROS_WARN_STREAM("Initialized MPC node");
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


    GRBQuadExpr obj = 0;
    GRBQuadExpr contObj = 0;
    /* Add variables to the model */
    double* xlb = new double[N * n_st]{ -GRB_INFINITY };
    double* xub = new double[N * n_st]{ GRB_INFINITY };
    char* xtype = new char[N * n_st]{ GRB_CONTINUOUS };

    double* ulb = new double[N * n_con]{ 0 };
    double* uub = new double[N * n_con]{ 1 };
    char* utype = new char[N * n_con]{ GRB_CONTINUOUS };

    // Array to store the constraints to remove them quickly
    GRBQConstr* colli_con = new GRBQConstr[N];

    double llb = 0;
    double lub = 1000; // Change to the track length if possible, otherwise +inf

    bool firstIteration = 1;

    //ros::NodeHandle* n;
    ros::Publisher pub;
    ros::Publisher pubTrajectory;
    ros::Subscriber sub;

    geometry_msgs::PoseArray path;

};

void mpc::nlquadModel() {
    // INCOMPELETE

    GRBQuadExpr xdot[3] = 0;
    GRBLinExpr xdotLin[6] = 0;
    GRBQuadExpr xdotQuad[3] = 0;

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
    double cQ = 0.055562; // Called cP, taken from AirSim / blob / master / AirLib / include / vehicles / multirotor / RotorParams.hpp

    /*
    // Change the lower bound, upper bound, and type
    // Working around the nonlinearities by defining new variables 
    GRBVar* cx = model.addVars(xlb, xub, NULL, xtype, NULL, 3 * N);
    GRBVar* sx = model.addVars(xlb, xub, NULL, xtype, NULL, 3 * N);
    for(int n = 0; n < N; n++){
        model.addGenConstrSin(x[n * n_st + 6], sx[3 * n + 0]);
        model.addGenConstrSin(x[n * n_st + 7], sx[3 * n + 1]);
        model.addGenConstrSin(x[n * n_st + 8], sx[3 * n + 2]);

        model.addGenConstrCos(x[n * n_st + 6], cx[3 * n + 0]);
        model.addGenConstrCos(x[n * n_st + 7], cx[3 * n + 1]);
        model.addGenConstrCos(x[n * n_st + 8], cx[3 * n + 2]); 
    }
    */

    for(int n = 0; n < N - 1; n++){ // The last time step is constraint to the time step before it
        //xd
        xdotLin[0] = x[n * n_st + 3];
        // yd
        xdotLin[1] = x[n * n_st + 4];
        // zd
        xdotLin[2] = x[n * n_st + 5];
        // xdd, horizontal
        //xdot[3] = (cT * (u[n * n_con + 0] * u[n * n_con + 0] + u[n * n_con + 1] * u[n * n_con + 1] + u[n * n_con + 2] * u[n * n_con + 2] + u[n * n_con + 3] * u[n * n_con + 3]) / m) * (cos(x[n * n_st + 6]) * sin(x[n * n_st + 7]) * cos(x[n * n_st + 8]) + sin(x[n * n_st + 6]) * sin(x[n * n_st + 8]));
        //xdot[0] = (cT * (u[n * n_con + 0] * u[n * n_con + 0] + u[n * n_con + 1] * u[n * n_con + 1] + u[n * n_con + 2] * u[n * n_con + 2] + u[n * n_con + 3] * u[n * n_con + 3]) / m) * (cx[n * 3 + 0] * sx[n * 3 + 1] * cx[n * 3 + 2] + sx[n * 3 + 0] * sx[n * 3 + 2]);
        // ydd, horizontal
        //xdot[4] = (cT * (u[n * n_con + 0] * u[n * n_con + 0] + u[n * n_con + 1] * u[n * n_con + 1] + u[n * n_con + 2] * u[n * n_con + 2] + u[n * n_con + 3] * u[n * n_con + 3]) / m) * (cos(x[n * n_st + 6]) * sin(x[n * n_st + 7]) * sin(x[n * n_st + 8]) - sin(x[n * n_st + 8]) * cos(x[n * n_st + 8]));
        // MAKE SURE OF THE - g
        // zdd, height
        //xdot[5] = (cT * (u[n * n_con + 0] * u[n * n_con + 0] + u[n * n_con + 1] * u[n * n_con + 1] + u[n * n_con + 2] * u[n * n_con + 2] + u[n * n_con + 3] * u[n * n_con + 3]) / m) * (cos(x[n * n_st + 6]) * cos(x[n * n_st + 7])) - g;
        // Rolld, phi
        xdotLin[4] = x[n * n_st + 9];
        // Pitchd, theta
        xdotLin[5] = x[n * n_st + 10];
        // Yawd, psi
        xdotLin[6] = x[n * n_st + 11];
        // Rolldd, phi
        xdotQuad[0] = (Iy - Iz) / Ix * x[n * n_st + 10] * x[n * n_st + 11] + cT * l * (u[n * n_con + 3] * u[n * n_con + 3] - u[n * n_con + 1] * u[n * n_con + 1]) / Ix;
        // Pitchdd, theta
        xdotQuad[1] = (Iz - Ix) / Iy * x[n * n_st + 9] * x[n * n_st + 11] + cT * l * (u[n * n_con + 2] * u[n * n_con + 2] - u[n * n_con + 0] * u[n * n_con + 0]) / Iy;
        // Yawdd, psi
        xdotQuad[2] = (Ix - Iy) / Iz * x[n * n_st + 9] * x[n * n_st + 10] + cQ * (-u[n * n_con + 0] * u[n * n_con + 0] + u[n * n_con + 1] * u[n * n_con + 1] - u[n * n_con + 2] * u[n * n_con + 2] + u[n * n_con + 3] * u[n * n_con + 3]) / Iz;

        // Adding linear constraints
        for(int i = 0; i < 6; i++){
            GRBLinExpr modelConstraint = 0;
            // x(t+1) = x(t) + xdot*dt
            modelConstraint = x[n * n_st + i] + xdotLin[i] * dt - x[(n + 1) * n_st + i];
            mpc::model.addConstr(modelConstraint == 0);
        }

        // Adding quadractic constraints
        for(int i = 0; i < 3; i++){
            GRBQuadExpr modelConstraint = 0;
            // x(t+1) = x(t) + xdot*dt
            modelConstraint = x[n * n_st + i] + xdotQuad[i] * dt - x[(n + 1) * n_st + i];
            mpc::model.addQConstr(modelConstraint == 0);
        }

        /*
        double intv = 1e-3;
        double xmax = log(9.0);
        int len = (int) ceil(xmax/intv) + 1;
        double* xpts = new double[len];
        double* upts = new double[len];
        for (int i = 0; i < len; i++) {
            xpts[i] = i*intv;
            upts[i] = fx(i*intv);
        }
        model.addGenConstrPWL(x[0], u[0], len, xpts, upts, "gc1");
        */

        /*
        for(int i = 0; i < n_st; i++){
            // Change to non linear
            GRBGenExpr modelConstraint = 0;
            // x(t+1) = x(t) + xdot*dt
            modelConstraint = x[n * n_st + i] + xdot[i] * dt - x[(n + 1) * n_st + i];
            mpc::model.addConstr(modelConstraint == 0);
        }
        */
    }

}


void mpc::lquadModel() {

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
    double cQ = 0.055562; // Called cP, taken from AirSim / blob / master / AirLib / include / vehicles / multirotor / RotorParams.hpp

    double A[12][12] = { // Missing 2 g's somewhere
        {0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0 },
        {0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0 },
        {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0 },
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
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
    GRBQuadExpr obj = 0;
    GRBLinExpr temp[3] = {0, 0, 0};

    double Q[3][3] = 
    {{1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}};
    int cols = 3;

    // Constructing the objective
    // (x[n * n_st + i] - path.poses[i].pose.x)*Q[i*cols+j]*(x[j * n_st + 0] - path.poses[j].pose.x)
    for (int n = 0; n < N; n++){ // For each time step
        temp[0] = x[n * n_st + 0] - path->poses[n].position.x;
        temp[1] = x[n * n_st + 1] - path->poses[n].position.y;
        temp[2] = x[n * n_st + 2] - path->poses[n].position.z;
        // Quad part (in the form of xT*Q*x), 3 for x, y, z states
        // Add this if nessecary mtimes([con.T, R, con])
        for (int i = 0; i < 3; i++){ 
            for (int j = 0; j < 3; j++){
                if (Q[i][j] != 0){
                    obj += Q[i][j]*temp[i]*temp[j];
                }
            }
        }
    }
    obj += contObj;

    model.setObjective(obj);

    // Assuming the initial state is 12 states that are appended as the last 2 poses in the path message
    double x0[n_st] = {
        path->poses[N].position.x,
        path->poses[N].position.y,
        path->poses[N].position.z,
        path->poses[N].orientation.x,
        path->poses[N].orientation.y,
        path->poses[N].orientation.z,
        path->poses[N+1].position.x,
        path->poses[N+1].position.y,
        path->poses[N+1].position.z,
        path->poses[N+1].orientation.x,
        path->poses[N+1].orientation.y,
        path->poses[N+1].orientation.z
    };
    for(int i = 0; i < n_st; i++){
        mpc::model.addConstr(x[i] - x0[i]  == 0);
    }

    // Adding the collision constraints to the optimization problem
    //collision(opponent);

    // Optimize
    //model.update();
    model.optimize();
/*
    // Save ego trajectory in p
    for (int i = 0; i < N; i++) {
        p[3 * i + 0] = x[n_st * i + 0].get(GRB_DoubleAttr_X);
        p[3 * i + 1] = x[n_st * i + 1].get(GRB_DoubleAttr_X);
        p[3 * i + 2] = x[n_st * i + 2].get(GRB_DoubleAttr_X);
    }
*/



}

void mpc::pubCont() {
    // Publish a 4d vector (Overloaded as a quaternion message for convenience) as the 4 control inputs to the quadcopter
    // Maybe use odometry instead
    geometry_msgs::Quaternion cont;

    cont.x = u[0].get(GRB_DoubleAttr_X);
    cont.y = u[1].get(GRB_DoubleAttr_X);
    cont.z = u[2].get(GRB_DoubleAttr_X);
    cont.w = u[3].get(GRB_DoubleAttr_X);

    //ROS_INFO("%s", toSend.data.c_str());
    pub.publish(cont);

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
    pubTrajectory.publish(p);

    ros::spinOnce();

}

void mpc::mpcCallback(const geometry_msgs::PoseArray::ConstPtr& msg){
    ROS_WARN_STREAM("Recieved Pose Array!");

    // TODO: Add collision constraints
    //collision(); // Adding collision constraints

    mpcSetup(msg);
    pubCont();
    pubTraj();

}

int main(int argc, char* argv[]){

    int L = 5;
    double alpha = 1;

    ros::init(argc, argv, "mpc");
    ros::NodeHandle n;

    mpc ego = mpc(&n);


    ros::spin();

    // Get the trajectory from the polynomial trajectory generator

    // Get the t, n, and k values from PTG

    // Give it to the gtp

    /*
    double* t = 0;
    double* n = 0;
    double k = 0;

    opponent.gtp(&ego, alpha, &t[0], &n[0], k);
    for (int i = 0; i < L; i++) {
        
        ego.gtp(&opponent, alpha, &t[0], &n[0], k);
        opponent.gtp(&ego, alpha, &t[0], &n[0], k);
    }
    ego.gtp(&opponent, alpha, &t[0], &n[0], k);
    ego.getTraj();

    // Send it using a pose message to the MPC node
    */

    return 0;
}

/*
    GRBConstr c0 = model.getConstrs()[0];
    //model.remove(c0);
    //model.addConstr(vars[0] + vars[1] == 1);
    c0.set(GRB_DoubleAttr_RHS, 2);

        vars[0].set(GRB_DoubleAttr_Start, 0.5);
    vars[1].set(GRB_DoubleAttr_Start, 0.5);
    vars[2].set(GRB_DoubleAttr_Start, 0);
    model.setObjective(obj);
    model.update();

    model.optimize();
    if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL) {
        *objvalP = model.get(GRB_DoubleAttr_ObjVal);
        for (i = 0; i < cols; i++)
            solution[i] = vars[i].get(GRB_DoubleAttr_X);
        success = true;
    }
    cout << "x: " << solution[0] << " y: " << solution[1] << " z: " << solution[2] << endl;


        // Add piecewise constraints
    for (int j = 0; j < n; j++) {
        model.addGenConstrPWL(x[j], y[j], npts, xpts, ypts);
    }



*/