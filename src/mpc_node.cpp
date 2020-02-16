/* Game Theory Planner */

// Set a 2 second time limit
//m->set(GRB_DoubleParam_TimeLimit, 2);

#include "ros/ros.h"
#include "gurobi_c++.h"

#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

#include <iostream>
#include <sstream>

//#include <valarray>
using namespace std;

#define N 20

#define f 10.0

class seibr {       // Iterative Best Response
public: // Access specifier
    seibr(ros::NodeHandle* node){

      pub = node->advertise<std_msgs::String>("gtp_path", 2);
      sub = node->subscribe("ptp", 10, &seibr::gtp_main, this);
    
      // Adding the optimization variables
      //l = model.addVar(llb, lub, NULL, GRB_INTEGER, "l");
      l = model.addVar(llb, lub, 0.0, GRB_INTEGER);
      
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
      //lquadModel();
      kineticModel();
      
      // Initilizing to 0's for the first iteration
      for (int i = 0; i < 3 * N; i++) {
          p[i] = 0;
      }
      for (int i = 0; i < N; i++) {
          mu[i] = 0;
      }
    }

    // Class methods
    void kineticModel(); // Simple acceleration model
    void nlquadModel(); // Nonlinear model
    void lquadModel(); // Linear model
    void collision(seibr* opponent); // Initializing collision constraints
    void gtp(seibr* opponent, double alpha, double* t, double* n, double k);
    double* getTraj();

    // Class attributes
    double dt = 1/f;
    int64_t n_st = 6;
    int64_t n_con = 3;
    GRBEnv* env = new GRBEnv();
    GRBModel model = GRBModel(*env);
    GRBVar l;
    GRBVar* x;
    GRBVar* u;
    double* p = new double[3 * N]{ 0 };
    double* mu = new double[N]{ 0 };

    GRBQuadExpr obj = 0;
    double beta[3];
    double mag = 1;
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

    void gtp_main(const std_msgs::Bool::ConstPtr& msg);

  private:
    int test;
    std::string st;
    //ros::NodeHandle* n;
    ros::Publisher pub;
    ros::Subscriber sub;


};

void seibr::nlquadModel() {
    // INCOMPELETE
    // PLACEHOLDER
    double* x = 0;
    double* xdot = 0;
    double* u = 0;

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

    //xd
    xdot[0] = x[3];
    // yd
    xdot[1] = x[4];
    // zd
    xdot[2] = x[5];
    // xdd, horizontal
    xdot[3] = (cT * (u[0] * u[0] + u[1] * u[1] + u[2] * u[2] + u[3] * u[3]) / m) * (cos(x[6]) * sin(x[7]) * cos(x[8]) + sin(x[6]) * sin(x[8]));
    // ydd, horizontal
    xdot[4] = (cT * (u[0] * u[0] + u[1] * u[1] + u[2] * u[2] + u[3] * u[3]) / m) * (cos(x[6]) * sin(x[7]) * sin(x[8]) - sin(x[8]) * cos(x[8]));
    // MAKE SURE OF THE - g
    // zdd, height
    xdot[5] = (cT * (u[0] * u[0] + u[1] * u[1] + u[2] * u[2] + u[3] * u[3]) / m) * (cos(x[6]) * cos(x[7])) - g;
    // Rolld, phi
    xdot[6] = x[9];
    // Pitchd, theta
    xdot[7] = x[10];
    // Yawd, psi
    xdot[8] = x[11];
    // Rolldd, phi
    xdot[9] = (Iy - Iz) / Ix * x[10] * x[11] + cT * l * (u[3] * u[3] - u[1] * u[1]) / Ix;
    // Pitchdd, theta
    xdot[10] = (Iz - Ix) / Iy * x[9] * x[11] + cT * l * (u[2] * u[2] - u[0] * u[0]) / Iy;
    // Yawdd, psi
    xdot[11] = (Ix - Iy) / Iz * x[9] * x[10] + cQ * (-u[0] * u[0] + u[1] * u[1] - u[2] * u[2] + u[3] * u[3]) / Iz;

}

void seibr::lquadModel() {

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

    GRBLinExpr modelConstraint = 0;
    // x_t+1 = A*x_t + B*u_t ???
    for (int i = 0; i < N - 1; i++) {
        for (int n = 0; n < seibr::n_st; n++) {
            modelConstraint = 0;
            for (int j = 0; j < seibr::n_st; j++) {
                if (A[n][j] != 0)
                    modelConstraint += A[n][j] * seibr::x[i * n_st + j];
            }
            for (int k = 0; k < seibr::n_con; k++) {
                if (B[n][k] != 0)
                    modelConstraint += B[n][k] * seibr::u[i * n_con + k];
            }
            // Make sure of this 
            // (A*x_t + B*u_t) * dt + x_t - x_t+1 == 0
            modelConstraint = modelConstraint * seibr::dt + seibr::x[i * n_st + n] - seibr::x[(i + 1) * n_st + n]; 

            /*
            printf("%f *", modelConstraint.getCoeff(0));
            cout << modelConstraint.getVar(0).get(GRB_StringAttr_VarName) << endl;
            printf("%f *", modelConstraint.getCoeff(1));
            cout << modelConstraint.getVar(1).get(GRB_StringAttr_VarName) << endl;
            */

            seibr::model.addConstr(modelConstraint == 0);

        }

    }

}

void seibr::kineticModel() {
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

    GRBLinExpr modelConstraint = 0;
    // x_t+1 = A*x_t + B*u_t ???
    for (int i = 0; i < N - 1; i++) {
        for (int n = 0; n < seibr::n_st; n++) {
            modelConstraint = 0;
            for (int j = 0; j < seibr::n_st; j++) {
                if (A[n][j] != 0)
                    modelConstraint += A[n][j] * seibr::x[i * n_st + j];
            }
            for (int k = 0; k < seibr::n_con; k++) {
                if (B[n][k] != 0)
                    modelConstraint += B[n][k] * seibr::u[i * n_con + k];
            }
            // Make sure of this 
            // (A*x_t + B*u_t) * dt + x_t - x_t+1 == 0
            modelConstraint = modelConstraint * seibr::dt + seibr::x[i * n_st + n] - seibr::x[(i + 1) * n_st + n];


            seibr::model.addConstr(modelConstraint == 0);

        }

    }
}

void seibr::collision(seibr* opponent) {

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

        colli_con[i] = seibr::model.addQConstr(colli_constraint >= d*d); // Using d*d instead of taking the square root
    }
    
}

void seibr::gtp(seibr* opponent, double alpha, double* t, double* n, double k) {
    GRBQuadExpr obj = 0;
    double beta[3];
    double mag = 1;

    // Setting up the objective
    for (int i = 0; i < N; i++){
        beta[0] = (opponent->p[i * 3 + 0] - p[i * 3 + 0]);
        beta[1] = (opponent->p[i * 3 + 1] - p[i * 3 + 1]);
        beta[2] = (opponent->p[i * 3 + 2] - p[i * 3 + 2]);
        mag = beta[0] * beta[0] + beta[1] * beta[1] + beta[2] * beta[2];
        obj += opponent->mu[i] * ((beta[0] * x[i * n_st + 0]) + (beta[1] * x[i * n_st + 1]) + (beta[2] * x[i * n_st + 2])) / mag;
    }
    obj *= alpha;

    // TODO: Make sure of this, maybe modify it
    double tTpN = (t[0] * p[3 * N + 0] + t[1] * p[3 * N + 1] + t[2] * p[3 * N + 2]);
    double pN_tTn = ((p[3 * N + 0] - t[0]) * n[0] + (p[3 * N + 1] - t[1]) * n[1] + (p[3 * N + 2] - t[2]) * n[2]);
    obj += tTpN / (1 - k * pN_tTn);

    model.setObjective(obj);

    // Adding the collision constraints to the optimization problem
    collision(opponent);

    // Optimize
    //model.update();
    model.optimize();

    // Save ego trajectory in p
    for (int i = 0; i < N; i++) {
        p[3 * i + 0] = x[n_st * i + 0].get(GRB_DoubleAttr_X);
        p[3 * i + 1] = x[n_st * i + 1].get(GRB_DoubleAttr_X);
        p[3 * i + 2] = x[n_st * i + 2].get(GRB_DoubleAttr_X);
    }
    // Save mu vector
    for (int i = 0; i < N; i++) {
        mu[i] = colli_con[i].get(GRB_DoubleAttr_Slack);
    }
}

double* seibr::getTraj() {
    // Input a list of pose messages as a pointer
    // Fill up using p from the class
    // Return it?
    return 0;
}

void seibr::gtp_main(const std_msgs::Bool::ConstPtr& msg)
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
  std_msgs::String toSend;

  //std::stringstream ss;
  //ss << "hello world ";
  toSend.data = std::string("Helo world");

  //ROS_INFO("%s", toSend.data.c_str());
  if(msg->data){
    pub.publish(toSend);
  }

  ros::spinOnce();
  
}

int main(int argc, char* argv[])
{

    int L = 5;
    double alpha = 1;

    ros::init(argc, argv, "gtp");
    ros::NodeHandle n;

    seibr opponent = seibr(&n);

    seibr ego = seibr(&n);

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