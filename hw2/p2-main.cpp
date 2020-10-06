/*======================================================================================
 * hw2-p2-main.cpp
 *
 * Implement operational space motion tracking for a 7-DOF redundant kuka 
 * by completing the "FILL ME IN" section 
 *
 * Elena Galbally, Spring 2019
 *
 *======================================================================================*/

/* --------------------------------------------------------------------------------------
   Include Required Libraries and Files
-----------------------------------------------------------------------------------------*/

#include <iostream>
#include <string>
#include <thread>
#include <math.h>

#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "Sai2Simulation.h"

#include "timer/LoopTimer.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew as part of graphicsinterface

using namespace std;

const string world_fname = "resources/hw2/world_2_iiwa.urdf";
const string robot_fname = "../resources/kuka_iiwa/kuka_iiwa.urdf";
const string robot_name = "Kuka-IIWA";
const string ee_link_name = "link6";

// const string camera_name = "camera_front";
// const string camera_name = "camera_back_top";
const string camera_name = "camera_top";

/* --------------------------------------------------------------------------------------
	Simulation Loop Initialization
-------------------------------------------------------------------------------------*/
// simulation loop
bool fSimulationRunning = false;
void control(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim);
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim);

// initialize window manager
GLFWwindow* glfwInitialize();

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);


/* =======================================================================================
   MAIN LOOP
========================================================================================== */
int main (int argc, char** argv) {
	cout << "Loading URDF world model file: " << world_fname << endl;

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_fname, false);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_fname, false);

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_fname, false);

	// set initial condition
	robot->_q << 125.9/180.0*M_PI,
				39.2/180.0*M_PI,
				-49.2/180.0*M_PI,
				70.0/180.0*M_PI,
				-62.4/180.0*M_PI,
				80.2/180.0*M_PI,
				187.2/180.0*M_PI;
	sim->setJointPositions(robot_name, robot->_q);
	robot->updateModel();
	
	// Eigen::Affine3d ee_trans;
	// robot->transform(ee_trans, ee_link_name);
	// cout << ee_trans.translation().transpose() << endl;
	// cout << ee_trans.rotation() << endl;

	// initialize GLFW window
	GLFWwindow* window = glfwInitialize();

    // set callbacks
	glfwSetKeyCallback(window, keySelect);

	// start the simulation thread first
	fSimulationRunning = true;
	thread sim_thread(simulation, robot, sim);

	// next start the control thread
	thread ctrl_thread(control, robot, sim);
	
    // while window is open:
    while (!glfwWindowShouldClose(window)) {
		// update kinematic models
		// robot->updateModel();

		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot_name, robot);
		graphics->render(camera_name, width, height);
		glfwSwapBuffers(window);
		glFinish();

	    // poll for events
	    glfwPollEvents();
	}

	// stop simulation
	fSimulationRunning = false;
	sim_thread.join();
	ctrl_thread.join();

    // destroy context
    glfwDestroyWindow(window);

    // terminate
    glfwTerminate();

	return 0;
}

/* =======================================================================================
   CONTROL LOOP
========================================================================================== */
void control(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim) {
	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(200); //200Hz timer
	double last_time = timer.elapsedTime(); //secs

	// cache variables
	bool fTimerDidSleep = true;
	Eigen::VectorXd tau = Eigen::VectorXd::Zero(robot->dof());

	// additional cache variables
	Eigen::VectorXd g(robot->dof()); //joint space gravity vector
	Eigen::MatrixXd J0(6, robot->dof()); //end effector basic Jacobian
	Eigen::MatrixXd L0(6, 6); //Lambda_0 at end effector
	Eigen::VectorXd p(6); //gravity vector at end-effector
	const Eigen::MatrixXd In = Eigen::MatrixXd::Identity(robot->dof(), robot->dof()); // n x n identity matrix
	Eigen::MatrixXd Jbar(robot->dof(), 6);	//A-weighted generalized inverse of J0
	Eigen::MatrixXd Nbar(robot->dof(), robot->dof()); //I - Jbar*J0, null space projection matrix for Jbar
	Eigen::Vector3d ee_pos; //end effector position
	Eigen::Vector3d v0; //end effector velocity 
	Eigen::Vector3d w0; //end effector angular velocity 
	Eigen::Matrix3d ee_rot_mat; //end effector rotation
	robot->rotation(ee_rot_mat, ee_link_name); // initialize
	Eigen::Quaterniond ee_rot_lambda=(Eigen::Quaterniond)ee_rot_mat; // end effector rotation in quaternion form
	//Eigen::VectorXd ee_error(6); //end effector operational space instantaneous error
	//Eigen::VectorXd v0(6); //end effector velocity 
	//Eigen::VectorXd w0(6); //end effector angularvelocity 
	
	Eigen::VectorXd v0d(3); //end effector desired velocity
	Eigen::VectorXd dv0d(3); //end effector desired acceleration
	Eigen::VectorXd w0d(4); //end effector desired angular velocity
	Eigen::VectorXd dw0d(4); //end effector desired angular acceleration
	Eigen::MatrixXd Erplus(3,4);
	Eigen::MatrixXd Eplus (6,7);//make all values 0
	Eigen::VectorXd wd(3);
	Eigen::VectorXd wddot(3);
	Eigen::VectorXd vw(6);
	Eigen::VectorXd xddot(6);
	Eigen::VectorXd xddotdot(6);
	Eigen::VectorXd xddpsi(6);
	Eigen::VectorXd xrd(4);
	Eigen::VectorXd xpd(3);
	Eigen::MatrixXd lamdahat_t(3,4);
	Erplus.setZero (); 
	Eplus.setZero (); 
		
	// gains
	double kpj = 50;
	double kvj = 20;
	double kpx = 50;
	double kvx = 20;


	while (fSimulationRunning) { //automatically set to false when simulation is quit
		fTimerDidSleep = timer.waitForNextLoop();

		// update time
		double curr_time = timer.elapsedTime();
		double loop_dt = curr_time - last_time;

		// read joint positions, velocities
		sim->getJointPositions(robot_name, robot->_q);
		sim->getJointVelocities(robot_name, robot->_dq);
		robot->updateModel();
		// constants
		
		double T = 5.0; //sec

		// -------------------------------------------
		// -------------------------------------------
		// FILL ME IN: set joint torques for simulation
		auto t=curr_time;

				Eigen::Vector3d ee_pos_d(3);
		ee_pos_d(0)=0; //x
		ee_pos_d(1)=0.5 + 0.1 * cos(2*M_PI*t/5);//yddq_d
		ee_pos_d(2)=0.65 -0.05 * cos(4*M_PI*t/5);//z

		//desired orientation in euler parameters
		Eigen::VectorXd lb(4); //lambdas
		double lamda_0d=1/sqrt(2)*sin(M_PI/4*cos((2*M_PI*t)/5));//lambda_0d
		double lamda_1d=1/sqrt(2)*cos(M_PI/4*cos((2*M_PI*t)/5));//lambda_1d
		double lamda_2d=1/sqrt(2)*sin(M_PI/4*cos((2*M_PI*t)/5));//lambda_2d
		double lamda_3d=1/sqrt(2)*cos(M_PI/4*cos((2*M_PI*t)/5));//lambda_1d

		//desired operational space velocity (cartesian, quaternion)
	
		Eigen::VectorXd opspace_vel_d(3); // operational space velocity
		opspace_vel_d(0)= 0;//dx
		opspace_vel_d(1)= -0.04*M_PI*sin((2*M_PI*t)/5);//dy
		opspace_vel_d(2)= 0.04*M_PI*sin((4*M_PI*t)/5);//dz
		double lamda_0ddot= -0.05*sqrt(2)*pow(M_PI,2)*cos(M_PI/4*cos((2*M_PI*t)/5))*sin((2*M_PI*t)/5);//d lambda_0d
		double lamda_1ddot= 0.05*sqrt(2)*pow(M_PI,2)*sin(M_PI/4*cos((2*M_PI*t)/5))*sin((2*M_PI*t)/5);//d lambda_1d
		double lamda_2ddot= -0.05*sqrt(2)*pow(M_PI,2)*cos(M_PI/4*cos((2*M_PI*t)/5))*sin((2*M_PI*t)/5);//d lambda_2d
		double lamda_3ddot= 0.05*sqrt(2)*pow(M_PI,2)*sin(M_PI/4*cos((2*M_PI*t)/5))*sin((2*M_PI*t)/5);//d lambda_3d

		//operational desired acceleration
		Eigen::VectorXd opspace_acc_d(3); // operational space velocity
		opspace_acc_d(0)=0;//ddx
		opspace_acc_d(1)=-0.016*pow(M_PI,2)*cos(2*M_PI*t/5);//ddy
		opspace_acc_d(2)=0.032*pow(M_PI,2)*cos(4*M_PI*t/5);//ddz
		double lamda_0ddotdt=-0.005*sqrt(2)*pow(M_PI,4)*sin(M_PI/4*cos(2*M_PI*t/5))*pow(sin(2*M_PI*t/5),2)-0.02*sqrt(2)*pow(M_PI,3)*cos(M_PI/4*cos(2*M_PI*t/5))*cos(2*M_PI*t/5);//ddlambda_0
		double lamda_1ddotdt=-0.005*sqrt(2)*pow(M_PI,4)*cos(M_PI/4*cos(2*M_PI*t/5))*pow(sin(2*M_PI*t/5),2)+0.02*sqrt(2)*pow(M_PI,3)*sin(M_PI/4*cos(2*M_PI*t/5))*cos(2*M_PI*t/5);//ddlambda_1
		double lamda_2ddotdt=-0.005*sqrt(2)*pow(M_PI,4)*sin(M_PI/4*cos(2*M_PI*t/5))*pow(sin(2*M_PI*t/5),2)-0.02*sqrt(2)*pow(M_PI,3)*cos(M_PI/4*cos(2*M_PI*t/5))*cos(2*M_PI*t/5);//ddlambda_2
		double lamda_3ddotdt=-0.005*sqrt(2)*pow(M_PI,4)*cos(M_PI/4*cos(2*M_PI*t/5))*pow(sin(2*M_PI*t/5),2)+0.02*sqrt(2)*pow(M_PI,3)*sin(M_PI/4*cos(2*M_PI*t/5))*cos(2*M_PI*t/5);//ddlambda_3
		
		// Obtain the basic Jacobian at the end-effector, J_0 = [Jv' Jw']'
		
		// double lamda_0d = (1/sqrt(2))*sin((M_PI/4)*cos((2*M_PI*curr_time)/5));
		// double lamda_1d = (1/sqrt(2))*cos((M_PI/4)*cos((2*M_PI*curr_time)/5));	
		// double lamda_2d = (1/sqrt(2))*sin((M_PI/4)*cos((2*M_PI*curr_time)/5));
		// double lamda_3d = (1/sqrt(2))*cos((M_PI/4)*cos((2*M_PI*curr_time)/5));	
		
		xpd =ee_pos_d;
		xrd << lamda_0d,lamda_1d,lamda_2d,lamda_3d;
		
		// desired angular velocities
		// double lamda_0ddot = -(pow(M_PI,2)/(10*sqrt(2)))*cos((M_PI/4)*cos((2*M_PI*curr_time)/5))*sin((2*M_PI*curr_time)/5);
		// double lamda_1ddot = (pow(M_PI,2)/(10*sqrt(2)))*sin((M_PI/4)*cos((2*M_PI*curr_time)/5))*sin((2*M_PI*curr_time)/5);	
		// double lamda_2ddot = -(pow(M_PI,2)/(10*sqrt(2)))*cos((M_PI/4)*cos((2*M_PI*curr_time)/5))*sin((2*M_PI*curr_time)/5);
		// double lamda_3ddot = (pow(M_PI,2)/(10*sqrt(2)))*sin((M_PI/4)*cos((2*M_PI*curr_time)/5))*sin((2*M_PI*curr_time)/5);		
		
		// desired linear accelerations
		double dxdt = 0;
		double dydt = -2.0*pow(M_PI,2)/125*cos(2.0*M_PI*curr_time/T);
		double dzdt = 4.0*pow(M_PI, 2)/125*cos(4.0*M_PI*curr_time/T);

		// // desired angular accelerations
		// double lamda_0ddotdt = -pow(M_PI, 4)/(4*10*pow(T,2)*sqrt(2.0))*sin(M_PI/4.0*cos(2.0*M_PI*curr_time/T))*pow(sin(2.0*M_PI*curr_time/T),2) - pow(M_PI,3)/(pow(T,2)*sqrt(2.0))*cos(M_PI/4*cos(2*M_PI*curr_time/T))*cos(2*M_PI*curr_time/T);
		// double lamda_1ddotdt =  pow(M_PI, 4)/(4*10*pow(T,2)*sqrt(2.0))*cos(M_PI/4.0*sin(2.0*M_PI*curr_time/T))*pow(sin(2.0*M_PI*curr_time/T),2) + pow(M_PI,3)/(pow(T,2)*sqrt(2.0))*sin(M_PI/4*cos(2*M_PI*curr_time/T))*cos(2*M_PI*curr_time/T);
		// double lamda_2ddotdt = -pow(M_PI, 4)/(4*10*pow(T,2)*sqrt(2.0))*sin(M_PI/4.0*cos(2.0*M_PI*curr_time/T))*pow(sin(2.0*M_PI*curr_time/T),2) - pow(M_PI,3)/(pow(T,2)*sqrt(2.0))*cos(M_PI/4*cos(2*M_PI*curr_time/T))*cos(2*M_PI*curr_time/T);
		// double lamda_3ddotdt =  pow(M_PI, 4)/(4*10*pow(T,2)*sqrt(2.0))*cos(M_PI/4.0*sin(2.0*M_PI*curr_time/T))*pow(sin(2.0*M_PI*curr_time/T),2) + pow(M_PI,3)/(pow(T,2)*sqrt(2.0))*sin(M_PI/4*cos(2*M_PI*curr_time/T))*cos(2*M_PI*curr_time/T);

		
		//Erplus and Eplus
		// Erplus << -2*lamda_1d, 2*lamda_0d, -2*lamda_3d,2*lamda_2d,
		// -2*lamda_2d, 2*lamda_3d, 2*lamda_0d, -2*lamda_1d,
		// -2*lamda_3d, -2*lamda_2d, 2*lamda_1d, 2*lamda_0d;
		
		// Eplus << MatrixXd::Identity(3,3),MatrixXd::Zero(3,4),
		// 		MatrixXd::Zero(3,3), Erplus;

				Eigen::MatrixXd clambda_t(3,4);
				clambda_t<<-lb(1), lb(0), -lb(3), lb(2), 
				   -lb(2),lb(3),lb(0),-lb(1),
				   -lb(3),-lb(2),lb(1),lb(0);
		
		//define matrix Er+
		auto Erplus=2*clambda_t;
	

		v0d << opspace_vel_d; //xddot
		w0d << lamda_0ddot, lamda_1ddot,lamda_2ddot, lamda_3ddot;  //wd
		
		dv0d =opspace_acc_d;
		dw0d << lamda_0ddotdt, lamda_1ddotdt,lamda_2ddotdt, lamda_3ddotdt; //wddot
		
		Eigen::Vector3d ee_pos_local;
		ee_pos_local << 0.0, 0.0, 0.0;
		robot-> position(ee_pos,ee_link_name,ee_pos_local);	//end effector tip position
		robot-> linearVelocity(v0,ee_link_name,ee_pos_local); //xp
		robot->angularVelocity(w0,ee_link_name,ee_pos_local); //w
		robot->J_0(J0, ee_link_name, ee_pos_local); //J0
		robot->gravityVector(g);
		lamdahat_t<< -ee_rot_lambda.x(),  ee_rot_lambda.w(), - ee_rot_lambda.z(),  ee_rot_lambda.y(),
					 -ee_rot_lambda.y(),  ee_rot_lambda.z(),   ee_rot_lambda.w(), -ee_rot_lambda.x(),
					 -ee_rot_lambda.z(), -ee_rot_lambda.y(),   ee_rot_lambda.x(),  ee_rot_lambda.w();
		
		// Eigen::Vector3d ee_error = -2*(lamdahat_t*xrd); //instantaneuos angular error
				//get transformation matrix for end effector
		Eigen::Affine3d _T;
		Eigen::Matrix3d rot_in_link = Matrix3d::Identity();
		robot->transform(_T,ee_link_name, ee_pos_local,rot_in_link);
		//get current orientation of end effector
		//Eigen::Quaterniond lb_c= (Eigen::Quaterniond)_T.rotation();
		robot->rotation(ee_rot_mat, ee_link_name,rot_in_link); // initialize
		Eigen::Quaterniond ee_rot_lambda=(Eigen::Quaterniond)ee_rot_mat; // end effector rotation in quaternion form
		Eigen::Quaterniond lb_c= (Eigen::Quaterniond)ee_rot_mat;
		lb_c.normalize();//normalize quaternion
		
		//define matrix check{\lambda}.transpose for current position
		Eigen::MatrixXd current_clambda_t(3,4);
		current_clambda_t<<-lb_c.x(), lb_c.w(), -lb_c.z(), lb_c.y(), 
				   -lb_c.y(),lb_c.z(),lb_c.w(),-lb_c.x(),
				   -lb_c.z(),-lb_c.y(),lb_c.x(),lb_c.w();

		//desired orientation in angle error
		Eigen::Vector3d ee_error;
		ee_error=-2*current_clambda_t*xrd;

		//wd = Erplus*{lamda_0d,lamda_1d,lamda_2d,lamda_3d}*{lamda_0ddot,lamda_1ddot,lamda_2ddot,lamda_3ddot};
		wd = Erplus*w0d;
		wddot = Erplus*dw0d;
		//first matrix in tau
		xddpsi = ee_pos-xpd,ee_error;
		//2nd matrix in tau
		vw << v0-v0d,
			  w0-wd;
		
		//3rd matrix in tau
		xddot << v0d,
				wd;
		
		
		//4th matrix in tau
		xddotdot << dv0d,
					wddot;
		
		L0 = (J0 * robot->_M_inv * J0.transpose()).inverse(); //L0
		p = L0*J0*robot->_M_inv*g;
		Jbar = robot->_M_inv * J0.transpose() * (J0 * robot->_M_inv * J0.transpose()).inverse();
		Nbar = In - J0.transpose()*Jbar.transpose();


		//--CONTROL SIGNALS
		//-------------------------------------
		//decoupled force
		Eigen::VectorXd F_star;
		F_star=dv0d-1*kpx*(ee_pos-xpd)-kvx*(v0-v0d);

		//decoupled moment
		Eigen::VectorXd M_star;
		M_star=wddot-kpx*ee_error-kvx*(w0-wd);

		//total decoupled
		Eigen::VectorXd V_star(6);
		V_star<<F_star,M_star;


		//Nbar = In - Jbar.transpose()*J0;
		auto _A=robot->_M;
		//tau = J0.transpose()*(L0*(xddotdot-kpx*xddpsi-kvx*(vw)))+g+ Nbar*((robot->_M) *(-kvj*(robot->_dq))+g);
		tau=J0.transpose()*(L0*V_star+p) + (In- J0.transpose()*Jbar.transpose())*(_A*(-kvj*(robot->_dq))+g);

		sim->setJointTorques(robot_name, tau);
		// -------------------------------------------
		// -------------------------------------------

		// update last time
		last_time = curr_time;
	}
}


/* =======================================================================================
   SIMULATION SETUP
   -----------------------
   * Simulation loop
   * Window initialization
   * Window error
   * Mouse click commands
========================================================================================== */
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim) {
	fSimulationRunning = true;

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); //1000Hz timer
	double last_time = timer.elapsedTime(); //secs

	bool fTimerDidSleep = true;
	while (fSimulationRunning) {
		fTimerDidSleep = timer.waitForNextLoop();

		// integrate forward
		double curr_time = timer.elapsedTime();
		double loop_dt = curr_time - last_time; 
		sim->integrate(loop_dt);

		// if (!fTimerDidSleep) {
		// 	cout << "Warning: timer underflow! dt: " << loop_dt << "\n";
		// }

		// update last time
		last_time = curr_time;
	}
}

//------------------------------------------------------------------------------
GLFWwindow* glfwInitialize() {
	
	/*------- Set up visualization -------*/
    // set up error callback
    glfwSetErrorCallback(glfwError);

    // initialize GLFW
    glfwInit();

    // retrieve resolution of computer display and position window accordingly
    GLFWmonitor* primary = glfwGetPrimaryMonitor();
    const GLFWvidmode* mode = glfwGetVideoMode(primary);

    // information about computer screen and GLUT display window
	int screenW = mode->width;
    int screenH = mode->height;
    int windowW = 0.8 * screenH;
    int windowH = 0.5 * screenH;
    int windowPosY = (screenH - windowH) / 2;
    int windowPosX = windowPosY;

    // create window and make it current
    glfwWindowHint(GLFW_VISIBLE, 0);
    GLFWwindow* window = glfwCreateWindow(windowW, windowH, "SAI2.0 - CS327a HW2", NULL, NULL);
	glfwSetWindowPos(window, windowPosX, windowPosY);
	glfwShowWindow(window);
    glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	return window;
}

//------------------------------------------------------------------------------

void glfwError(int error, const char* description) {
	cerr << "GLFW Error: " << description << endl;
	exit(1);
}

//------------------------------------------------------------------------------

void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    // option ESC: exit
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
    {
        // exit application
         glfwSetWindowShouldClose(window, 1);
    }
}
