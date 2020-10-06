/*======================================================================================
 * hw2-p3-main.cpp
 *
 * Implement motion-force control for a 7-DOF redundant kuka 
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

#include "force_sensor/ForceSensorSim.h"
#include "force_sensor/ForceSensorDisplay.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew as part of graphicsinterface

using namespace std;

const string world_fname = "resources/hw2/world_3_iiwa.urdf";
const string robot_fname = "../resources/kuka_iiwa/kuka_iiwa.urdf";
const string robot_name = "Kuka-IIWA";
const string camera_name = "camera_back_side";
const string ee_link_name = "link6";

// const string camera_name = "camera_front";
// const string camera_name = "camera_top";
// const string camera_name = "camera_back_top";

/* --------------------------------------------------------------------------------------
	Simulation Loop Initialization
-------------------------------------------------------------------------------------*/

// simulation loop
bool fSimulationRunning = false;
void control(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim);
void simulation(Sai2Model::Sai2Model* robot, Simulation::Sai2Simulation* sim);

// force sensor
ForceSensorSim* force_sensor;
// display widget for forces at end effector
ForceSensorDisplay* force_display;

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
    
    // set co-efficient of restition to zero for force control
    // see issue: https://github.com/manips-sai/sai2-simulation/issues/1
    sim->setCollisionRestitution(0.0);
    // set co-efficient of friction also to zero for now as this causes jitter
    sim->setCoeffFrictionStatic(0.0);
    sim->setCoeffFrictionDynamic(0.0);

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

	// initialize force sensor: needs Sai2Simulation sim interface type
	force_sensor = new ForceSensorSim(robot_name, ee_link_name, Eigen::Affine3d::Identity(), sim, robot);
	force_display = new ForceSensorDisplay(force_sensor, graphics);

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
		force_display->update();
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

	// sensed forces and moments from sensor
	Eigen::Vector3d sensed_force;
    Eigen::Vector3d sensed_moment;

    // additional cache variables
	Eigen::VectorXd g(robot->dof()); //joint space gravity vector
	Eigen::MatrixXd J0(6, robot->dof()); //end effector basic Jacobian
	Eigen::MatrixXd L0(6, 6); //Lambda_0 at end effector
	Eigen::VectorXd p(6); //gravity vector at end-effector
	const Eigen::MatrixXd In = Eigen::MatrixXd::Identity(robot->dof(), robot->dof()); // n x n identity matrix
	Eigen::MatrixXd Jbar(robot->dof(), 6);	//A-weighted generalized inverse of J0
	Eigen::MatrixXd Nbar(robot->dof(), robot->dof()); //I - Jbar*J0, null space projection matrix for Jbar
	Eigen::Vector3d ee_pos; //end effector position
	Eigen::Matrix3d ee_rot_mat; //end effector rotation
	robot->rotation(ee_rot_mat, ee_link_name); // initialize
	Eigen::Quaterniond ee_rot_lambda(ee_rot_mat); // end effector rotation in quaternion form
	Eigen::VectorXd ee_error(6); //end effector operational space instantaneous error
	Eigen::VectorXd v0(6); //end effector velocity
	Eigen::VectorXd v0d(6); //end effector desired velocity
	Eigen::VectorXd dv0d(6); //end effector desired acceleration
	Eigen::MatrixXd select_motion(6,6); // selection matrix for motion, Omega
	Eigen::MatrixXd select_forces(6,6); // selection matrix for forces, Omega_bar
	Eigen::VectorXd Fd(6); // desired end effector force

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

		// update force sensor readings
		force_sensor->update();
		force_sensor->getForce(sensed_force);
        force_sensor->getMoment(sensed_moment);

		// -------------------------------------------
		// -------------------------------------------
		// FILL ME IN: set joint torques for simulation
		auto t=curr_time;

		//---END EFFECTOR DESIRED POSITIONS
		//-------------------------------------
		//end effector desired position in cartesian
		Eigen::Vector3d ee_pos_d(3);
		ee_pos_d(0)=0; //x
		ee_pos_d(1)=0.5 + 0.1 * cos(2*M_PI*t/5);//yddq_d
		ee_pos_d(2)=0.65 -0.05 * cos(4*M_PI*t/5);//z

		//desired orientation in euler parameters
		Eigen::VectorXd lb(4); //lambdas
		lb(0)=1/sqrt(2)*sin(M_PI/4*cos((2*M_PI*t)/5));//lambda_0d
		lb(1)=1/sqrt(2)*cos(M_PI/4*cos((2*M_PI*t)/5));//lambda_1d
		lb(2)=1/sqrt(2)*sin(M_PI/4*cos((2*M_PI*t)/5));//lambda_2d
		lb(3)=1/sqrt(2)*cos(M_PI/4*cos((2*M_PI*t)/5));//lambda_1d

		//desired operational space velocity (cartesian, quaternion)
		Eigen::VectorXd opspace_vel_d(7); // operational space velocity
		opspace_vel_d(0)= 0;//dx
		opspace_vel_d(1)= -0.04*M_PI*sin((2*M_PI*t)/5);//dy
		opspace_vel_d(2)= 0.04*M_PI*sin((4*M_PI*t)/5);//dz
		opspace_vel_d(3)= -0.05*sqrt(2)*pow(M_PI,2)*cos(M_PI/4*cos((2*M_PI*t)/5))*sin((2*M_PI*t)/5);//d lambda_0d
		opspace_vel_d(4)= 0.05*sqrt(2)*pow(M_PI,2)*sin(M_PI/4*cos((2*M_PI*t)/5))*sin((2*M_PI*t)/5);//d lambda_1d
		opspace_vel_d(5)= -0.05*sqrt(2)*pow(M_PI,2)*cos(M_PI/4*cos((2*M_PI*t)/5))*sin((2*M_PI*t)/5);//d lambda_2d
		opspace_vel_d(6)= 0.05*sqrt(2)*pow(M_PI,2)*sin(M_PI/4*cos((2*M_PI*t)/5))*sin((2*M_PI*t)/5);//d lambda_3d

		//operational desired acceleration
		Eigen::VectorXd opspace_acc_d(7); // operational space velocity
		opspace_acc_d(0)=0;//ddx
		opspace_acc_d(1)=-0.016*pow(M_PI,2)*cos(2*M_PI*t/5);//ddy
		opspace_acc_d(2)=0.032*pow(M_PI,2)*cos(4*M_PI*t/5);//ddz
		opspace_acc_d(3)=-0.005*sqrt(2)*pow(M_PI,4)*sin(M_PI/4*cos(2*M_PI*t/5))*pow(sin(2*M_PI*t/5),2)-0.02*sqrt(2)*pow(M_PI,3)*cos(M_PI/4*cos(2*M_PI*t/5))*cos(2*M_PI*t/5);//ddlambda_0
		opspace_acc_d(4)=-0.005*sqrt(2)*pow(M_PI,4)*cos(M_PI/4*cos(2*M_PI*t/5))*pow(sin(2*M_PI*t/5),2)+0.02*sqrt(2)*pow(M_PI,3)*sin(M_PI/4*cos(2*M_PI*t/5))*cos(2*M_PI*t/5);//ddlambda_1
		opspace_acc_d(5)=-0.005*sqrt(2)*pow(M_PI,4)*sin(M_PI/4*cos(2*M_PI*t/5))*pow(sin(2*M_PI*t/5),2)-0.02*sqrt(2)*pow(M_PI,3)*cos(M_PI/4*cos(2*M_PI*t/5))*cos(2*M_PI*t/5);//ddlambda_2
		opspace_acc_d(6)=-0.005*sqrt(2)*pow(M_PI,4)*cos(M_PI/4*cos(2*M_PI*t/5))*pow(sin(2*M_PI*t/5),2)+0.02*sqrt(2)*pow(M_PI,3)*sin(M_PI/4*cos(2*M_PI*t/5))*cos(2*M_PI*t/5);//ddlambda_3

		//---ROBOT PROPERTIES
		//-------------------------------------
		//mass matrix
		auto _A=robot->_M;
		auto _A_inv=robot->_M_inv;

		//get J0 of the robot. note J0=[Jv; Jw]
		Eigen::Vector3d ee_pos_local;
		ee_pos_local << 0.0, 0.0, 0.0;
		robot->J_0(J0,ee_link_name,ee_pos_local); //linkname= the end efector
		//dynamic inverse
		auto Jbar0=_A_inv*J0.transpose()*L0;

		//define Lambda0
		L0=(J0*_A_inv*J0.transpose()).inverse();

		//define matrix check{\lambda}.transpose
		Eigen::MatrixXd clambda_t(3,4);
		clambda_t<<-lb(1), lb(0), -lb(3), lb(2), 
				   -lb(2),lb(3),lb(0),-lb(1),
				   -lb(3),-lb(2),lb(1),lb(0);
		
		//define matrix Er+
		auto Erplus=2*clambda_t;

		//Eplus matrix
		Eigen::MatrixXd Eplus(6,7);
		Eplus<< MatrixXd::Identity(3,3), MatrixXd::Zero(3,4),
			MatrixXd::Zero(3,3), Erplus;

		//get gravity
		robot->gravityVector(g); //get gravity vector
		p = L0*J0*robot->_M_inv*g;


		//--GET END EFFECTOR CURRENT POSITION, VELOCITY 
		//-------------------------------------
		// end-effector tip position in base frame (Cartesian coordinates)
		robot->position(ee_pos, ee_link_name, ee_pos_local);

		//end-effector linear velocity in base frame
		Eigen::Vector3d ee_vx;
		robot->linearVelocity(ee_vx,ee_link_name, ee_pos_local);

		//end-effector angula velocity in base frame
		Eigen::Vector3d ee_omega;
		robot->angularVelocity(ee_omega,ee_link_name, ee_pos_local);

		//total velocity [vx,vy,vz,omegax,omegay,omegaz]
		v0<< ee_vx,ee_omega;

		//---END-EFFECTOR ANGLE ERROR AND DESIRED ANGULAR VELOCITY
		//-------------------------------------
		//get transformation matrix for end effector
		Eigen::Affine3d _T;
		Eigen::Matrix3d rot_in_link = Matrix3d::Identity();
		robot->transform(_T,ee_link_name, ee_pos_local,rot_in_link);
		//get current orientation of end effector
		Eigen::Quaterniond lb_c= (Eigen::Quaterniond)_T.rotation();
		lb_c.normalize();//normalize quaternion
		
		//define matrix check{\lambda}.transpose for current position
		Eigen::MatrixXd current_clambda_t(3,4);
		current_clambda_t<<-lb_c.x(), lb_c.w(), -lb_c.z(), lb_c.y(), 
				   -lb_c.y(),lb_c.z(),lb_c.w(),-lb_c.x(),
				   -lb_c.z(),-lb_c.y(),lb_c.x(),lb_c.w();

		//desired orientation in angle error
		Eigen::VectorXd theta_error;
		theta_error=-2*current_clambda_t*lb;

		
		//angular velocity desired
		Eigen::VectorXd omega_d;
		omega_d=Erplus*opspace_vel_d.tail(4); //opspace_vel_d last 4 element are dlambda

		//angular aceleration desired
		Eigen::VectorXd domega_d;
		domega_d=Erplus*opspace_acc_d.tail(4); //opspace_acc_d last 4 element are ddlambda

		//total desired velcity
		v0d<<opspace_vel_d.head(3),omega_d;


		//--OMEGA MATRICES
		//-------------------------------------
		Eigen::MatrixXd Omega(6,6);
		Omega << (Matrix3d() << 0,0,0,0,1,0,0,0,1).finished(),
     		MatrixXd::Zero(3,3),
     		MatrixXd::Zero(3,3),
			(Matrix3d() << 1,0,0,0,0,0,0, 0,0).finished();
     		//MatrixXd::Identity(3,3);
		
		auto Omega_bar=MatrixXd::Identity(6,6)-Omega;


		//--CONTROL SIGNALS
		//-------------------------------------
		//decoupled force_motion
		Eigen::VectorXd F_star;
		F_star=opspace_acc_d.head(3)-1*kpx*(ee_pos-ee_pos_d)-kvx*(ee_vx-opspace_vel_d.head(3));

		//decoupled moment_motion
		Eigen::VectorXd M_star;
		M_star=domega_d-kpx*theta_error-kvx*(ee_omega-omega_d);

		//total decoupled_motion
		Eigen::VectorXd V_star(6);
		V_star<<F_star,M_star;

		//decoupled active force
		Eigen::VectorXd Fd(6);//desired force
		Fd<< 10,0,0,0,0,0;
		auto F_star_f=-kvx*(v0-v0d);

		//----TAU
		//-------------------------------------
		tau=J0.transpose()*(L0*Omega*V_star+p) + (In- J0.transpose()*Jbar0.transpose())*(_A*(-kvj*(robot->_dq))+g)+J0.transpose()*(Omega_bar*Fd+L0*Omega_bar*F_star_f);
		
		
		sim->setJointTorques(robot_name, tau);

		//print contact forces
		auto F_contact=J0*tau;
		cout << t << ", " <<sensed_force(0) << "\n\r";
		//cout <<ee_pos << "\n\r\n\r";
		
		//cout << t <<","<< F_contact(0) << "\n\r";
		//cout << F_contact << "\n\r\n\r";

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
