/*======================================================================================
 * hw2-p1-main-sol.cpp
 *
 * Implement three variations of a position controller for a 6-DOF Puma 
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

#include <GLFW/glfw3.h> 

using namespace std;

const string world_fname = "resources/hw2_sol/world_1_puma.urdf";
const string robot_fname = "../resources/puma/puma.urdf";
const string robot_name = "Puma";
// const string camera_name = "camera_front";
const string camera_name = "camera_top";
const string ee_link_name = "end-effector";

// problem part selection
enum PROBLEM_PART_TYPE {
	PART1=0,
	PART2,
	PART3,
	N_PARTS
};
PROBLEM_PART_TYPE enum_problem_part;
void selectProblemPart(char* input);

/* --------------------------------------------------------------------------------------
	Simulation Loop Initialization
-------------------------------------------------------------------------------------*/
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
	
	// get problem part
	if (argc < 2) {
		cout << "Usage: ./hw2-p1-sol <part: 1 or 2 or 3>" << endl;
		return 0;
	}
	selectProblemPart(argv[1]);

	cout << "Loading URDF world model file: " << world_fname << endl;

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_fname, false);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_fname, false);

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_fname, false);

	// set initial condition
	robot->_q << 90/180.0*M_PI,
				-22.5/180.0*M_PI,
				180/180.0*M_PI,
				90.0/180.0*M_PI,
				100/180.0*M_PI,
				180/180.0*M_PI;
	sim->setJointPositions(robot_name, robot->_q);
	robot->updateModel();

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

	// end effector tip position in link frame
	Eigen::Vector3d ee_pos_local;
	ee_pos_local << -0.2, 0.0, 0.0;

	// end effector desired position
	Eigen::Vector3d ee_des_pos;
	robot->updateModel();
	robot->position(ee_des_pos, ee_link_name, ee_pos_local);

	// gains
	double kpx = 50.0; // operational space kp
	double kvx = 20.0; // operational space kv
	double kpj = 50.0; // joint space kp
	double kvj = 20.0; // joint space kv

	// ---------------------------------------------------------------------------------
	// Suggested cache variables
	//----------------------------------------------------------------------------------
	Eigen::VectorXd g(robot->dof()); //joint space gravity vector
	Eigen::MatrixXd Jv(3, robot->dof()); //end effector linear velocity Jacobian
	Eigen::MatrixXd Lv(3, 3); //Lambda_v at end effector
	Eigen::Vector3d p; //gravity vector at end-effector corresponding to linear motion
	Eigen::MatrixXd Jpseudo(robot->dof(), 3); //pseudo inverse of Jv
	const Eigen::MatrixXd In = Eigen::MatrixXd::Identity(robot->dof(), robot->dof()); // n x n identity matrix
	Eigen::MatrixXd Npseudo(robot->dof(), robot->dof()); //I - Jpseudo * Jv, null space projection matrix for pseudo-inverse
	Eigen::MatrixXd Jbar(robot->dof(), 3);	//A-weighted generalized inverse of Jv
	Eigen::MatrixXd Nbar(robot->dof(), robot->dof()); //I - Jbar*Jv, null space projection matrix for Jbar
	Eigen::Vector3d ee_pos; //end effector position
	Eigen::Vector3d v; //end effector velocity
	Eigen::VectorXd qd(robot->dof()); //desired joint positions
	
	while (fSimulationRunning) { //automatically set to false when simulation is quit
		fTimerDidSleep = timer.waitForNextLoop();

		// update time
		double curr_time = timer.elapsedTime();
		double loop_dt = curr_time - last_time;

		// read joint positions, velocities, update model
		sim->getJointPositions(robot_name, robot->_q);
		sim->getJointVelocities(robot_name, robot->_dq);
		robot->updateModel();

		/* ------------------------------------------------------------------------------------
			FILL ME IN: set joint torques
		-------------------------------------------------------------------------------------*/

		// Update model parameters
		robot->gravityVector(g);
		robot->position(ee_pos, ee_link_name, ee_pos_local);
		robot->Jv(Jv, ee_link_name, ee_pos_local);
		v = Jv*robot->_dq;
		Lv = (Jv * robot->_M_inv * Jv.transpose()).inverse();
		Jbar = robot->_M_inv * Jv.transpose() * Lv;
		Nbar = In - Jbar*Jv;
		p = Jbar.transpose()*g;

		switch (enum_problem_part) {
			
			// (1) Pure null space damping  
			//----------------------------------------------------
			case PART1:
				tau = Jv.transpose()*(Lv*(-kpx*(ee_pos - ee_des_pos) -kvx*(v)) + p) + // op. space trajectory tracking 
					  robot->_M*(-kvj*robot->_dq); // joint space joint damping
				break;

			// (2) Pseudo-inverse decoupling (p.88-89 course reader)
			//----------------------------------------------------
			case PART2:
				// null space motion
				qd = robot->_q;
				qd[1] = -M_PI/8.0 + M_PI/8.0*sin(2.0*M_PI*curr_time/10.0);
				// pseudo inverse and associated null space
				Jpseudo = Jv.transpose()*(Jv*Jv.transpose()).inverse();
				Npseudo = (In - Jpseudo*Jv);
				// control torques 
				tau = Jv.transpose()*(Lv*(-kpx*(ee_pos - ee_des_pos) -kvx*(v)) + p) +  // op. space trajectory tracking 
					  Npseudo.transpose()*(robot->_M*(-kpj*(robot->_q - qd) -kvj*robot->_dq) + g); // null space joint trajectory 
					  																			   // tracking and joint damping
				break;

			// (3) Null space dynamically consistent decoupling (p.88-89 course reader)
			//----------------------------------------------------
			case PART3:
				// null space motion
				qd = robot->_q;
				qd[1] = -M_PI/8.0 + M_PI/8.0*sin(2.0*M_PI*curr_time/10.0);
				// control torques using dynamically consistent inverse 
				tau = Jv.transpose()*(Lv*(-kpx*(ee_pos - ee_des_pos) -kvx*(v)) + p) + 
					  Nbar.transpose()*(robot->_M*(-kpj*(robot->_q - qd) -kvj*robot->_dq) + g);
				break;

			default:
				tau.setZero();
				break;
		}

		sim->setJointTorques(robot_name, tau);
		// -------------------------------------------

		// update last time
		last_time = curr_time;
	}
}


/* =======================================================================================
   SIMULATION SETUP
   -----------------------
   * Simulation loop
   * Select problem part
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

		// update last time
		last_time = curr_time;
	}
}

//------------------------------------------------------------------------------
void selectProblemPart(char* input) {
	switch (input[0]) {
		case '1':
			enum_problem_part = PART1;
			break;
		case '2':
			enum_problem_part = PART2;
			break;
		case '3':
			enum_problem_part = PART3;
			break;
		default:
			cout << "Usage: ./hw2-p1-sol <part: 1 or 2 or 3>" << endl;
			exit(0);
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
    GLFWwindow* window = glfwCreateWindow(windowW, windowH, "SAI2.0 - CS327a HW2 - SOLUTION", NULL, NULL);
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
