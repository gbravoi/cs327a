/*======================================================================================
 * hw1-main.cpp
 *
 * Implement an algorithm to have a 7-DOF kuka track a desired trajectory
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

const string world_fname = "resources/hw1/world.urdf";
const string robot_fname = "../resources/kuka_iiwa/kuka_iiwa.urdf";
const string robot_name = "Kuka-IIWA";
const string camera_name = "camera_front";
// const string camera_name = "camera_top";
const string ee_link_name = "link6";

/* --------------------------------------------------------------------------------------
	Simulation Loop Initialization
-------------------------------------------------------------------------------------*/

// simulation loop
bool fSimulationRunning = false;
void simulation(Sai2Model::Sai2Model* robot);

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

	// set initial condition
	robot->_q << 125.9/180.0*M_PI,
				39.2/180.0*M_PI,
				-49.2/180.0*M_PI,
				70.0/180.0*M_PI,
				-62.4/180.0*M_PI,
				80.2/180.0*M_PI,
				187.2/180.0*M_PI;
	robot->updateModel();

	// initialize GLFW window
	GLFWwindow* window = glfwInitialize();

    // set callbacks
	glfwSetKeyCallback(window, keySelect);

	// start the simulation
	thread sim_thread(simulation, robot);
	
    // while window is open:
    while (!glfwWindowShouldClose(window)) {

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

    // destroy context
    glfwDestroyWindow(window);

    // terminate
    glfwTerminate();

	return 0;
}

/* =======================================================================================
   SIMULATION SETUP
   -----------------------
   * Simulation loop
   * Window initialization
   * Window error
   * Mouse click commands
========================================================================================== */

void simulation(Sai2Model::Sai2Model* robot) {  //--- Simulation loop ---//
	fSimulationRunning = true;

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(500); //500Hz timer
	double last_time = timer.elapsedTime(); //secs

	Eigen::MatrixXd Jbar;
	Eigen::MatrixXd Jv;
	Eigen::MatrixXd Jw;
	Eigen::MatrixXd J0;
	Eigen::VectorXd opspace_vel(6); // operational space velocity

	bool fTimerDidSleep = true;

	while (fSimulationRunning) {
		fTimerDidSleep = timer.waitForNextLoop();

		// integrate joint velocity to joint positions
		double curr_time = timer.elapsedTime();
		double loop_dt = curr_time - last_time; 
		robot->_q += robot->_dq*loop_dt;

		// update kinematic models
		robot->updateModel();

		/* --------------------------------------------------------------------------------------
			FILL ME IN: set new joint velocities
		-------------------------------------------------------------------------------------*/
		double t=curr_time; //local variable to make easy call current time

		//evaluate xd
		//position cartesian (values not used)
		//orientation in euler parameters
		Eigen::VectorXd lb(4); //lambdas
		lb(0)=1/sqrt(2)*sin(M_PI/4*cos((2*M_PI*t)/5));//lambda_0d
		lb(1)=1/sqrt(2)*cos(M_PI/4*cos((2*M_PI*t)/5));//lambda_1d
		lb(2)=1/sqrt(2)*sin(M_PI/4*cos((2*M_PI*t)/5));//lambda_2d
		lb(3)=1/sqrt(2)*cos(M_PI/4*cos((2*M_PI*t)/5));//lambda_1d

		//evaluate operational space velocity
		Eigen::VectorXd opspace_vel(7); // operational space velocity
		opspace_vel(0)= 0;//dx
		opspace_vel(1)= -0.04*M_PI*sin((2*M_PI*t)/5);//dy
		opspace_vel(2)= 0.04*M_PI*sin((4*M_PI*t)/5);//dz
		opspace_vel(3)= -0.05*sqrt(2)*pow(M_PI,2)*cos(M_PI/4*cos((2*M_PI*t)/5))*sin((2*M_PI*t)/5);//d lambda_0d
		opspace_vel(4)= 0.05*sqrt(2)*pow(M_PI,2)*sin(M_PI/4*cos((2*M_PI*t)/5))*sin((2*M_PI*t)/5);//d lambda_1d
		opspace_vel(5)= -0.05*sqrt(2)*pow(M_PI,2)*cos(M_PI/4*cos((2*M_PI*t)/5))*sin((2*M_PI*t)/5);//d lambda_2d
		opspace_vel(6)= 0.05*sqrt(2)*pow(M_PI,2)*sin(M_PI/4*cos((2*M_PI*t)/5))*sin((2*M_PI*t)/5);//d lambda_3d


		//define matrix E+
		Eigen::MatrixXd Eplus(6,7);
		//make all values 0
		Eplus.setZero();
		
		//fill Ep+
		Eplus(0,0)=1;
		Eplus(1,1)=1;
		Eplus(2,2)=1;
		
		//fillEr+
		Eplus(3,3)=-2*lb(1);
		Eplus(3,4)=2*lb(0);
		Eplus(3,5)=-2*lb(3);
		Eplus(3,6)=2*lb(2);

		Eplus(4,3)=-2*lb(2);
		Eplus(4,4)=2*lb(3);
		Eplus(4,5)=2*lb(0);
		Eplus(4,6)=-2*lb(1);

		Eplus(5,3)=-2*lb(3);
		Eplus(5,4)=-2*lb(2);
		Eplus(5,5)=2*lb(1);
		Eplus(5,6)=2*lb(0);

		//linear velocity and angular velocity
		Eigen::VectorXd vel(6);
		vel=Eplus*opspace_vel;

		//get J0 of the robot. note J0=[Jv; Jw]
		//define pos_in_link
		auto pos_in_link=Eigen::Vector3d::Zero();//values described in homework instructions
		robot->J_0(J0,ee_link_name,pos_in_link); //linkname= the end efector

		//Jacobian pseudo inverse
		//define local variables for readability
		auto _A_inv=robot->_M_inv;
		auto _A=robot->_M;
		auto _Lambda=(J0*_A_inv*J0.transpose()).inverse();
		Jbar=_A_inv*J0.transpose()*_Lambda;

		//save value of dq
		robot->_dq=Jbar*vel;

		

		// update last time
		last_time = curr_time;
	}
}

//---------------------------------	---------------------------------------------
GLFWwindow* glfwInitialize() { //--- Window initialization ---//
	
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
    GLFWwindow* window = glfwCreateWindow(windowW, windowH, "SAI2.0 - CS327a HW1", NULL, NULL);
	glfwSetWindowPos(window, windowPosX, windowPosY);
	glfwShowWindow(window);
    glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	return window;
}

//------------------------------------------------------------------------------

void glfwError(int error, const char* description) {  //--- Window error ---// 
	cerr << "GLFW Error: " << description << endl;
	exit(1);
}

//------------------------------------------------------------------------------

void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods) //--- Mouse click commands ---//
{
    // option ESC: exit
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
    {
        // exit application
         glfwSetWindowShouldClose(window, 1);
    }
}
