/*======================================================================================
 * hw4 - p1-main-sol.cpp
 *
 * Implement cooperative manipulation
 * by completing the "FILL ME IN" section 
 *
 *
 * Elena Galbally, Spring 2019 
 * Wesley Guo, Spring 2020
 *======================================================================================*/

/* --------------------------------------------------------------------------------------
   Include Required Libraries and Files
-----------------------------------------------------------------------------------------*/
#include <iostream>
#include <string>
#include <csignal>
#include <thread>
#include <chrono>
#include <math.h>

#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "Sai2Simulation.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <dynamics3d.h>
#include "timer/LoopTimer.h"
#include <GLFW/glfw3.h> 

#include "keys.h"

using namespace std;

const double time_slowdown_factor = 10;

constexpr const char *sim_title = "SAI2.0 - CS327a HW4 P1 Solution";
const string world_fname = "resources/hw4/world_1_puma.urdf";
const string robot_fname = "../resources/puma/puma_gripper.urdf";
const string robot1_name = "Puma1";
const string robot2_name = "Puma2";
const string object_name = "CoordObject";
const string object_fname = "resources/hw4/object.urdf";
const string object_link_name = "object";
const string camera_name = "camera_front";
const string ee_link_name = "end-effector";
const string gripper_joint_name = "gripper";

RedisClient redis_client;
/* ----------------------------------------------------------------------------------
	Simulation and Control Loop Setup
-------------------------------------------------------------------------------------*/
// state machine setup
enum ControlMode {
	CONTROL_GRASP_STABILIZE = 0,
	CONTROL_AUGMENTED_OBJECT
};

// simulation loop
bool fSimulationRunning = false;
void control(Sai2Model::Sai2Model* robot1, Sai2Model::Sai2Model* robot2, Sai2Model::Sai2Model* object_model, Simulation::Sai2Simulation* sim);
void simulation(Simulation::Sai2Simulation* sim);

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

    // set up redis callbacks
    redis_client.connect();
    redis_client.createReadCallback(0);
    redis_client.createWriteCallback(0);

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_fname, false);

	// load robots
	auto robot1 = new Sai2Model::Sai2Model(robot_fname, false);
	auto robot2 = new Sai2Model::Sai2Model(robot_fname, false);

	// load object
	auto coobject = new Sai2Model::Sai2Model(object_fname, false);

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_fname, false);
	// set co-efficient of restition to zero to avoid bounce
    // see issue: https://github.com/manips-sai/sai2-simulation/issues/1
    sim->setCollisionRestitution(0.0);
    sim->setCoeffFrictionStatic(0.5);
    sim->setCoeffFrictionDynamic(0.5);

    // set joint damping on grippers: 
    auto base_1 = sim->_world->getBaseNode(robot1_name);
    auto gripper_1 = base_1->getJoint(gripper_joint_name);
    gripper_1->setDamping(10.0);
    gripper_1->setJointLimits(-0.005, 0.068, 0.005);
    auto base_2 = sim->_world->getBaseNode(robot2_name);
    auto gripper_2 = base_2->getJoint(gripper_joint_name);
    gripper_2->setDamping(10.0);
    gripper_2->setJointLimits(-0.005, 0.068, 0.005);

    // set initial conditions
	robot1->_q << 90/180.0*M_PI,
				-22.5/180.0*M_PI,
				212/180.0*M_PI,
				90.0/180.0*M_PI,
				100/180.0*M_PI,
				180/180.0*M_PI,
				0.0;
	sim->setJointPositions(robot1_name, robot1->_q);
	robot1->updateModel();
	robot2->_q << 90/180.0*M_PI,
				202.5/180.0*M_PI,
				-28/180.0*M_PI,
				-90.0/180.0*M_PI,
				97/180.0*M_PI,
				180/180.0*M_PI,
				0.0;
	sim->setJointPositions(robot2_name, robot2->_q);
	robot2->updateModel();
	Eigen::Affine3d ee_trans;

	// set up error callback
    glfwSetErrorCallback(glfwError);

    // initialize GLFW
    glfwInit();

    // retrieve resolution of computer display and position window accordingly
    GLFWmonitor *primary = glfwGetPrimaryMonitor();
    const GLFWvidmode *mode = glfwGetVideoMode(primary);

    // information about computer screen and GLUT display window
    int screenW = mode->width;
    int screenH = mode->height;
    int windowW = 0.8 * screenH;
    int windowH = 0.5 * screenH;
    int windowPosY = (screenH - windowH) / 2;
    int windowPosX = windowPosY;

    // create window and make it current
    glfwWindowHint(GLFW_VISIBLE, 0);
    GLFWwindow *window = glfwCreateWindow(windowW, windowH, sim_title, NULL, NULL);
    glfwSetWindowPos(window, windowPosX, windowPosY);
    glfwShowWindow(window);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // set callbacks
    glfwSetKeyCallback(window, keySelect);

    // cache variables
    double last_cursorx, last_cursory;

	// start the simulation thread first
    fSimulationRunning = true;
	thread sim_thread(simulation, sim);

	// next start the control thread
	thread ctrl_thread(control, robot1, robot2, coobject, sim);
	
    // while window is open:
    while (!glfwWindowShouldClose(window)) {

		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot1_name, robot1);
		graphics->updateGraphics(robot2_name, robot2);
		graphics->updateGraphics(object_name, coobject);
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

/* ----------------------------------------------------------------------------------
	Utility functions
-------------------------------------------------------------------------------------*/
// Calculate the cross product matrix
Eigen::Matrix3d getCrossProductMat(const Eigen::Vector3d& t) {
	Eigen::Matrix3d ret;
	ret <<  0, -t(2), t(1),
			t(2), 0, -t(0),
			-t(1), t(0), 0;
	return ret;
}

/* =======================================================================================
   CONTROL LOOP
========================================================================================== */
void control(Sai2Model::Sai2Model* robot1, Sai2Model::Sai2Model* robot2, Sai2Model::Sai2Model* object_model, Simulation::Sai2Simulation* sim) {
	
	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); //1000Hz timer
	double last_time = timer.elapsedTime()/time_slowdown_factor; //secs

	// load robot global frame to robot base transformations: todo: move out to a function
	Eigen::Affine3d robot1_base_frame = sim->getRobotBaseTransform(robot1_name);//from worl to base frame
	Eigen::Affine3d robot2_base_frame = sim->getRobotBaseTransform(robot2_name);

	// cache variables
	bool fTimerDidSleep = true;
	bool fTorqueUseDefaults = false; // set true when torques are overriden for the first time
	Eigen::VectorXd tau1 = Eigen::VectorXd::Zero(robot1->dof());
	Eigen::VectorXd tau2 = Eigen::VectorXd::Zero(robot2->dof());

	Eigen::Affine3d object_com_frame;//transformation matrix from world to object
	Eigen::Vector3d object_current_pos;
	Eigen::MatrixXd object_inertia(6,6);
	Eigen::MatrixXd object_j(6,6);
	Eigen::VectorXd object_p(6);

	Eigen::VectorXd robot1_g(robot1->dof());
	Eigen::VectorXd robot2_g(robot2->dof());

	// **  Other sugested variables and gains **
	 Eigen::Vector3d object_com_in_robot1_ee_frame;
	 Eigen::Vector3d object_com_in_robot2_ee_frame;
	 Eigen::MatrixXd robot1_j0_objcom(6, robot1->dof());
	 Eigen::MatrixXd robot2_j0_objcom(6, robot2->dof());
	 Eigen::MatrixXd robot1_j0_objectcom_bar(robot1->dof(), 6);
	 Eigen::MatrixXd robot2_j0_objectcom_bar(robot2->dof(), 6);
	 Eigen::MatrixXd robot1_objcom_inertia(6,6);
	 Eigen::MatrixXd robot2_objcom_inertia(6,6);

	 Eigen::MatrixXd augmented_object_inertia(6,6);
	 Eigen::VectorXd augmented_object_p(6);

	 Eigen::MatrixXd G(2*6, 2*6);
	// Eigen::MatrixXd W(6, 2*6);

	 Eigen::Vector3d obj_des_pos;
	 Eigen::Vector3d obj_ori_error;
	 Eigen::VectorXd obj_task_err(6);
	 Eigen::VectorXd force_des_vec(12);
	// Eigen::VectorXd force_ee_vec(12);

	 double kp = 30;
	 double kv = 10;

	// ** Control Mode **
	// 		0 = grasp stabilizing controller
	//		1 = augmented object controller
	ControlMode control_mode = CONTROL_GRASP_STABILIZE; 

	while (fSimulationRunning) { //automatically set to false when simulation is quit
		fTimerDidSleep = timer.waitForNextLoop();

		// update time
		double curr_time = timer.elapsedTime()/time_slowdown_factor;
		double loop_dt = curr_time - last_time;

        // read joint positions, velocities
        sim->getJointPositions(robot1_name, robot1->_q);
        sim->getJointVelocities(robot1_name, robot1->_dq);
        robot1->updateModel();
        sim->getJointPositions(robot2_name, robot2->_q);
        sim->getJointVelocities(robot2_name, robot2->_dq);
        robot2->updateModel();

        // read object position
        sim->getJointPositions(object_name, object_model->_q);
        sim->getJointVelocities(object_name, object_model->_dq);
        object_model->updateModel();

        // update object dynamics
		// - find object COM frame in global frame
		object_model->transform(object_com_frame, object_link_name);//tranformation from Object to world
		object_current_pos << object_com_frame.translation();//current position in world frame
		redis_client.addEigenToWriteCallback(0, CURRENT_OBJECT_POSITION_KEY, object_current_pos);
		// - obtain object inertia matrix in world frame
		object_model->J_0(object_j, object_link_name, Eigen::Vector3d::Zero());
		object_inertia = (object_j*object_model->_M_inv*object_j.transpose()).inverse();//Lambd amatrix of the object
		// - obtain object p
		object_p << 0, 0, 9.8*0.5, 0, 0, 0;//gravity vector of the object










        // ---------------------------------------------------------------------------------
        /* ---------------------- FILL ME IN ----------------------------------------------- */
		
		// --------------------------------------------------------------------
		// (1) Manipulator Jacobians
		//---------------------------------------------------------------------
		// ** FILL ME IN ** 
		//First we need to known the position of the object COM seen from the end effectors of the robot
		//This mean Transform object COM to Link frame. We need: 
		//step1. Get transformation matrices from  eef link frame to base of robot
		Eigen::Affine3d T_Robot1eef_to_base;
		Eigen::Affine3d T_Robot2eef_to_base;
		robot1->transform(T_Robot1eef_to_base, ee_link_name);
		robot2->transform(T_Robot2eef_to_base, ee_link_name);
		//step 2: Get total_transformation=T_{base to eef}*T_{world to base}. robot1_base_frame=base_to_world
		Eigen::Affine3d T_Robot1world_to_eef;
		Eigen::Affine3d T_Robot2world_to_eef;
		T_Robot1world_to_eef=T_Robot1eef_to_base.inverse()*robot1_base_frame.inverse();
		T_Robot2world_to_eef=T_Robot2eef_to_base.inverse()*robot2_base_frame.inverse();
		//Step3:get the position of the object in world frame
		auto T_Objectself_to_world=object_com_frame;
		auto object_current_pos_world=(T_Objectself_to_world.translation());
		//step4:Position COM object in eef frame=total_transformation*COM_object_in_worldFrame. No need to add 1 in vector?
		object_com_in_robot1_ee_frame=T_Robot1world_to_eef*object_current_pos_world;
	    object_com_in_robot2_ee_frame=T_Robot2world_to_eef*object_current_pos_world;

		//Now we compute J0 in the end effector frame, at the distance where the COM ob the object is placed. using world frame
		robot1->J_0WorldFrame(robot1_j0_objcom,ee_link_name, object_com_in_robot1_ee_frame);
		robot2->J_0WorldFrame(robot2_j0_objcom,ee_link_name, object_com_in_robot2_ee_frame);

		



		// --------------------------------------------------------------------
		// (2) Augmented Object Model
		//---------------------------------------------------------------------
		// ** FILL ME IN ** 
		// AUGMENTD OBJECT INERTIA
		//step1: compute the inertia (Lambda) at the end effector of each robot
		robot1_objcom_inertia=(robot1_j0_objcom*robot1->_M_inv*robot1_j0_objcom.transpose()).inverse();
	    robot2_objcom_inertia=(robot2_j0_objcom*robot2->_M_inv*robot2_j0_objcom.transpose()).inverse();
		//step 2: add them
		augmented_object_inertia=object_inertia+robot1_objcom_inertia+robot2_objcom_inertia;

		//AUGMENTED OBJECT P
		//step1: we need the gravity vector of each robot
		robot1->gravityVector(robot1_g);
		robot2->gravityVector(robot2_g);
		//step 2: we will need J_bar (in the COM of object) Jbar=M_inv*J.transpose*Lambda
		robot1_j0_objectcom_bar=robot1->_M_inv*robot1_j0_objcom.transpose()*robot1_objcom_inertia;
		robot2_j0_objectcom_bar=robot2->_M_inv*robot2_j0_objcom.transpose()*robot2_objcom_inertia;
		//step3: compute p for each robot p=Jbar.transpose*g
		Eigen::VectorXd robot1_p(6);
		Eigen::VectorXd robot2_p(6);
		robot1_p=robot1_j0_objectcom_bar.transpose()*robot1_p;
		robot2_p=robot2_j0_objectcom_bar.transpose()*robot2_p;
		//step4: sum all p
		augmented_object_p=object_p+robot1_p+robot2_p;

		

		// --------------------------------------------------------------------
		// (3) Grasp Matrix
		//---------------------------------------------------------------------
		// ** FILL ME IN ** 
		//I will construct the blocks that compose the grasp matrix, then build the grasp matrix
		//useful matrices
		//Identity 3x3
		const Eigen::MatrixXd I3 = Eigen::MatrixXd::Identity(3,3);
		//zero 3x3
		const Eigen::MatrixXd Z3= Eigen::MatrixXd::Zero(3,3);

		//WF
		//step1:we need the position of the end efectors seen from the object frame.
		//--step1.1: get the position of the end efector in world frame
		Eigen::Vector3d robot1_eef_world;
		Eigen::Vector3d robot2_eef_world;
		robot1_eef_world=(T_Robot1world_to_eef.inverse()).translation();
		robot2_eef_world=(T_Robot2world_to_eef.inverse()).translation();
		//--step1.2. get the position of the end efector seen from object p_eef_object=T_{from world to object}*p_ee_world
		Eigen::Vector3d robot1_eef_objcom=T_Objectself_to_world.inverse()*robot1_eef_world;
		Eigen::Vector3d robot2_eef_objcom=T_Objectself_to_world.inverse()*robot2_eef_world;
		//step2: get r1_hat(r1=robot1_eef_objcom), and r2_hat (r2=robot2_eef_objcom) (crossproduct matrices). 
		Eigen::Matrix3d r1_hat;
		Eigen::Matrix3d r2_hat;
		r1_hat=getCrossProductMat(robot1_eef_objcom);
		r2_hat=getCrossProductMat(robot2_eef_objcom);
		//step3: compute Wf
		Eigen::MatrixXd Wf(6, 6);
		Wf<< I3,I3,r1_hat,r2_hat;

		
		//Wm
		Eigen::MatrixXd Wm(6, 6);
		Wm<< Z3,Z3,I3,I3;

		//E_bar
		//step1: compute E. Link in direction (0,1,0)
		Eigen::MatrixXd E(6,1);
		E<<0,-1,0,0,1,0;
		//step2: compute E_bar
		Eigen::MatrixXd E_bar(1,6);
		E_bar=(E.transpose()*E).inverse()*E.transpose();

		//I9 to tranform 2 external moments to 5 internal moments. where the moment in Y will be the diference of the Y component of the external moments
		Eigen::MatrixXd I9(5,6);
		I9<< 0,-0.5, 0 ,0, 0.5,0,
			1,0,0,0,0,0,
			0,0,1,0,0,0,
			0,0,0,1,0,0,
			0,0,0,0,0,1;

		//Fill G
		G<< Wf,Wm,
			E_bar,Eigen::MatrixXd::Zero(1,6),
			Eigen::MatrixXd::Zero(5,6),I9;



		// --------------------------------------------------------------------
		// (4) Force Control 
		//---------------------------------------------------------------------
		if (control_mode == CONTROL_AUGMENTED_OBJECT) {
		
		// ** FILL ME IN ** Compute tau1 and tau2 	
		//Useful parameters
		auto t=curr_time; //easy refer to time

		//DESIRED VALUES positiona nd speed for the COM of Object
		//---desired position of the object
		obj_des_pos<< 0, 0.15*sin(2*M_PI*t/3),0.4;
		//---desired speed of the object
		Eigen::Vector3d obj_des_v;
		obj_des_v<<0,0.15*2*M_PI/3*cos(2*M_PI*t/3),0;
		//---desired acceleration of the object
		Eigen::Vector3d obj_des_a;
		obj_des_a<<0,-0.15*4*pow(M_PI,2)/9*sin(2*M_PI*t/3),0;

		//---desired orientation in euler parameters
		Eigen::VectorXd obj_des_ori(4); //lambdas. orientation
		obj_des_ori<< 1,0,0,0;
		

		//----desired internal tension
        double t_int = -15;
		//----desired moments
		Eigen::VectorXd tau_int(5); //lambdas
		tau_int<< 0,0,0,0,0;


		//CURRENT VALUES Position and Velocity of the object
		//--current position of object:
		//no need to compute, it will be object_current_pos_world
		//--current orientation of the object as quaternion seen from world
		Eigen::Quaterniond obj_curr_orient=(Eigen::Quaterniond)((T_Objectself_to_world).rotation());

	
		//compute with jacobean linear and angular velocity
		Eigen::VectorXd obj_v_omega(6);
		obj_v_omega=object_j*object_model->_dq;
		Eigen::Vector3d obj_v;
		Eigen::Vector3d obj_omega;
		obj_v=obj_v_omega.segment(0,3);
		obj_omega=obj_v_omega.segment(3,6);

		//ORIENTATION ERROR
		//define matrix check{\lambda}.transpose for current object orientation
		auto lb_c=obj_curr_orient;//to make it easier to write
		Eigen::MatrixXd obj_current_clambda_t(3,4);
		obj_current_clambda_t<<-lb_c.x(), lb_c.w(), -lb_c.z(), lb_c.y(), 
				   -lb_c.y(),lb_c.z(),lb_c.w(),-lb_c.x(),
				   -lb_c.z(),-lb_c.y(),lb_c.x(),lb_c.w();

		//angle error between current and desired orientation
		obj_ori_error=-2*obj_current_clambda_t*obj_des_ori;

		


		//DECOUPLED CONTROLLERS
		//Decoupled Force for motion control
		Eigen::VectorXd F_star;
		F_star=obj_des_a-1*kp*(object_current_pos_world-obj_des_pos)-kv*(obj_v-obj_des_v);
		//decoupled moment_motion, no acceleration and velcity desired, only position.
		Eigen::VectorXd M_star;
		M_star=-kp*obj_ori_error-kv*(obj_omega);
		//total decoupled_motion
		Eigen::VectorXd V_star(6);
		V_star<<F_star,M_star;


		//COMPUTE FORCES AND MOMENTUM PERSIVED both ROBOTS 
		Eigen::VectorXd obj_F(12);//auxiliary vector for the multiplication G.inverse()*obj_F
		obj_F<< augmented_object_inertia*V_star+object_p, t_int, tau_int;
		force_des_vec=G.inverse()*obj_F;
		//the previous forces are in the object frame. We need them in world frame
		//step1:extratc forces and momentums
		Eigen::Vector3d f1_ee_object_frame;
		Eigen::Vector3d m1_ee_object_frame;
		Eigen::Vector3d f2_ee_object_frame;
		Eigen::Vector3d m2_ee_object_frame;
		f1_ee_object_frame=force_des_vec.segment(0,3);
		m1_ee_object_frame=force_des_vec.segment(6,3);
		f2_ee_object_frame=force_des_vec.segment(3,3);
		m2_ee_object_frame=force_des_vec.segment(9,3);
		//step2: Compute forces for each robot
		Eigen::VectorXd F1_ee(6);
		Eigen::VectorXd F2_ee(6);		
		F1_ee<< T_Objectself_to_world.rotation()*f1_ee_object_frame,T_Objectself_to_world.rotation()*m1_ee_object_frame;
		F2_ee<<  T_Objectself_to_world.rotation()*f2_ee_object_frame,T_Objectself_to_world.rotation()*m2_ee_object_frame;


		

		//TAUS
		//step 1: we need the J in the robot end effector in world frame
		Eigen::MatrixXd  robot1_j0(6, robot1->dof());
		Eigen::MatrixXd  robot2_j0(6, robot1->dof());
		robot1->J_0WorldFrame(robot1_j0,ee_link_name, Eigen::VectorXd::Zero(3));
		robot2->J_0WorldFrame(robot2_j0,ee_link_name, Eigen::VectorXd::Zero(3));
		
		
		//step3: compute tau
		tau1=robot1_j0.transpose()*F1_ee+robot1_g;
		tau2=robot2_j0.transpose()*F2_ee+robot2_g;

	








		} else if (control_mode == CONTROL_GRASP_STABILIZE) { // initial grasp stabilization
			
			Eigen::MatrixXd robot1_j0_ee(6, robot1->dof());
			Eigen::MatrixXd robot2_j0_ee(6, robot2->dof());
			robot1->J_0(robot1_j0_ee, ee_link_name, Eigen::Vector3d::Zero());
			robot2->J_0(robot2_j0_ee, ee_link_name, Eigen::Vector3d::Zero());

			// Joint Torques
			tau1 = robot1_j0_ee.transpose()*(object_p/2) + robot1->_M*(-10.0*robot1->_dq) + robot1_g;
			tau2 = robot2_j0_ee.transpose()*(object_p/2) + robot2->_M*(-10.0*robot2->_dq) + robot2_g;
			
			// Grasp stabilization
			static uint grasp1Counter = 0;
			static uint grasp2Counter = 0;
			if (robot1->_dq[6] < 0.1) {
				grasp1Counter += 1;
			} else {
				grasp1Counter = 0;
			}
			if (robot2->_dq[6] < 0.1) {
				grasp2Counter += 1;
			} else {
				grasp2Counter = 0;
			}
			if (grasp1Counter > 40 && grasp2Counter > 40) {
				cout << " ** Switch Control Mode to Augmented Object Model ** " << endl;
				control_mode = CONTROL_AUGMENTED_OBJECT;
			}
		}

		/* ----------------------------------------------------------------------------------
			Safety torques 
		-------------------------------------------------------------------------------------*/ 
		
		// Set constant gripper forces
		tau1[6] = 15;
		tau2[6] = 15;

        // Default values if torques are exceeded:
        bool fTorqueOverride = false; // to avoid robot blow-ups
        const double tau1_max = 200;
        const double tau2_max = 200;
        if (!fTorqueUseDefaults) {
        	if (tau1.cwiseAbs().maxCoeff() > tau1_max || tau2.cwiseAbs().maxCoeff() > tau2_max) {
	        	fTorqueOverride = true;
	        	cerr << "Torque overriden. User asked torques beyond safe limit: \n";
	        	cerr << "Robot 1: " << tau1.transpose() << "\n";
	        	cerr << "Robot 2: " << tau2.transpose() << "\n";
	        	fTorqueUseDefaults = true;
	        }
	        // Also default values if object is dropped
	        const double object_thickness = 0.05;
	        bool fRobot1DroppedObject = robot1->_q[6] > object_thickness/2;
	        bool fRobot2DroppedObject = robot2->_q[6] > object_thickness/2;
	        if (fRobot1DroppedObject || fRobot2DroppedObject) {
	        	cerr << "Torque overriden. Robot 1 or 2 dropped object. \n";
	        	fTorqueUseDefaults = true;
	        }
        }
        else {
        	robot1->gravityVector(tau1);
			tau1 = tau1 + robot1->_M*(-10.0*robot1->_dq);
			tau1 = (tau1.array() >= tau1_max).select(tau1_max*Eigen::VectorXd::Ones(robot1->dof()), tau1);
			tau1 = (tau1.array() <= -tau1_max).select(-tau1_max*Eigen::VectorXd::Ones(robot1->dof()), tau1);
			robot2->gravityVector(tau2);
			tau2 = tau2 + robot2->_M*(-10.0*robot2->_dq);
			tau2 = (tau2.array() >= tau2_max).select(tau2_max*Eigen::VectorXd::Ones(robot2->dof()), tau2);
			tau2 = (tau2.array() <= -tau2_max).select(-tau2_max*Eigen::VectorXd::Ones(robot2->dof()), tau2);
        }

        /* ----------------------------------------------------------------------------------
			Send robot torques from controller
		-------------------------------------------------------------------------------------*/ 
		sim->setJointTorques(robot1_name, tau1);
		sim->setJointTorques(robot2_name, tau2);
		
		// write object location key out to redis
		redis_client.executeWriteCallback(0);

		// update last time
		last_time = curr_time;
	}
}

/* =======================================================================================
   SIMULATION SETUP
   -----------------------
   * Simulation loop
   * Select trajectory
   * Window initialization
   * Window error
   * Mouse click commands
========================================================================================== */
void simulation(Simulation::Sai2Simulation* sim) {
	fSimulationRunning = true;

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(10000); //10000Hz timer

	double last_time = timer.elapsedTime()/time_slowdown_factor; //secs
	bool fTimerDidSleep = true;
	while (fSimulationRunning) {
		fTimerDidSleep = timer.waitForNextLoop();
		// integrate forward
		double curr_time = timer.elapsedTime()/time_slowdown_factor;
		double loop_dt = curr_time - last_time; 
		// sim->integrate(loop_dt);
		sim->integrate(loop_dt);
		// update last time
		last_time = curr_time;
	}
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
