#include <boost/bind.hpp>
#include <gazebo.hh>
#include <physics/physics.hh>
#include <common/common.hh>
#include <sensors/sensors.hh>
#include <sensors/CameraSensor.hh>
#include <stdio.h>
#include <iostream>
#include <sys/types.h> /* definicja typu pid_t */
#include <sys/wait.h>  /* wait() */
#include <unistd.h>    /* getpid(); */
#include <signal.h>    /* kill(); */
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>             /* for strerror(int errno) */
#include <errno.h>
#include <pthread.h> 
#include <math.h>
#include <time.h>
#include "PID.hh"
#define PI 3.14159
#include <termios.h>

static struct termios oldt;

void restore_terminal_settings(void)
{
    tcsetattr(0, TCSANOW, &oldt);  /* Apply saved settings */
}

void disable_waiting_for_enter(void)
{
    struct termios newt;

    /* Make terminal read 1 char at a time */
    tcgetattr(0, &oldt);  /* Save terminal settings */
    newt = oldt;  /* Init new settings */
    newt.c_lflag &= ~(ICANON | ECHO);  /* Change settings */
    tcsetattr(0, TCSANOW, &newt);  /* Apply settings */
    atexit(restore_terminal_settings); /* Make sure settings will be restored when program ends  */
}

using namespace std;
namespace gazebo
{
  //Zmienne globalne bleeeee
  pthread_mutex_t r_mutex = PTHREAD_MUTEX_INITIALIZER;
  pthread_t sthread;
  int ZMIENNA_GLOBALNA='0';
  double Angle=0;
  
  static struct timespec tS;
  //Koniec zmiennych globalnych
  
  
  //WATEK
  void* ruchable(void*){
    int c;
    disable_waiting_for_enter();
    while(1){
      c=getchar();
      pthread_mutex_lock(&r_mutex);
      ZMIENNA_GLOBALNA=c;       
      //cout<< ZMIENNA_GLOBALNA<<endl;
      pthread_mutex_unlock(&r_mutex);
    }
  }
  class MobileBasePlugin : public ModelPlugin
  {
  public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) 
    {

      // Store the pointer to the model
      this->model = _parent;

      // Load parameters for this plugin
      if (this->LoadParams(_sdf))
	{
	  // Listen to the update event. This event is broadcast every
	  // simulation iteration.
	  this->updateConnection = event::Events::ConnectWorldUpdateStart(
									  boost::bind(&MobileBasePlugin::OnUpdate, this));
	}
    }

  public: bool LoadParams(sdf::ElementPtr _sdf) 
    {

      // Find controller gain
      // if (!_sdf->HasElement("gain"))
      // 	{
      // 	  gzerr << "param [gain] not found\n";
      // 	  return false;
      // 	}
      // else
      // 	{
      // 	  // Get sensor name
      // 	  this->gain =
      // 	    _sdf->GetElement("gain")->GetValueDouble();
      // 	}

      // Find sensor name from plugin param

      pthread_create(&sthread,NULL,ruchable,NULL);
      if (!_sdf->HasElement("camera_sensor"))
	{
	  gzerr << "param [camera] not found\n";
	  return false;
	}
      else
	{
	  // Get sensor name
	  std::string sensorName =
	    _sdf->GetElement("camera_sensor")->GetValueString();

	  // Get pointer to sensor using the SensorMangaer
	  sensors::SensorPtr sensor =
	    sensors::SensorManager::Instance()->GetSensor(sensorName);

	  if (!sensor)
	    {
	      gzerr << "sensor by name ["
		    << sensorName
		    << "] not found in model\n";
	      return false;
	    }

	  this->camera = boost::shared_dynamic_cast<sensors::CameraSensor>
	    (sensor);
	  if (!this->camera)
	    {
	      gzerr << "camera by name ["
		    << sensorName
		    << "] not found in model\n";
	      return false;
	    }
	}

      // Load joints from plugin param
      if (!this->FindJointByParam(_sdf, this->leftWheelJoint,
      				  "left_wheel_hinge") ||
          !this->FindJointByParam(_sdf, this->rightWheelJoint,
      				  "right_wheel_hinge"))
        return false;

      // success
      return true;
    }

  public: bool FindJointByParam(sdf::ElementPtr _sdf,
				physics::JointPtr &_joint,
				std::string _param)
    {
      if (!_sdf->HasElement(_param))
	{
	  gzerr << "param [" << _param << "] not found\n";
	  return false;
	}
      else
	{
	  _joint = this->model->GetJoint(
					 _sdf->GetElement(_param)->GetValueString());

	  if (!_joint)
	    {
	      gzerr << "joint by name ["
		    << _sdf->GetElement(_param)->GetValueString()
		    << "] not found in model\n";
	      return false;
	    }
	}
      return true;
    }

    // Called by the world update start event
  public: void OnUpdate()
    {
      static int Licznik=0;
      static double takenTime=0;
      if(Licznik==0)
	clock_settime(CLOCK_PROCESS_CPUTIME_ID, &tS);
      char buffer [70];
      sprintf (buffer, "/home/kuba/frames/%d.png", Licznik);
      Licznik++;
      static PID LeftAngleController(5,0.01,0.1);
      static PID RightAngleController(5,0.01,0.1);
      static PID LeftVelController(0.1,0.01,0.01);
      static PID RightVelController(0.1,0.01,0.01);
      LeftAngleController.SetSP(Angle);
      RightAngleController.SetSP(Angle);

      pthread_mutex_lock(&r_mutex);
      if(ZMIENNA_GLOBALNA=='a'){
	Angle=Angle-5;
	LeftAngleController.SetSP(Angle);
	RightAngleController.SetSP(Angle);
	ZMIENNA_GLOBALNA='0';
	cout<<"Aktualny Kat:"<<Angle<<endl;
      }
      else if(ZMIENNA_GLOBALNA=='d'){
	Angle=Angle+5;
	LeftAngleController.SetSP(Angle);
	RightAngleController.SetSP(Angle);
	cout<<"Aktualny Kat:"<<Angle<<endl;
	ZMIENNA_GLOBALNA='0';
      }
      else{
	//cout<<"Bez zmian"<<endl;
      }
      pthread_mutex_unlock(&r_mutex);
      double katL=leftWheelJoint->GetAngle(0).Degree();
      double katR=rightWheelJoint->GetAngle(0).Degree();

      LeftVelController.SetSP(-5);
      RightVelController.SetSP(-5);
      double velL=leftWheelJoint->GetVelocity(1);
      double velR=rightWheelJoint->GetVelocity(1);

      

      leftWheelJoint->SetForce(0, LeftAngleController.Control(katL));
      rightWheelJoint->SetForce(0, RightAngleController.Control(katR));	
      leftWheelJoint->SetForce(1, LeftVelController.Control(velL));  
      rightWheelJoint->SetForce(1, RightVelController.Control(velR));
     // this->camera->SaveFrame(buffer);
      clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &tS);
      takenTime+=(double) tS.tv_nsec;
      if(takenTime>30000000){ //300k mikrosekund
	this->camera->SaveFrame(buffer);
	//cout<<"Robie zdjecie"<<takenTime<<endl;
	takenTime=0;
      }
    }

   // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    private: physics::JointPtr leftWheelJoint;
    private: physics::JointPtr rightWheelJoint;
    private: sensors::CameraSensorPtr camera;
    // private: double gain;

   // Pointer to the model
  // private: physics::ModelPtr model;

  //   // Pointer to the update event connection
  // private: event::ConnectionPtr updateConnection;

  // private: physics::JointPtr left_wheel_joint_;
  // private: physics::JointPtr right_wheel_joint_;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(MobileBasePlugin)
}
