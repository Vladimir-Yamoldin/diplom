 /* AUTHOR: Carlo Pinciroli <cpinciro@ulb.ac.be>
 *
 * An example flocking controller for the foot-bot.
 *
 * This controller lets a group of foot-bots flock in an hexagonal lattice towards
 * a light source placed in the arena. To flock, it exploits a generalization of the
 * well known Lennard-Jones potential. The parameters of the Lennard-Jones function
 * were chosen through a simple trial-and-error procedure on its graph.
 *
 * This controller is meant to be used with the XML file:
 *    experiments/flocking.argos
 */

#ifndef FOOTBOT_FLOCKING_H
#define FOOTBOT_FLOCKING_H

/* Include some necessary headers. */
/* Definition of the CCI_Controller class. */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the generic positioning sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
/* Definition of the LEDs actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
/* Definition of the omnidirectional camera sensor */
#include <argos3/plugins/robots/generic/simulator/colored_blob_omnidirectional_camera_rotzonly_sensor.h>
/* Definition of the foot-bot light sensor */
#include <argos3/plugins/robots/foot-bot/control_interface/ci_footbot_light_sensor.h>
/* Definition of the foot-bot distance scanner sensor */
#include <argos3/plugins/robots/foot-bot/simulator/footbot_distance_scanner_rotzonly_sensor.h>
/* Definition of the foot-bot distance scanner actuator*/
#include <argos3/plugins/robots/foot-bot/simulator/footbot_distance_scanner_default_actuator.h>

/*
 * All the ARGoS stuff in the 'argos' namespace.
 * With this statement, you save typing argos:: every time.
 */
using namespace argos;

// a controller is simply an implementation of the CCI_Controller class
class CFootBotFlocking : public CCI_Controller {

public:

   CFootBotFlocking(); /* Class constructor. */

   virtual ~CFootBotFlocking() {} /* Class destructor. */

   /*
    * This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML file
    * in the <controllers><footbot_flocking_controller> section.
    */
   virtual void Init(TConfigurationNode& t_node);

   /*
    * This function is called once every time step.
    * The length of the time step is set in the XML file.
    */
   virtual void ControlStep();

   /*
    * This function resets the controller to its state right after the Init().
    * It is called when you press the reset button in the GUI.
    * In this example controller there is no need for resetting anything, so
    * the function could have been omitted. It's here just for completeness.
    */
   virtual void Reset();

   /*
    * Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need for clean anything up, so
    * the function could have been omitted. It's here just for completeness.
    */
   virtual void Destroy();

protected:

   virtual CVector3 TargetForce(); //the attraction force, caused by the target
   virtual CVector3 RepulsionForce(); //the repulsion force, caused by the obstacles and other agents
   virtual CVector3 SocialForce(); //the attraction force, caused by neighbouring agents

private:

   /* Pointer to the differential steering actuator */
   CCI_DifferentialSteeringActuator* m_pcWheels;
   /* Pointer to the positioning sensor */
   CCI_PositioningSensor* m_pcPosition;
   /* Pointer to the foot-bot light sensor */
   CCI_FootBotLightSensor* m_pcLight;
   /* Pointer to the LEDs actuator */
   CCI_LEDsActuator* m_pcLEDs;
   /* Pointer to the omnidirectional camera sensor */
   CColoredBlobOmnidirectionalCameraRotZOnlySensor* m_pcCamera;
   /* Pointer to the distance scanner sensor */
   CFootBotDistanceScannerRotZOnlySensor* m_pcDistSens;
   /* Pointer to the distance scanner actuator */
   CFootBotDistanceScannerDefaultActuator* m_pcDistAct;
   CDegrees m_cAlpha; //maximum tolerance for the angle between the robot heading direction and the velocity
   Real m_fTargetRadius; //the radius of the target area (in meters)
   Real Upper_limit; //maximum wheel speed
   Real Lower_limit; //minimum wheel speed
   Real Change_limit; //maximum wheel speed change per step (acceleration restriction)
   Real PrevSpeedLeft; //left wheel speed on the previous step
   Real PrevSpeedRight; //right wheel speed on the previous step
   CVector3 m_fMassCenterVelocity; //footbot mass center velocity
   CVector3 m_fTargetCenter; //center of the target area
   int ticks_counter = 0;
   int lost_counter = 0;
   bool got_lost = false;
};

#endif
