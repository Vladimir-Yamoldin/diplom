/*
 * AUTHOR: Carlo Pinciroli <cpinciro@ulb.ac.be>
 *
 * An example diffusion controller for the foot-bot.
 *
 * This controller makes the robots behave as gas particles. The robots
 * go straight until they get close enough to another robot, in which
 * case they turn, loosely simulating an elastic collision. The net effect
 * is that over time the robots diffuse in the environment.
 *
 * The controller uses the proximity sensor to detect obstacles and the
 * wheels to move the robot around.
 *
 * This controller is meant to be used with the XML files:
 *    experiments/diffusion_1.argos
 *    experiments/diffusion_10.argos
 */

#ifndef FOOTBOT_DIFFUSION_H
#define FOOTBOT_DIFFUSION_H

/* Include some necessary headers */

/* Definition of the CCI_Controller class */
#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the generic differential steering actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_differential_steering_actuator.h>
/* Definition of the generic positioning sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
/* Definition of the LEDs actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
/* Definition of the omnidirectional camera sensor */
#include <argos3/plugins/robots/generic/simulator/colored_blob_omnidirectional_camera_rotzonly_sensor.h>
/* Definition of the foot-bot distance scanner sensor */
#include <argos3/plugins/robots/foot-bot/simulator/footbot_distance_scanner_rotzonly_sensor.h>
/* Definition of the foot-bot distance scanner actuator*/
#include <argos3/plugins/robots/foot-bot/simulator/footbot_distance_scanner_default_actuator.h>

//all the ARGoS stuff is in the 'argos' namespace; 
//with this statement, you save typing argos:: every time
using namespace argos;

//a controller is simply an implementation of the CCI_Controller class
class CFootBotDiffusion : public CCI_Controller {

public:

   CFootBotDiffusion(); //Class constructor 

   virtual ~CFootBotDiffusion() {} //Class destructor

   /* This function initializes the controller.
    * The 't_node' variable points to the <parameters> section in the XML
    * file in the <controllers><footbot_diffusion_controller> section */
   virtual void Init(TConfigurationNode& t_node);

   /* This function is called once every time step.
    * The length of the time step is set in the XML file */
   virtual void ControlStep();

   /* This function resets the controller to its state right after the Init().
    * It is called when you press the reset button in the GUI */
   virtual void Reset();

   /* Called to cleanup what done by Init() when the experiment finishes.
    * In this example controller there is no need to clean anything up,
    * so the function could have been omitted. It's here just for completeness */
   virtual void Destroy();

protected:

   virtual CVector3 RepulsionForce(); //the repulsion force, caused by the obstacles and other agents
   virtual CVector3 SocialForce(); //the attraction force, caused by neighbouring agents

private:

   CCI_DifferentialSteeringActuator* m_pcWheels; //pointer to the differential steering actuator
   CCI_PositioningSensor* m_pcPosition; //pointer to the positioning sensor
   CCI_LEDsActuator* m_pcLEDs; //pointer to the LEDs actuator
   CColoredBlobOmnidirectionalCameraRotZOnlySensor* m_pcCamera; //pointer to the omnidirectional camera sensor
   CFootBotDistanceScannerRotZOnlySensor* m_pcDistSens; //pointer to the distance scanner sensor
   CFootBotDistanceScannerDefaultActuator* m_pcDistAct; //pointer to the distance scanner actuator

   /* The following variables are used as parameters for the
    * algorithm. You can set their value in the <parameters> section
    * of the XML configuration file, under the
    * <controllers><footbot_diffusion_controller> section */

   CDegrees m_cAlpha; //maximum tolerance for the angle between the robot heading direction and the velocity
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