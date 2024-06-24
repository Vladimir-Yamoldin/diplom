/* Include the controller definition */
#include "footbot_flocking.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/logging/argos_log.h>
/* 3D vector definition */
#include <argos3/core/utility/math/vector3.h>

static const Real FOOTBOT_RADIUS = 0.07f; //footbot radius in meters
static const Real OBST_RADIUS = 0.2f; //radius of obstacle influence in meters
static const Real STEP_TIME = 0.1f; //step time in seconds

/****************************************/
/****************************************/

CFootBotFlocking::CFootBotFlocking() :
   m_pcWheels(NULL),
   m_pcPosition(NULL),
   m_pcLight(NULL),
   m_pcLEDs(NULL),
   m_pcCamera(NULL),
   m_pcDistSens(NULL),
   m_pcDistAct(NULL) {}

/****************************************/
/****************************************/

void CFootBotFlocking::Init(TConfigurationNode& t_node) {
   /*
    * Get sensor/actuator handles
    *
    * The passed string (ex. "differential_steering") corresponds to the XML tag of the
    * device whose handle we want to have. For a list of allowed values, type at the
    * command prompt:
    *
    * $ argos3 -q actuators
    *
    * to have a list of all the possible actuators, or
    *
    * $ argos3 -q sensors
    *
    * to have a list of all the possible sensors.
    *
    * NOTE: ARGoS creates and initializes actuators and sensors internally, on the basis of
    *       the lists provided the configuration file at the
    *       <controllers><footbot_diffusion><actuators> and
    *       <controllers><footbot_diffusion><sensors> sections. If you forgot to
    *       list a device in the XML and then you request it here, an error occurs.
    */ 
   m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator          >("differential_steering");
   m_pcPosition = GetSensor<CCI_PositioningSensor                     >("positioning");
   m_pcLight  = GetSensor  <CCI_FootBotLightSensor                    >("footbot_light");
   m_pcLEDs   = GetActuator<CCI_LEDsActuator                          >("leds");
   m_pcCamera = GetSensor  <CColoredBlobOmnidirectionalCameraRotZOnlySensor>("colored_blob_omnidirectional_camera");
   m_pcDistSens = GetSensor<CFootBotDistanceScannerRotZOnlySensor     >("footbot_distance_scanner");
   m_pcDistAct = GetActuator<CFootBotDistanceScannerDefaultActuator   >("footbot_distance_scanner");
   Real x, y, z;
   try {
      GetNodeAttribute(t_node, "threshold", m_cAlpha);
      GetNodeAttribute(t_node, "upper_limit", Upper_limit);
      GetNodeAttribute(t_node, "lower_limit", Lower_limit);
      GetNodeAttribute(t_node, "change_limit", Change_limit);
      GetNodeAttribute(t_node, "target_r", m_fTargetRadius);
      GetNodeAttribute(t_node, "target_x", x);
      GetNodeAttribute(t_node, "target_y", y);
      GetNodeAttribute(t_node, "target_z", z);
   }
   catch(CARGoSException& ex) {
      THROW_ARGOSEXCEPTION_NESTED("leader: error initializing controller parameters", ex);
   }
   m_fTargetCenter = CVector3(x, y, z); //define the target center
   Reset(); //other init stuff
}

/****************************************/
/****************************************/

void CFootBotFlocking::Reset() {
   //initial speeds are all zero
   m_fMassCenterVelocity = CVector3::ZERO;
   PrevSpeedLeft = 0.0;
   PrevSpeedRight = 0.0;
   //set beacon color to green to show that this agent is a leader
   m_pcLEDs->SetSingleColor(12, CColor::GREEN);
   m_pcCamera->Enable(); //enable camera filtering
   m_pcDistAct->Enable(); //enable distance scanner
   ticks_counter = 0;
   lost_counter = 0;
   got_lost = false;
   //LOG.RedirectToFile("graph.txt"); //does not work((
}

/****************************************/
/****************************************/

void CFootBotFlocking::ControlStep() {
   CVector3 Acceleration; Real Velocity;
   CQuaternion Orient = m_pcPosition->GetReading().Orientation;
   CRadians RobotAngleZ, RobotAngleY, RobotAngleX;
   Orient.ToEulerAngles(RobotAngleZ, RobotAngleY, RobotAngleX);
   CVector3 Target, Repulsion, Attraction;
   Target = TargetForce();
   /*rotation around Z is needed, because the footbot local coordinate system 
     is turned with respect to the global one */
   Repulsion = RepulsionForce().RotateZ(RobotAngleZ);
   Attraction = SocialForce();
   //leader acceleration is defined only by target and repulsion forces
   Acceleration = Target + Repulsion;
   //calculate mass center velocity vector on this step, V(t) = V(t-1) + a * step_time
   m_fMassCenterVelocity += STEP_TIME * Acceleration;
   Velocity = m_fMassCenterVelocity.Length();
   CRadians VelocityAngleZ; //the angle between Z axis and the velocity vector
   VelocityAngleZ = m_fMassCenterVelocity.GetZAngle();
   CRadians AngleZ; //the angle between the robot heading direction and the velocity vector
   AngleZ = NormalizedDifference(VelocityAngleZ, RobotAngleZ);
   /* if the leader is close enough to the target, it is ready to stop, because the mission is 
      complete. PROBLEM: the leader just stops and ignores slowing down */
   if (Target == CVector3::ZERO) {
      m_pcWheels->SetLinearVelocity(0, 0);
      return;
   }
   Real speed1, speed2; //speeds of the wheels, speed1 <= speed2 always
   /* in case -alpha < angle < alpha the footbot goes straight, 
      both wheels move as a mass center and with the same speeds */
   if (Abs(AngleZ) <= ToRadians(m_cAlpha)) {
      speed1 = speed2 = Velocity;
      /* the next restrictions come from social interaction rules. If the leader observes few 
      other agents, it moves as slowly as possible waiting for other agents to come */
      if (Attraction.GetX() < 7) speed1 = speed2 = Lower_limit;
      //check the wheel acceleration limit
      if (Abs(PrevSpeedLeft - speed1) > Change_limit) {
         if (speed1 < PrevSpeedLeft) {
            speed1 = PrevSpeedLeft - Change_limit;
         } else {
            speed1 = PrevSpeedLeft + Change_limit;
         }
      }
      if (Abs(PrevSpeedRight - speed2) > Change_limit) {
         if (speed2 < PrevSpeedRight) {
            speed2 = PrevSpeedRight - Change_limit;
         } else {
            speed2 = PrevSpeedRight + Change_limit;
         }
      }
      //check the wheel speed limit
      if (speed1 > Upper_limit) speed1 = Upper_limit;
      if (speed2 > Upper_limit) speed2 = Upper_limit;
      if (speed1 < Lower_limit) speed1 = Lower_limit;
      if (speed2 < Lower_limit) speed2 = Lower_limit;
      //save the speeds for the next step
      PrevSpeedLeft = speed1;
      PrevSpeedRight = speed2;
      //update info about the mass center velocity
      Velocity = speed1;
      m_fMassCenterVelocity = m_fMassCenterVelocity.Normalize() * Velocity;
      //apply the calculated speeds to the appropriate wheels
      m_pcWheels->SetLinearVelocity(speed1, speed2);
   /* otherwise the wheels move with different speeds and the footbot turns 
      these speeds must make the mass center move with the calculated speed */
   } else {
      Real d1, r, r1, r2, l1, l2;
      d1 = Velocity * STEP_TIME; //distance of mass center in cm
      r = d1 / (2*Sin(Abs(AngleZ))); //curvative raduis of mass center in cm
      r1 = r - 100 * FOOTBOT_RADIUS; //the smaller curvative radius for a slower wheel in cm
      r2 = r + 100 * FOOTBOT_RADIUS; //the greater curvative radius for a faster wheel in cm
      l1 = r1 * 2 * AngleZ.GetAbsoluteValue(); //the length of the shorter arc in cm
      l2 = r2 * 2 * AngleZ.GetAbsoluteValue(); //the length of the longer arc in cm
      speed1 = l1 / STEP_TIME;
      speed2 = l2 / STEP_TIME;
      if (Abs(AngleZ) > CRadians::PI_OVER_TWO) { //turn as sharply as possible
         speed1 = Lower_limit;
         speed2 = Upper_limit;
      }
      /* the next restrictions come from social interaction rules. If the leader observes few 
      other agents, it moves as slowly as possible waiting for other agents to come */
      if (Attraction.GetX() < 7) speed1 = speed2 = Lower_limit;
      if (AngleZ < -ToRadians(m_cAlpha)) { //turn right
         //here speed2 is for the left wheel, speed1 - for the right wheel
         //check the wheel acceleration limit
         if (Abs(PrevSpeedLeft - speed2) > Change_limit) {
            if (speed2 < PrevSpeedLeft) {
               speed2 = PrevSpeedLeft - Change_limit;
            } else {
               speed2 = PrevSpeedLeft + Change_limit;
            }
         }
         if (Abs(PrevSpeedRight - speed1) > Change_limit) {
            if (speed1 < PrevSpeedRight) {
               speed1 = PrevSpeedRight - Change_limit;
            } else {
               speed1 = PrevSpeedRight + Change_limit;
            }
         }
         //check the wheel speed limit
         if (speed2 > Upper_limit) speed2 = Upper_limit;
         if (speed1 > Upper_limit) speed1 = Upper_limit;
         if (speed2 < Lower_limit) speed2 = Lower_limit;
         if (speed1 < Lower_limit) speed1 = Lower_limit;
         //save the speeds for the next step
         PrevSpeedLeft = speed2;
         PrevSpeedRight = speed1;
         //update info about the mass center velocity
         Velocity = (speed1 + speed2) / 2;
         m_fMassCenterVelocity = m_fMassCenterVelocity.Normalize() * Velocity;
         //apply the calculated speeds to the appropriate wheels
         m_pcWheels->SetLinearVelocity(speed2, speed1);
      } else { //turn left
         //here speed1 is for the left wheel, speed2 - for the right wheel
         //check the wheel acceleration limit
         if (Abs(PrevSpeedLeft - speed1) > Change_limit) {
            if (speed1 < PrevSpeedLeft) {
               speed1 = PrevSpeedLeft - Change_limit;
            } else {
               speed1 = PrevSpeedLeft + Change_limit;
            }
         }
         if (Abs(PrevSpeedRight - speed2) > Change_limit) {
            if (speed2 < PrevSpeedRight) {
               speed2 = PrevSpeedRight - Change_limit;
            } else {
               speed2 = PrevSpeedRight + Change_limit;
            }
         }
         //check the wheel speed limit
         if (speed1 > Upper_limit) speed1 = Upper_limit;
         if (speed2 > Upper_limit) speed2 = Upper_limit;
         if (speed1 < Lower_limit) speed1 = Lower_limit;
         if (speed2 < Lower_limit) speed2 = Lower_limit;
         //save the speeds for the next step
         PrevSpeedLeft = speed1;
         PrevSpeedRight = speed2;
         //update info about the mass center velocity
         Velocity = (speed1 + speed2) / 2;
         m_fMassCenterVelocity = m_fMassCenterVelocity.Normalize() * Velocity;
         //apply the calculated speeds to the appropriate wheels
         m_pcWheels->SetLinearVelocity(speed1, speed2);
      }
   }
}

/****************************************/
/****************************************/

CVector3 CFootBotFlocking::TargetForce() {
   CVector3 Coordinates = m_pcPosition->GetReading().Position; //get the current position
   CVector3 Direction = m_fTargetCenter - Coordinates; //Direction points towards the target
   //if the leader is close enough to the target, it is going to stop
   if (Direction.Length() < m_fTargetRadius) {
      m_pcLEDs->SetAllColors(CColor::YELLOW); //become all yellow (a sign for others)
      m_pcLEDs->SetSingleColor(12, CColor::GREEN); //set one blob to green to remain a leader
      return CVector3::ZERO; //no force in this case
   }
   Direction = Direction.Normalize() * 100; //the force is constant
   return Direction;
}

/****************************************/
/****************************************/

CVector3 CFootBotFlocking::RepulsionForce() {
   //the leader repulsion force is identical to one of an ordinary robot
   CVector3 Accumulator; //Accumulator characterizes the obstacles around, now it is a zero vector
   m_pcDistSens->Update(); //update information from distance scanner sensor
   size_t Readings = 0; //the counter of considerable readings
   //get readings (in meters) from distance scanner
   const CFootBotDistanceScannerRotZOnlySensor::TReadingsMap& DistReads = m_pcDistSens->GetReadingsMap();
   for (CCI_FootBotDistanceScannerSensor::TReadingsMap::const_iterator it = DistReads.cbegin(); it != DistReads.cend(); ++it) {
      //the reading is considered if the corresponding scanner ray has detected an obstacle close enough
      if (it->second > 0 && it->second <= OBST_RADIUS) {
         CRadians Angle = it->first;
         Real Dist = it->second;
         Angle.UnsignedNormalize(); //transfer Angle from [-pi; pi] scale to [0; 2pi] scale
         //add the force vector to Accumulator
         Accumulator += CVector3((1 / Dist - 1 / OBST_RADIUS) / Square(Dist), CRadians::PI_OVER_TWO, Angle);
         ++Readings; //increase the counter
      }
   }
   //remove inaccuracy after computations in Z coordinate, so that Accumulator is fully in XY plane
   Accumulator.SetZ(0);
   //average Accumulator
   if (Readings > 0) Accumulator /= Readings;
   //the resulting repulsion force is directed from the obstacles and is opposite to Accumulator
   return -30 * Accumulator;
}

/****************************************/
/****************************************/

CVector3 CFootBotFlocking::SocialForce() {
   m_pcCamera->Update(); //update information from the camera
   //get the camera readings (in cm)
   const CColoredBlobOmnidirectionalCameraRotZOnlySensor::SReadings& sReadings = m_pcCamera->GetReadings();
   int number = sReadings.BlobList.size(); //a number of seen blobs
   if (sReadings.BlobList.size() == 0) { 
      ++ticks_counter; 
   } else { 
      ticks_counter = 0;
      got_lost = false;
   }
   if (ticks_counter > 30) {
      if (!got_lost) {
         got_lost = true;
         ++lost_counter;
      }
   }
   //the resulting vector just keeps the number of seen blobs in its first coordinate
   //it does not have any geometric sense (however can always be implemented)
   return number * CVector3::X;
}

/****************************************/
/****************************************/

void CFootBotFlocking::Destroy()
{
   CVector3 Position = m_pcPosition->GetReading().Position;
   std::cout << "leader: I got lost " << lost_counter << " times and I finish in " << Position << std::endl;
}
/*
 * This statement notifies ARGoS of the existence of the controller.
 * It binds the class passed as first argument to the string passed as second argument.
 * The string is then usable in the XML configuration file to refer to this controller.
 * When ARGoS reads that string in the XML file, it knows which controller class to instantiate.
 * See also the XML configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CFootBotFlocking, "footbot_flocking_controller")
