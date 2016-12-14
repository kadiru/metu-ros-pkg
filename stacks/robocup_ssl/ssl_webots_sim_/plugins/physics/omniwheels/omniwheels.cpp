#include <ode/ode.h>
#include <plugins/physics.h>

#include "iostream"
#include "vector"
#include "string"
#include "sstream"

// it is crucial that the WHEEL_DEF indices start from "0" in the wbt file.
// and wheels are defined as starting with "WHEEL".
// e.g. WHEEL0, WHEEL1, WHEEL2, WHEEL3 etc.
static const std::string WHEEL_DEF = "WHEEL";
static const std::string FIELD_DEF = "FIELD";

static const int BLUE_TEAM = 0;
static const int YELLOW_TEAM = 1;

//it is also important to have team names as
// "B" for blue, "Y" for yellow team.
// hence robot defs are to be as B0, B1, ..., Y0, Y1, etc.
static const std::string BLUE = "B";
static const std::string YELLOW = "Y";
static const uint MAX_TEAM_SIZE = 11;

#define MAX_CONTACTS 10

static dWorldID world = NULL;
static dSpaceID space = NULL;
static dJointGroupID contact_joint_group = NULL;

static dBodyID ground_body;
static dGeomID ground_geom;

static int n_contacts;
static dContact contacts[MAX_CONTACTS];
static dJointID contact_joints[MAX_CONTACTS];

struct CollisionRobot
{
  dBodyID robot_body;
  //std::vector<dBodyID> robot_wheels;
  std::vector<dGeomID> robot_wheels_geom;
  std::vector<dBodyID> robot_wheels_body;
  int team;
  CollisionRobot (std::string robot_def)
  {
    if (robot_def[0] == 'B')
      team = BLUE_TEAM;
    else
      team = YELLOW_TEAM;

    robot_body = dWebotsGetBodyFromDEF(robot_def.c_str());
    if (robot_body != NULL)
    {
      int n_wheels = 0;

      //get the wheels for this robot definition from the world tree
      while (true)
      {
        std::stringstream s;
        s << n_wheels;
        std::string wheel_def = robot_def + "." + WHEEL_DEF + s.str ();
        dGeomID wheel_geom_id = dWebotsGetGeomFromDEF(wheel_def.c_str());
        dBodyID wheel_body_id = dWebotsGetBodyFromDEF(wheel_def.c_str());

        if (wheel_geom_id != NULL && wheel_body_id != NULL)
        {
          robot_wheels_geom.push_back (wheel_geom_id);
          robot_wheels_body.push_back (wheel_body_id);
          n_wheels++;
        }
        else
          break;
      }
    }
  }

  uint8
  getNWheels ()
  {
    return robot_wheels_geom.size ();
  }

  dReal*
  getPosition ()
  {
    return (dReal*)dBodyGetPosition (robot_body);
  }

  dReal*
  getWheelPosition (uint wheel_id)
  {
    if (wheel_id > robot_wheels_geom.size ())
    {
      //no such wheel
      exit (-1);
    }
    return (dReal*)dGeomGetPosition (robot_wheels_geom[wheel_id]);
  }

  double
  getOrientation ()
  {
    dReal* rot = (dReal*)dBodyGetRotation (robot_body);

    if (rot[10] > 0)//if rotated around +z
      return atan2 (rot[4], rot[0]);
    else
      //if rotated around -z
      return -atan2 (rot[4], rot[0]);
    //    return 0;
  }

  //returns wheel axis orientation in global coordinate system
  //which is to used to calculate fDir1 etc. for collision handling
  double
  getWheelAxisOrientation (const uint wheel_id)
  {
    dReal* robot_center = getPosition ();
    dReal* wheel_center = getWheelPosition (wheel_id);
    return atan2 (wheel_center[1] - robot_center[1], wheel_center[0] - robot_center[0]);
    //    return 0;
  }
};

static std::vector<CollisionRobot> blue_team;
static std::vector<CollisionRobot> yellow_team;

void
webots_physics_init (dWorldID w, dSpaceID s, dJointGroupID j)
{
  world = w;
  space = s;
  contact_joint_group = j;

  //get blue team robots
  for (uint i = 0; i < MAX_TEAM_SIZE; i++)
  {
    std::stringstream s;
    s << (int)i;
    std::string robot_name = BLUE + s.str ();
    CollisionRobot robot (robot_name);

    //get all the robots being created that have proper names and wheels
    if (robot.robot_body != NULL && robot.getNWheels () > 0)
      blue_team.push_back (robot);
  }

  //get yellow team robots
  for (uint i = 0; i < MAX_TEAM_SIZE; i++)
  {
    std::stringstream s;
    s << (int)i;
    std::string robot_name = YELLOW + s.str ();
    CollisionRobot robot (robot_name);

    //get all the robots being created that have proper names and wheels
    if (robot.robot_body != NULL && robot.getNWheels () > 0)
      yellow_team.push_back (robot);
  }

  ground_body = dWebotsGetBodyFromDEF("FIELD");
  ground_geom = dWebotsGetGeomFromDEF("FIELD");
}

void
webots_physics_step ()
{
  /*
   * Do here what needs to be done at every time step, e.g. add forces to bodies
   *   dBodyAddForce(body1, f[0], f[1], f[2]);
   *   ...
   */
  char buffer[80];
  //  sprintf (buffer, "%d\n", n_contacts);
  //	dWebotsConsolePrintf(buffer);

  if (ground_body != NULL)
  {
//    sprintf (buffer, "%f %f %f \n", dBodyGetPosition (ground_body)[0], dBodyGetPosition (ground_body)[1],
//             dBodyGetPosition (ground_body)[2]);
//    dWebotsConsolePrintf(buffer);
  }
  else
  {
//    sprintf (buffer, "no field body !!! \n");
//    dWebotsConsolePrintf(buffer);
  }
}

void
webots_physics_draw ()
{
}

int
webots_physics_collide (dGeomID g1, dGeomID g2)
{
  //check the wheel friction between the wheels and the field plane
  //hence check if g1 or g2 is field first,
  //then check if other object is a wheel
  //get the robot id and the wheel id of this wheel

  dGeomID field_geom = NULL;
  dGeomID wheel_geom = NULL;

  if (g1 == ground_geom)
  {
    field_geom = g1;
    wheel_geom = g2;//possible wheel geom
  }
  else if (g2 == ground_geom)
  {
    field_geom = g2;
    wheel_geom = g1;//possible wheel geom
  }
  else
    //collision is not handled here since it is not a field-wheel collision (no field collision)
    return 0;

  //now check if the wheel_geom is really a wheel geom
  bool wheel_found = false;
  uint8 team_id = BLUE_TEAM;
  uint8 player_id = 0;
  uint8 wheel_id = 0;

  //blue team
  for (uint8 i = 0; i < blue_team.size () && !wheel_found; i++)
  {
    for (uint8 j = 0; j < blue_team[i].getNWheels (); j++)
    {
      if (blue_team[i].robot_wheels_geom[j] == wheel_geom)
      {
        wheel_found = true;
        player_id = i;
        wheel_id = j;
        break;
      }
    }
  }

  //yellow team
  for (uint8 i = 0; i < yellow_team.size () && !wheel_found; i++)
  {
    for (uint8 j = 0; j < yellow_team[i].getNWheels (); j++)
    {
      if (yellow_team[i].robot_wheels_geom[j] == wheel_geom)
      {
        wheel_found = true;
        team_id = YELLOW_TEAM;
        player_id = i;
        wheel_id = j;
        break;
      }
    }
  }

  if (!wheel_found)//collision is not handled here since it is not a field-wheel collision (now wheel collision)
    return 0;

  char buffer[80];
  std::vector<CollisionRobot>* team;
  if (team_id == BLUE_TEAM)
    team = &blue_team;
  else
    team = &yellow_team;

  double wheel_axial_orientation = (*team)[player_id].getWheelAxisOrientation (wheel_id) + M_PI_2;
  //  sprintf (buffer, "wheel_id: %d\t axial_rot: %f\n ", wheel_id, wheel_axial_orientation);

  // see how many collision points there are between these objects
  n_contacts = dCollide (g1, g2, MAX_CONTACTS, &contacts[0].geom, sizeof(dContact));
  uint8 contact_id = 0;
  for (; contact_id < n_contacts; contact_id++)
  {
    // custom parameters for creating the contact joint
    // remove or tune these contact parameters to suit your needs
    contacts[contact_id].surface.mode = dContactApprox1 | dContactMu2 | dContactFDir1 | dContactSoftCFM;
    //    contacts[contact_id].surface.mu = 0.05f;
    //    contacts[contact_id].surface.mu2 = 2.0f;
    contacts[contact_id].surface.mu = 2.0f;
    contacts[contact_id].surface.mu2 = 0.05f;
    contacts[contact_id].surface.soft_cfm = 0.002;
    contacts[contact_id].fdir1[0] = cos (wheel_axial_orientation);
    contacts[contact_id].fdir1[1] = sin (wheel_axial_orientation);
    contacts[contact_id].fdir1[2] = 0;
    contacts[contact_id].fdir1[3] = 0;
    dSafeNormalize3(contacts[contact_id].fdir1);//it is already normalized but anyway

//    sprintf (buffer, "wheel_id: %d\t contact_id: %d\t fdir_x: %f\t fdir_y: %f\t fdir_z: %f\n ", wheel_id, contact_id,
//             contacts[contact_id].fdir1[0], contacts[contact_id].fdir1[1], contacts[contact_id].fdir1[2]);
//
//    dWebotsConsolePrintf(buffer);

    // create a contact joint that will prevent the two bodies from intersecting
    // note that contact joints are added to the contact_joint_group
    contact_joints[contact_id] = dJointCreateContact (world, contact_joint_group, &contacts[contact_id]);

    // attach joint between the body and the static environment (0)
    dJointAttach (contact_joints[contact_id], (*team)[player_id].robot_wheels_body[wheel_id], 0);

  }
  return 1;// collision was handled above
}

void
webots_physics_cleanup ()
{
}
