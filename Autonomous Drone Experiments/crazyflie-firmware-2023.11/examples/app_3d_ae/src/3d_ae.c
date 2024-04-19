#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#include "radiolink.h"
#include "configblock.h"

#define DEBUG_MODULE "P2P"
#include "debug.h"
#include "log.h"
#include "param.h"
#include "crtp_commander_high_level.h"
#include "commander.h"
#include "pptraj.h"
#include "estimator_kalman.h"
#include "deck_constants.h"

// Flocking algorithm params
double alpha = 2.0;
double beta = 1.0;
double gama = 1.0;
double kappa = 0.0;
double sb = 0.3;
double sv = 0.5;
double su = 0.3;
double umax = 0.3;
double wmax = 1.5708;
double K1 = 0.08;
double K2 = 0.2;
double epsilon = 12.0;
double dt = 0.05;

uint8_t alpha_tmp;
uint8_t beta_tmp;
uint8_t kappa_tmp;
uint8_t sb_tmp;
uint8_t sv_tmp;
uint8_t K1_tmp;
uint8_t K2_tmp;;
uint8_t lmx_tmp;
uint8_t lmn_tmp;
uint8_t fh_tmp;
uint8_t wmax_tmp;
uint8_t umax_tmp;
uint8_t u_add_tmp;

// Boundary repulsion
double rx = 0.0;
double ry = 0.0;
double rz = 0.0;
double _temp_rep = 0.0;

// Neighbourhood
double neg_xs[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double neg_ys[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0};
double neg_zs[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0};
double neg_hxcs[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0};
double neg_hycs[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0};
double neg_hzcs[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0};
double self_pos[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
bool neg_alive[] = {false, false, false, false, false, false, false, false, false, false, false};

// General variables
double distance = 0.0;
double distance_x = 0.0;
double distance_y = 0.0;
double distance_z = 0.0;
double ij_ang_x = 0.0;
double ij_ang_y = 0.0;
double ij_ang_z = 0.0;
double fh = 0.5;

// Heading alignment
double hx = 0.0;
double hy = 0.0;
double hz = 0.0;
double sum_hxc = 0.0;
double sum_hyc = 0.0;
double sum_hzc = 0.0;

// Proximal force
double px = 0.0;
double py = 0.0;
double pz = 0.0;
double temp_px = 0.0;
double temp_py = 0.0;
double temp_pz = 0.0;

// Total Force
double fx = 0.0;
double fy = 0.0;
double fz = 0.0;
double f_mag = 0.0;
double dot_fh = 0.0;
double cos_dot_fh = 0.0;
double ang_fh = 0.0;
double h_m = 0.0;

// Goal force
double gx = 0.0;
double gy = 0.0;

// Linear and Angular Velocity
double u = 0.0;
double u_add = 0.05;
double w = 0.0;

// Velocity
double vx = 0.0;
double vy = 0.0;
double vz = 0.0;

// Light sensor
float lmx = 450;
float lmn = 50;

// Offset for lighthouse
float f_offset_x = 3.09;
float f_offset_y = 2.2;

// Total flight time
double total_flight = 0.0;

// Temporary variables
static float self_x;
static float self_y;
static float self_z;

static paramVarId_t paramIdCommanderEnHighLevel;
static paramVarId_t paramIdResetKalman;
static paramVarId_t paramIdiftakeoff;
static paramVarId_t paramIdifterminate;
static paramVarId_t paramIdifland;
static paramVarId_t paramIdifheading;
static paramVarId_t paramIdfmode;
static paramVarId_t paramIdgoalx;
static paramVarId_t paramIdgoaly;

static paramVarId_t paramIdUpdateParams;
static paramVarId_t paramIdAlpha;
static paramVarId_t paramIdBeta;
static paramVarId_t paramIdKappa;
static paramVarId_t paramIdSb;
static paramVarId_t paramIdSv;
static paramVarId_t paramIdK1;
static paramVarId_t paramIdK2;
static paramVarId_t paramIdFh;
static paramVarId_t paramIdLmn;
static paramVarId_t paramIdLmx;
static paramVarId_t paramIdFh;
static paramVarId_t paramIdWmax;
static paramVarId_t paramIdUmax;
static paramVarId_t paramIdUadd;

static void resetKalman() { paramSetInt(paramIdResetKalman, 1); }
static void enableHighlevelCommander() { paramSetInt(paramIdCommanderEnHighLevel, 1); }

// Coordinates struct
typedef struct {
  uint8_t id;
  uint8_t x;
  uint8_t y;
  uint8_t z;
  uint8_t hx;
  uint8_t hy;
  uint8_t hz;
  } _coords;

// Agg. data structure
typedef struct {
  uint64_t data_aggregated;
  double x;
  double y;
  double h;
  double light;
  } _data_to_agg;

// Struct to be used to return rotated heading vector
  typedef struct {
    double vr_x;
    double vr_y;
    double vr_z;
  } _vrot_struct;

// State machine
typedef enum {
    idle,
    takingOff,
    onAir,
    land,
    terminate,
} State;

static State state = idle;


void p2pcallbackHandler(P2PPacket *p)
{
    // Defining what to do when a packet is received
  _coords other_coords;
  memcpy(&other_coords, p->data, sizeof(other_coords));
  uint8_t other_id = other_coords.id;
  neg_alive[other_id - 1] = true;

  // Store received coordinates
  uint8_t other_X = other_coords.x;
  uint8_t other_Y = other_coords.y;
  uint8_t other_Z = other_coords.z;
  uint8_t other_HX = other_coords.hx;
  uint8_t other_HY = other_coords.hy;
  uint8_t other_HZ = other_coords.hz;
  neg_xs[other_id - 1] = (double)other_X * 6 / 255;
  neg_ys[other_id - 1] = (double)other_Y * 4 / 255;
  neg_zs[other_id - 1] = (double)other_Z * 1.5 / 255;
//  neg_hxcs[other_id - 1] = (double)other_HX * 1 / 255;
//  neg_hycs[other_id - 1] = (double)other_HY * 1 / 255;
//  neg_hzcs[other_id - 1] = (double)other_HZ * 1 / 255;
    neg_hxcs[other_id - 1] = ((double)other_HX - 127) / 127;
    neg_hycs[other_id - 1] = ((double)other_HY - 127) / 127;
    neg_hzcs[other_id - 1] = ((double)other_HZ - 127) / 127;
//  DEBUG_PRINT("other_HX: %f, other_HY: %f, other_HZ: %f \n", (double)other_HX , (double)other_HY, (double)other_HZ);
//  DEBUG_PRINT("Neg ID: %f, X: %f, Y: %f, Z: %f, hx: %f, hy: %f, hz: %f \n", (double)other_id ,neg_xs[other_id - 1], neg_ys[other_id - 1], neg_zs[other_id - 1], neg_hxcs[other_id - 1], neg_hycs[other_id - 1], neg_hzcs[other_id - 1] );
}

// This function is used to send velocity commands since the original set VelCmd does
// not have altitude control integrated and needs to be send with vz.
// SetHoverSetpoint has altitude control integrated
static void setVelCmd(setpoint_t *setpoint, float vx, float vy, float vz)
{

  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeVelocity;
  setpoint->mode.z = modeVelocity;

  setpoint->velocity.x = vx;
  setpoint->velocity.y = vy;
  setpoint->velocity.z = vz;

  setpoint->mode.yaw = modeVelocity;
  setpoint->attitudeRate.yaw = 0.0;
//  setpoint->velocity_body = true;

}

uint64_t aggregate_data(double x, double y, double h, double l)
{
  /*
  Aggregation function to send relevant data in a single int.
  Example: x:5.0 y:1.0 heading(yaw):2.5 light_intensity:120
  This becomes -> 501025120
  */
  if (h < 0)
  {
    h = h + 6.28;
  }

  x = round(x * 10) / 10;
  y = round(y * 10) / 10;
  h = round(h * 10) / 10;
  l = round(l * 10) / 10;

  x = x * pow(10, 8);
  y = y * pow(10, 6);
  h = h * pow(10, 4);

  double t;
  t = x + y + h + l;

  uint64_t c_data_aggregated;
  c_data_aggregated = (uint64_t)t;

  return c_data_aggregated;
}

_vrot_struct rotate_vector(double hx, double hy, double hz, double fx, double fy, double fz, double wdt)
{
  /*
    Rotate heading vector on the plane defined on heading and total force vector, in the amount of wdt per step
  */

  static double hmag;
  static double fmag;
  static double nmag;
  static double nx;
  static double ny;
  static double nz;
  static double kc_x;
  static double kc_y;
  static double kc_z;
  static double dp;
  _vrot_struct return_vrot;
//  if (wdt == 0) {wdt = 0.001;}

  hmag = sqrt((hx*hx) + (hy*hy) + (hz*hz));
  fmag = sqrt((fx*fx) + (fy*fy) + (fz*fz));

  fx = (fx * hmag) / fmag;
  fy = (fy * hmag) / fmag;
  fz = (fz * hmag) / fmag;

  nx = hy * fz - hz * fy;
  ny = hz * fx - hx * fz;
  nz = hx * fy - hy * fx;

  nmag = sqrt((nx*nx) + (ny*ny) + (nz*nz));

  if (nmag > 0)
  {
    nx = nx / nmag;
    ny = ny / nmag;
    nz = nz / nmag;
  }

  if (nmag <= 0)
  {
    nx = nx / 1;
    ny = ny / 1;
    nz = nz / 1;
  }

  kc_x = ny * hz - nz * hy;
  kc_y = nz * hx - nx * hz;
  kc_z = nx * hy - ny * hx;

  dp = nx * hx + ny * hy + nz * hz;

  return_vrot.vr_x = hx * cos(wdt) + kc_x * sin(wdt) + nx * dp * (1 - cos(wdt));
  return_vrot.vr_y = hy * cos(wdt) + kc_y * sin(wdt) + ny * dp * (1 - cos(wdt));
  return_vrot.vr_z = hz * cos(wdt) + kc_z * sin(wdt) + nz * dp * (1 - cos(wdt));

  return return_vrot;
}

void appMain()
{
    static setpoint_t setpoint;
    static float heading_log;
    heading_log = 99;
    static uint8_t iftakeoff;
    static uint8_t ifterminate;
    static uint8_t ifheading;
    static uint8_t ifupdateParams;
    static uint8_t ifland;
    static uint8_t _fmode;
    static float _goal_x;
    static float _goal_y;
    _fmode = 2;
    _goal_x = 3.0;
    _goal_y = 2.25;
    iftakeoff = 0;
    ifterminate = 0;
    ifland = 0;
    ifheading = 1;
    ifupdateParams = 0;

    float a_read = 0.0f;
//    adcInit();
    static float light_log;
    light_log = 0;
    static int32_t agg_data_log = 99999999;
    static uint8_t smy_id;
    static uint8_t log_pos_x = 0;
    static uint8_t log_pos_y = 0;
    static uint8_t log_pos_z = 0;
    static uint8_t log_hxc = 0;
    static uint8_t log_hyc = 0;
    static uint8_t log_hzc = 0;


    // Setting Ids for logging
    logVarId_t idX = logGetVarId("stateEstimate", "x");
    logVarId_t idY = logGetVarId("stateEstimate", "y");
    logVarId_t idZ = logGetVarId("stateEstimate", "z");

    static P2PPacket p_reply;
    p_reply.port=0x00;

    uint64_t address = configblockGetRadioAddress();
    uint8_t my_id =(uint8_t)((address) & 0x00000000ff);
    p2pRegisterCB(p2pcallbackHandler);
    _coords self_coords;
    self_coords.id = my_id;
    smy_id = my_id;


    // Crazyflie logging groups
    //  -> Light, heading and agg data
    LOG_GROUP_START(synthLog)
    LOG_ADD_CORE(LOG_FLOAT, heading, &heading_log)
    LOG_ADD_CORE(LOG_FLOAT, light_intensity, &light_log)
    LOG_ADD_CORE(LOG_UINT8, agent_id, &smy_id)
    LOG_ADD_CORE(LOG_UINT8, log_pos_x, &log_pos_x)
    LOG_ADD_CORE(LOG_UINT8, log_pos_y, &log_pos_y)
    LOG_ADD_CORE(LOG_UINT8, log_pos_z, &log_pos_z)
    LOG_ADD_CORE(LOG_UINT8, log_hxc, &log_hxc)
    LOG_ADD_CORE(LOG_UINT8, log_hyc, &log_hyc)
    LOG_ADD_CORE(LOG_UINT8, log_hzc, &log_hzc)
    LOG_GROUP_STOP(synthLog)

    // -> Flight modes
    PARAM_GROUP_START(fmodes)
    PARAM_ADD_CORE(PARAM_UINT8, if_takeoff, &iftakeoff)
    PARAM_ADD_CORE(PARAM_UINT8, if_terminate, &ifterminate)
    PARAM_ADD_CORE(PARAM_UINT8, if_heading, &ifheading)
    PARAM_ADD_CORE(PARAM_UINT8, if_land, &ifland)
    PARAM_ADD_CORE(PARAM_UINT8, fmode, &_fmode)
    PARAM_ADD_CORE(PARAM_FLOAT, goal_x, &_goal_x)
    PARAM_ADD_CORE(PARAM_FLOAT, goal_y, &_goal_y)
    PARAM_GROUP_STOP(fmodes)

    // -> Parameters for experiments behaviours
    PARAM_GROUP_START(flockParams)
    PARAM_ADD_CORE(PARAM_UINT8, update_params, &ifupdateParams)
    PARAM_ADD_CORE(PARAM_UINT8, fh_param, &fh_tmp)
    PARAM_ADD_CORE(PARAM_UINT8, alpha_param, &alpha_tmp)
    PARAM_ADD_CORE(PARAM_UINT8, beta_param, &beta_tmp)
    PARAM_ADD_CORE(PARAM_UINT8, sb_param, &sb_tmp)
    PARAM_ADD_CORE(PARAM_UINT8, sv_param, &sv_tmp)
    PARAM_ADD_CORE(PARAM_UINT8, kappa_param, &kappa_tmp)
    PARAM_ADD_CORE(PARAM_UINT8, k1_param, &K1_tmp)
    PARAM_ADD_CORE(PARAM_UINT8, k2_param, &K2_tmp)
    PARAM_ADD_CORE(PARAM_UINT8, lmn_param, &lmn_tmp)
    PARAM_ADD_CORE(PARAM_UINT8, lmx_param, &lmx_tmp)
    PARAM_ADD_CORE(PARAM_UINT8, wmax_param, &wmax_tmp)
    PARAM_ADD_CORE(PARAM_UINT8, umax_param, &umax_tmp)
    PARAM_ADD_CORE(PARAM_UINT8, u_add_param, &u_add_tmp)
    PARAM_GROUP_STOP(flockParams)


    paramIdCommanderEnHighLevel = paramGetVarId("commander", "enHighLevel");
    paramIdResetKalman = paramGetVarId("kalman", "resetEstimation");
    paramIdiftakeoff = paramGetVarId("fmodes", "if_takeoff");
    paramIdifterminate = paramGetVarId("fmodes", "if_terminate");
    paramIdifland = paramGetVarId("fmodes", "if_land");
    paramIdfmode = paramGetVarId("fmodes", "fmode");
    paramIdifheading = paramGetVarId("fmodes", "if_heading");
    paramIdgoalx = paramGetVarId("fmodes", "goal_x");
    paramIdgoaly = paramGetVarId("fmodes", "goal_y");

    paramIdUpdateParams = paramGetVarId("flockParams", "update_params");
    paramIdFh = paramGetVarId("flockParams", "fh_param");
    paramIdAlpha = paramGetVarId("flockParams", "alpha_param");
    paramIdBeta = paramGetVarId("flockParams", "beta_param");
    paramIdSb = paramGetVarId("flockParams", "sb_param");
    paramIdSv = paramGetVarId("flockParams", "sv_param");
    paramIdKappa = paramGetVarId("flockParams", "kappa_param");
    paramIdK1 = paramGetVarId("flockParams", "k1_param");
    paramIdK2 = paramGetVarId("flockParams", "k2_param");
    paramIdLmn = paramGetVarId("flockParams", "lmn_param");
    paramIdLmx = paramGetVarId("flockParams", "lmx_param");
    paramIdWmax = paramGetVarId("flockParams", "wmax_param");
    paramIdUmax = paramGetVarId("flockParams", "umax_param");
    paramIdUadd = paramGetVarId("flockParams", "u_add_param");

    resetKalman();
    enableHighlevelCommander();
    srand((unsigned int)xTaskGetTickCount());
    double test_rand = (double)rand()/(double)(RAND_MAX/6.28);
    test_rand = test_rand + 0.01;
//    DEBUG_PRINT("random is %f\n", test_rand);
    vTaskDelay(M2T(8000));


   TickType_t time_begin=0, time_end=0;
   const TickType_t LOOP_DURATION_TARGET = 50;

  while(1) {
     time_begin = xTaskGetTickCount();
    // Reading the lights from the onboard light sensor
//    a_read = analogRead(DECK_GPIO_TX2);
    a_read = 0;
    light_log = a_read;

    // Getting flag for On-the-fly parameter update
    ifupdateParams = paramGetInt(paramIdUpdateParams);

    if (ifupdateParams == 1) {
      K1 = ((double)paramGetInt(paramIdK1)) / 50;
      K2 = ((double)paramGetInt(paramIdK2)) / 50;
      alpha = ((double)paramGetInt(paramIdAlpha)) / 50;
      beta = ((double)paramGetInt(paramIdBeta)) / 50;
      sb = ((double)paramGetInt(paramIdSb)) / 50;
      sv = ((double)paramGetInt(paramIdSv)) / 50;
      kappa = ((double)paramGetInt(paramIdKappa)) / 50;
      fh = ((double)paramGetInt(paramIdFh)) / 10;
      lmn = ((float)paramGetInt(paramIdLmn)) * 4;
      lmx = ((float)paramGetInt(paramIdLmx)) * 4;
      umax = ((double)paramGetInt(paramIdUmax)) / 100;
      wmax = ((double)paramGetInt(paramIdWmax)) / 10 * 3.14;
      u_add = ((double)paramGetInt(paramIdUadd)) / 100;
      ifupdateParams = 0;
    }

    if (state == idle) {

      if (iftakeoff == 0) {
        // Gets takeoff flag
        iftakeoff = paramGetInt(paramIdiftakeoff);
        vTaskDelay(M2T(100));
      }

      // Take off sequence
      if (iftakeoff == 1) {
        vTaskDelay(M2T(1000));
        crtpCommanderHighLevelTakeoff(0.5, 2.5);
        vTaskDelay(M2T(4000));
        state = takingOff;
        // Angle initilzation!!!
        static double theta;
        theta = (double)rand()/(double)(RAND_MAX/6.28);
        static double phi;
        phi = (double)rand()/(double)(RAND_MAX/3.14);
        self_pos[3] = sin(phi) * cos(theta);
        self_pos[4] = sin(phi) * sin(theta);
        self_pos[5] = cos(phi);
      }
    }

    // House keeping while drone is taking off
    if (state == takingOff) {
      if (crtpCommanderHighLevelIsTrajectoryFinished())
      {state = onAir;}
//      state = onAir;
    }

    // Flying state
    if (state == onAir) {
      // Reseting all the vectors to be computed later
      // Proximal
      px = 0.0;
      py = 0.0;
      pz = 0.0;

      // Alignment
      hx = 0.0;
      hy = 0.0;
      hz = 0.0;

      // Boundary
      rx = 0.0;
      ry = 0.0;
      rz = 0.0;

      // Update of current state
//      DEBUG_PRINT("idx is %.3f\n", (double)logGetFloat(idX) );
      self_x = (logGetFloat(idX) + (float)(f_offset_x)) * (float)(255 / 6);
//      DEBUG_PRINT("selfx is %.3f\n", (double)self_x );
      self_y = (logGetFloat(idY) + (float)(f_offset_y)) * (float)(255 / 4);
//      DEBUG_PRINT("selfy is %.3f\n", (double)self_y );
      self_z = logGetFloat(idZ) * (float)(255 / 1.5);

      if (self_x > 254)
      { self_x = 254;}
      if (self_x < 2)
      { self_x = 2;}
      if (self_y > 254)
      { self_y = 254;}
      if (self_y < 2)
      { self_y = 2;}
      if (self_z > 251)
      { self_z = 251;}
      if (self_z < 5)
      { self_z = 5;}

//      DEBUG_PRINT("selfx clipped is %.3f\n", (double)self_x );
//      DEBUG_PRINT("selfy clipped is %.3f\n", (double)self_y );

      self_coords.x = (uint8_t)(self_x);
//      DEBUG_PRINT("selfcoordx is %u\n", (unsigned int)self_coords.x );
      self_coords.y = (uint8_t)(self_y);
//      DEBUG_PRINT("selfcoordy is %u\n", (unsigned int)self_coords.y );
      self_coords.z = (uint8_t)(self_z);

      self_pos[0] = (double)self_x * 6 / 255;
//      DEBUG_PRINT("selfpos0 is %f\n", self_pos[0] );
      self_pos[1] = (double)self_y * 4 / 255;
//      DEBUG_PRINT("selfpos0 is %f\n", self_pos[1]);
      self_pos[2] = (double)self_z * 1.5 / 255;
      self_coords.hx = (uint8_t)(127 * self_pos[3] + 127);
      self_coords.hy = (uint8_t)(127 * self_pos[4] + 127);
      self_coords.hz = (uint8_t)(127 * self_pos[5] + 127);
//  DEBUG_PRINT("hx: %f, hy: %f, hz: %f \n", (double)self_coords.hx, (double)self_coords.hy, (double)self_coords.hz );
//  DEBUG_PRINT("My ID: %f, X: %f, Y: %f, Z: %f, hx: %f, hy: %f, hz: %f \n", (double)self_coords.id ,self_pos[0], self_pos[1], self_pos[2], self_pos[3], self_pos[4], self_pos[5] );

      heading_log = (float)0.0;

      // light cap
      if (light_log < lmn) {light_log=lmn;}
      if (light_log >lmx) {light_log=lmx;}

      /*
      Fmodes are use to define specific missions.
      For this set of experiments, fmode==2 is the one used
      */
      if (_fmode == 1) {
        // Flocking mode
        kappa = 0.0;
        u_add = 0.05;
      }

      else if (_fmode == 2) {
        // Gradient following mode
        // Equation (2)
//        su = sb + pow((light_log-lmn)/(lmx - lmn), 0.1)*sv;
        su = sb;
        u_add = 0.05;
        kappa = 0.0;
      }

      else if (_fmode == 3) {
        // Go to goal mode
        _goal_x = paramGetFloat(paramIdgoalx);
        _goal_y = paramGetFloat(paramIdgoaly);
        gx = (double)_goal_x - self_pos[0];
        gy = (double)_goal_y - self_pos[1];
        gx = gx / (sqrt(gx*gx + gy*gy));
        gy = gy / (sqrt(gx*gx + gy*gy));
        u_add = 0.05;
      }

      sum_hxc = self_pos[3];
      sum_hyc = self_pos[4];
      sum_hzc = self_pos[5];

      for (int i = 0; i < 10; i++)
      {
        if ( neg_alive[i] && (neg_xs[i] > 0.0) && ((i+1) != my_id) )
        {
          // Computation of interagent distance and bearings
          distance_x = neg_xs[i] - self_pos[0];
          distance_y = neg_ys[i] - self_pos[1];
          distance_z = neg_zs[i] - self_pos[2];
          distance = sqrt( (distance_x * distance_x) +  (distance_y * distance_y) + (distance_z * distance_z));

          ij_ang_x = acos(distance_x / distance);
          ij_ang_y = acos(distance_y / distance);
          ij_ang_z = acos(distance_z / distance);

          // Equation (1) Computing proximal control
          temp_px = ((-epsilon * (( 2.*( pow(sb, 4.) / pow(distance, 5.) ) ) - ( pow(sb, 2.) / pow(distance, 3.) ) ) ) * cos(ij_ang_x) );
          temp_py = ((-epsilon * (( 2.*( pow(sb, 4.) / pow(distance, 5.) ) ) - ( pow(sb, 2.) / pow(distance, 3.) ) ) ) * cos(ij_ang_y) );
          temp_pz = ((-epsilon * (( 2.*( pow(sb, 4.) / pow(distance, 5.) ) ) - ( pow(sb, 2.) / pow(distance, 3.) ) ) ) * cos(ij_ang_z) );

          px = px + temp_px;
          py = py + temp_py;
          pz = pz + temp_pz;


          sum_hxc += neg_hxcs[i];
          sum_hyc += neg_hycs[i];
          sum_hzc += neg_hzcs[i];

        }
      }

      // Equation (3) Alignment control vector
      hx = sum_hxc / (sqrt((sum_hxc*sum_hxc) + (sum_hyc*sum_hyc) + (sum_hzc*sum_hzc)));
      hy = sum_hyc / (sqrt((sum_hxc*sum_hxc) + (sum_hyc*sum_hyc) + (sum_hzc*sum_hzc)));
      hz = sum_hzc / (sqrt((sum_hxc*sum_hxc) + (sum_hyc*sum_hyc) + (sum_hzc*sum_hzc)));

      // Equation (4) Boundary repulsion vector
      if (self_pos[0]<0.5)
      {
        _temp_rep = -epsilon*5 * (2 * (pow(0.05, 4) / pow(fabs(self_pos[0]), 5)) - (pow(0.05, 2) / pow(fabs(self_pos[0]), 5)));
        if (_temp_rep < 0) {_temp_rep = 0;}
        rx += _temp_rep;
      }
      if (self_pos[0]>5.5)
      {
        _temp_rep = -epsilon*5 * (2 * (pow(0.05, 4) / pow(fabs(6.0-self_pos[0]), 5)) - (pow(0.05, 2) / pow(fabs(6.0-self_pos[0]), 3)));
        if (_temp_rep < 0) {_temp_rep = 0;}
        rx += _temp_rep * (-1);
      }
      if (self_pos[1]<0.5)
      {
        _temp_rep = -epsilon*5 * (2 * (pow(0.05, 4) / pow(fabs(self_pos[1]), 5)) - (pow(0.05, 2) / pow(fabs(self_pos[1]), 3)));
        if (_temp_rep < 0) {_temp_rep = 0;}
        ry += _temp_rep;
      }
      if (self_pos[1]>3.5)
      {
        _temp_rep = -epsilon*5 * (2 * (pow(0.05, 4) / pow(fabs(4-self_pos[1]), 5)) - (pow(0.05, 2) / pow(fabs(4-self_pos[1]), 3)));
        if (_temp_rep < 0) {_temp_rep = 0;}
        ry += _temp_rep * (-1);
      }

      if (self_pos[2]<0.5)
      {
        _temp_rep = -epsilon*5 * (2 * (pow(0.05, 4) / pow(fabs(self_pos[2]), 5)) - (pow(0.05, 2) / pow(fabs(self_pos[2]), 3)));
        if (_temp_rep < 0) {_temp_rep = 0;}
        rz += _temp_rep;
      }
      if (self_pos[2]>1.0)
      {
        _temp_rep = -epsilon*5 * (2 * (pow(0.05, 4) / pow(fabs(1.5-self_pos[2]), 5)) - (pow(0.05, 2) / pow(fabs(1.5-self_pos[2]), 3)));
        if (_temp_rep < 0) {_temp_rep = 0;}
        rz += _temp_rep * (-1);
      }


      // Equation (10) Total force vector
      fx = alpha * px + beta * hx + gama * rx;
      if (fx == 0.0) {fx = 0.0001;}
//      DEBUG_PRINT("fx: is %f, px is: %f, hx is %f, rx is %f \n", fx, px, hx, rx );
      fy = alpha * py + beta * hy + gama * ry;
      if (fy == 0.0) {fy = 0.0001;}
//      DEBUG_PRINT("fy: is %f, py is: %f, hy is %f, ry is %f \n", fy, py, hy, ry );
      fz = alpha * pz + beta * hz + gama * rz;
      if (fz == 0.0) {fz = 0.0001;}
//      DEBUG_PRINT("fz: is %f, pz is: %f, hz is %f, rz is %f \n", fz, pz, hz, rz );

      f_mag = sqrt((fx*fx) + (fy*fy) + (fz*fz));

      dot_fh = fx * self_pos[3] + fy * self_pos[4] + fz * self_pos[5];
      cos_dot_fh = dot_fh / (f_mag * sqrt( (self_pos[3]*self_pos[3]) + (self_pos[4]*self_pos[4]) + (self_pos[5]*self_pos[5])));
//      DEBUG_PRINT("cos_dot_fh: is %f \n", cos_dot_fh);
      ang_fh = acos(cos_dot_fh);

      // Equation (11) Linear and Angular speeds
      u = K1 * f_mag * cos(ang_fh);
      w = K2 * f_mag * sin(ang_fh);

      if (u>umax) {u = umax;}
      else if (u<0) {u = 0.0;}

      u = u + 0.05;
//      DEBUG_PRINT("u: is %f \n", u);

      if (w>wmax) {w = wmax;}
      else if (w<-wmax) {w = -wmax;}

      h_m = sqrt((self_pos[3]*self_pos[3]) + (self_pos[4]*self_pos[4]) + (self_pos[5]*self_pos[5]));

     // Linear speed (u) is defined as the velocity in X axis
      vx = (float)(u * (self_pos[3] / h_m));
      vy = (float)(u * (self_pos[4] / h_m));
      vz = (float)(u * (self_pos[5] / h_m));

//      DEBUG_PRINT("vx %.3f\n", (double)vx );
//      DEBUG_PRINT("vy %.3f\n", (double)vy );
//      DEBUG_PRINT("vz %.3f\n", (double)vz );

      // Update the virtual heading
      _vrot_struct new_heading;

      new_heading  = rotate_vector(self_pos[3], self_pos[4], self_pos[5], fx, fy, fz, w*dt);
//      DEBUG_PRINT("poshxc: is %f, poshyc is: %f, poshzc is: %f , wdt is :%f \n", self_pos[3], self_pos[4], self_pos[5], w*dt);
      self_pos[3] = new_heading.vr_x;
      self_pos[4] = new_heading.vr_y;
      self_pos[5] = new_heading.vr_z;
//     DEBUG_PRINT("u: is %f, w is: %f, h_m is %f \n", u, w, h_m);

    log_pos_x = (uint8_t)(self_pos[0] * 255 / 6.5);
    log_pos_y = (uint8_t)(self_pos[1] * 255 / 4.5);
    log_pos_z = (uint8_t)(self_pos[2] * 255 / 2.0);

    log_hxc = (uint8_t)(self_pos[3] * 255);
    log_hyc = (uint8_t)(self_pos[4] * 255);
    log_hzc = (uint8_t)(self_pos[5] * 255);


//       Sending control commands to low level Controllers
      setVelCmd(&setpoint, vx, vy, vz);
      commanderSetSetpoint(&setpoint, 3);

      // Communications stuff
      memcpy(p_reply.data, &self_coords, sizeof(self_coords));
      p_reply.size = sizeof(self_coords)+1;
      radiolinkSendP2PPacketBroadcast(&p_reply);
      ifterminate = paramGetInt(paramIdifterminate);
      ifland = paramGetInt(paramIdifland);
      _fmode = paramGetInt(paramIdfmode);
      ifheading = paramGetInt(paramIdifheading);

      // Data aggregation for next logging
      agg_data_log = aggregate_data(self_pos[0], self_pos[1], 0.0, 0.0);

//      vTaskDelay(M2T(dt*500));

      // Setting a total flight time
      total_flight += dt * 1000;
      if (total_flight > 210000 || ifland == 1) {
        state = land;
      }

      // Terminate immediately
      if (ifterminate == 1) {
        state = terminate;
      }
    }

    // Our defined land sequence.
    if(state == land) {
      int tm;
      tm = 0;

      while (tm < 61)
        {
            setVelCmd(&setpoint, 0.0, 0.0, -0.1);
            commanderSetSetpoint(&setpoint, 3);
            tm += 1;
            vTaskDelay(M2T(50));
        }
      vTaskDelay(M2T(500));
      return;
    }

    time_end = xTaskGetTickCount();
    TickType_t loop_duration = time_end - time_begin;
     if (loop_duration < LOOP_DURATION_TARGET ) {
        vTaskDelay(M2T(LOOP_DURATION_TARGET - loop_duration));
     }
     else
     {
//     DEBUG_PRINT("WARNING! loop took %lu ms, which is more than the target %lu ms\n", loop_duration, LOOP_DURATION_TARGET);
     }
  }
}

