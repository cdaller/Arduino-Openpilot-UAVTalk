//
//
//


#define npanels		2			    // # of possible panels


static uint8_t      panel = 0;                      // active panel: 0 = first panel. 1 = second panel, 2 = panel off

static int16_t      chan1_raw = 0;
static int16_t      chan2_raw = 0;

static uint8_t      ch_toggle = 0;
static boolean      switch_mode = false;

static uint8_t      overspeed = 0;
static uint8_t      stall = 0;

static uint8_t      battv = 0;                      // Battery warning voltage - units Volt *10
static float        osd_vbat_A = 0;                 // Battery A voltage in milivolt
static int16_t      osd_curr_A = 0;                 // Battery A current
static int8_t       osd_battery_remaining_A = 0;    // 0 to 100 <=> 0 to 1000
static uint8_t      batt_warn_level = 0;

static uint8_t      osd_mode = 0;                   // FlightMode of the FlightControl

static uint8_t      osd_satellites_visible = 0;     // number of satelites
static uint8_t      osd_fix_type = 0;               // GPS lock 0-1=no fix, 2=2D, 3=3D
static float        osd_lat = 0;                    // latitude
static float        osd_lon = 0;                    // longitude
static float        osd_alt = 0;                    // altitude
static float        osd_climb = 0;                  // climb rate
static float        osd_groundspeed = 0;            // ground speed
static float        osd_travel_distance = 0;        // travel distance
static float        osd_heading = 0;                // ground course heading

static int16_t      osd_roll = 0;                   // roll from FC
static int16_t      osd_pitch = 0;                  // pitch from FC
static int16_t      osd_yaw = 0;                    // relative heading form FC
static uint16_t     osd_throttle = 0;               // throttle

static uint8_t      osd_alt_cnt = 0;                // counter for stable osd_alt
static float        osd_alt_prev = 0;               // previous altitude
static uint8_t      osd_got_home = 0;               // tells if got home position or not
static float        osd_home_lat = 0;               // home latitude
static float        osd_home_lon = 0;               // home longitude
static float        osd_home_alt = 0;               // home altitude
static long         osd_home_distance = 0;          // distance from home
static uint8_t      osd_home_direction;             // arrow direction pointing to home (1-16 to CW loop)

// OpenPilot UAVTalk:
static uint8_t      op_uavtalk_mode = 0;            // OP UAVTalk mode, start with normal behavior
static uint8_t      op_alarm = 0;                   // OP alarm info
static uint8_t      osd_armed = 0;                  // OP armed info
static uint8_t      osd_time_hour = 0;              // OP GPS time hour info
static uint8_t      osd_time_minute = 0;            // OP GPS tiem minute info
#ifdef REVO_ADD_ONS
static int16_t      revo_baro_alt = 0;              // Revo baro altitude
static int8_t       oplm_rssi = 0;                  // OPLM RSSI
static uint8_t      oplm_linkquality = 0;           // OPLM LinkQuality
#endif

// Flight Batt on MinimOSD:
static int          volt_div_ratio = 0;             // Volt * 100
static int          curr_amp_per_volt = 0;          // Ampere * 100
static int          curr_amp_offset = 0;            // Ampere * 10000
// Flight Batt on MinimOSD and Revo
static uint16_t     osd_total_A = 0;                // Battery total current [mAh]
// Flight Batt on Revo
static uint16_t     osd_est_flight_time = 0;        // Battery estimated flight time [sec]


// Panel BIT registers
byte panA_REG[npanels] = {0b00000000};
byte panB_REG[npanels] = {0b00000000};
byte panC_REG[npanels] = {0b00000000};
byte panD_REG[npanels] = {0b00000000};
byte panE_REG[npanels] = {0b00000000};


// First 8 panels and their X,Y coordinate holders
byte panCenter_XY[2][npanels];
byte panPitch_XY[2][npanels];
byte panRoll_XY[2][npanels];
byte panBatt_A_XY[2][npanels];
//byte panBatt_B_XY[2];
byte panGPSats_XY[2][npanels];
byte panGPL_XY[2][npanels];
byte panGPS_XY[2][npanels];
byte panBatteryPercent_XY[2][npanels];


//Second 8 set of panels and their X,Y coordinate holders
byte panRose_XY[2][npanels];
byte panHeading_XY[2][npanels];
byte panMavBeat_XY[2][npanels];
byte panHomeDir_XY[2][npanels];
byte panHomeDis_XY[2][npanels];
byte panWPDir_XY[2][npanels];
byte panWPDis_XY[2][npanels];
byte panTime_XY[2][npanels];


// Third set of panels and their X,Y coordinate holders
byte panCur_A_XY[2][npanels];
//byte panCur_B_XY[2];
byte panAlt_XY[2][npanels];
byte panHomeAlt_XY[2][npanels];
byte panVel_XY[2][npanels];
byte panAirSpeed_XY[2][npanels];
byte panThr_XY[2][npanels];
byte panFMod_XY[2][npanels];
byte panHorizon_XY[2][npanels];

// Third set of panels and their X,Y coordinate holders
byte panWarn_XY[2][npanels];
//byte panOff_XY[2];
byte panWindSpeed_XY[2][npanels];
byte panClimb_XY[2][npanels];
byte panTune_XY[2][npanels];
//byte panSetup_XY[2];
byte panRSSI_XY[2][npanels];
byte panDistance_XY[2][npanels];


//*************************************************************************************************************
// rssi variables
static uint8_t      rssipersent = 0;
static uint8_t      rssical = 0;
static uint8_t      osd_rssi = 0;		// value from PacketRxOk or analogRSSI
static int16_t      rssi = -99;			// scaled value 0-100%
static boolean      rssiraw_on = false;		// 0- display scale value | 1- display raw value
static uint8_t      rssi_warn_level = 0;

// raw channel variables
static uint16_t     osd_chan5_raw = 1000;
static uint16_t     osd_chan6_raw = 1000;
static uint16_t     osd_chan7_raw = 1000;
static uint16_t     osd_chan8_raw = 1000;

