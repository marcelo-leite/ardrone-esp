#ifndef NAVDATA_H   // guardas de cabecalho, impedem inclusoes ciclicas
#define NAVDATA_H

#include <stdint.h>
#define _ATTRIBUTE_PACKED_  __attribute__((packed, aligned(1)))


typedef struct {
  float x;
  float y;
  float z;
}  vector31_t;


typedef struct {
  float x;
  float y;

}  vector21_t;

typedef struct {
  float m11;
  float m12;
  float m13;
  float m21;
  float m22;
  float m23;
  float m31;
  float m32;
  float m33;

}  matrix33_t;

typedef struct {
  int32_t x;
  int32_t y;

}  screen_point_t;

typedef struct{

  uint8_t sat; //  Satellite ID
  uint8_t c_n0; 

} navdata_gps_channel;




//#pragma pack(push, 1)
// ID 0
typedef struct {
  uint32_t    ctrl_state;             /*!< Flying state (landed, flying, hovering, etc.) defined in CTRL_STATES enum. */
  uint32_t    baterry; /*!< battery voltage filtered (mV) */

  float   theta;                   /*!< UAV's pitch in milli-degrees */
  float   phi;                    /*!< UAV's roll  in milli-degrees */
  float   psi;                    /*!< UAV's yaw   in milli-degrees */

  int32_t     altitude;               /*!< UAV's altitude in centimeters */

  float   vx;                     /*!< UAV's estimated linear velocity */
  float   vy;                     /*!< UAV's estimated linear velocity */
  float   vz;                     /*!< UAV's estimated linear velocity */

}_ATTRIBUTE_PACKED_ navdata_demo_t;

// ID1
typedef struct {

  uint32_t  times;  /*!< 32 bit value where the 11 most significant bits represents the seconds, and the 21 least significant bits are the microseconds. */
}_ATTRIBUTE_PACKED_ navdata_time_t;

// ID2
typedef struct {

  uint16_t  raw_accs[3];    // filtered accelerometers
  int16_t   raw_gyros[3];  // filtered gyrometers
  int16_t   raw_gyros_110[2];     // gyrometers  x/y 110 deg/s
  uint32_t  vbat_raw;             // battery voltage raw (mV)
  uint16_t  us_debut_echo;
  uint16_t  us_fin_echo;
  uint16_t  us_association_echo;
  uint16_t  us_distance_echo;
  uint16_t  us_courbe_temps;
  uint16_t  us_courbe_valeur;
  uint16_t  us_courbe_ref;
  uint16_t  flag_echo_ini;
  // TODO:   uint16_t  frame_number; // from ARDrone_Magneto
  uint16_t  nb_echo;
  uint32_t  sum_echo;
  int32_t   alt_temp_raw;
  int16_t   gradient;
}_ATTRIBUTE_PACKED_ navdata_raw_measures_t;


// ID3
typedef struct {

  float   accs_temp;
  uint16_t    gyro_temp;
  vector31_t   phys_accs;
  vector31_t   phys_gyros;
  uint32_t    alim3V3;              // 3.3volt alim [LSB]
  uint32_t    vrefEpson;            // ref volt Epson gyro [LSB]
  uint32_t    vrefIDG;              // ref volt IDG gyro [LSB]
}_ATTRIBUTE_PACKED_ navdata_phys_measures_t;

// ID4
typedef struct {
  vector31_t offset_g;

}_ATTRIBUTE_PACKED_ navdata_gyros_offsets_t;

// ID5
typedef struct {

  float   theta_a;
  float   phi_a;
}_ATTRIBUTE_PACKED_ navdata_euler_angles_t;


// ID6
typedef struct {

  int32_t   ref_theta;
  int32_t   ref_phi;
  int32_t   ref_theta_I;
  int32_t   ref_phi_I;
  int32_t   ref_pitch;
  int32_t   ref_roll;
  int32_t   ref_yaw;
  int32_t   ref_psi;

  float vx_ref;
	float vy_ref;
	float theta_mod;
	float phi_mod;

	float k_v_x;
	float k_v_y;
	uint32_t  k_mode;

	float ui_time;
	float ui_theta;
	float ui_phi;
	float ui_psi;
	float ui_psi_accuracy;
	int32_t ui_seq;

}_ATTRIBUTE_PACKED_ navdata_references_t;





// TODO: depreciated struct ? remove it ?
// ID7
typedef struct {

  float angular_rates_trim_r;
  float euler_angles_trim_theta;
  float euler_angles_trim_phi;
}_ATTRIBUTE_PACKED_ navdata_trims_t;


// ID8
typedef struct {

  int32_t    rc_ref_pitch;
  int32_t    rc_ref_roll;
  int32_t    rc_ref_yaw;
  int32_t    rc_ref_gaz;
  int32_t    rc_ref_ag;
}_ATTRIBUTE_PACKED_ navdata_rc_references_t;

// ID9
typedef struct {

  uint8_t     motor1;
  uint8_t     motor2;
  uint8_t     motor3;
  uint8_t     motor4;
  uint8_t	  sat_motor1;
  uint8_t	  sat_motor2;
  uint8_t	  sat_motor3;
  uint8_t	  sat_motor4;
  float   gaz_feed_forward;
  float   gaz_altitude;
  float   altitude_integral;
  float   vz_ref;
  int32_t     u_pitch;
  int32_t     u_roll;
  int32_t     u_yaw;
  float   yaw_u_I;
  int32_t     u_pitch_planif;
  int32_t     u_roll_planif;
  int32_t     u_yaw_planif;
  float   u_gaz_planif;
  uint16_t    current_motor1;
  uint16_t    current_motor2;
  uint16_t    current_motor3;
  uint16_t    current_motor4;
	//WARNING: new navdata (FC 26/07/2011)
	float 	altitude_prop;
	float 	altitude_der;
}_ATTRIBUTE_PACKED_ navdata_pwm_t;

// ID10
typedef struct {

  int32_t   altitude_vision;
  float altitude_vz;
  int32_t   altitude_ref;
  int32_t   altitude_raw;

	float		obs_accZ;
	float 	obs_alt;
	vector31_t 	obs_x;
	uint32_t 		obs_state;
	vector21_t	est_vb;
	uint32_t 		est_state ;

}_ATTRIBUTE_PACKED_ navdata_altitude_t;


// ID11
typedef struct {

  float vision_tx_raw;
  float vision_ty_raw;
  float vision_tz_raw;
}_ATTRIBUTE_PACKED_ navdata_vision_raw_t;


// ID12
typedef struct {

  float   of_dx[5];
  float   of_dy[5];
}_ATTRIBUTE_PACKED_ navdata_vision_of_t;


// ID13
typedef struct _navdata_vision_t {

  uint32_t   vision_state;
  int32_t    vision_misc;
  float  vision_phi_trim;
  float  vision_phi_ref_prop;
  float  vision_theta_trim;
  float  vision_theta_ref_prop;

  int32_t    new_raw_picture;
  float  theta_capture;
  float  phi_capture;
  float  psi_capture;
  int32_t    altitude_capture;
  uint32_t   time_capture;     // time in TSECDEC format (see config.h)
  vector31_t body_v;

  float  delta_phi;
  float  delta_theta;
  float  delta_psi;

	uint32_t  gold_defined;
	uint32_t  gold_reset;
	float gold_x;
	float gold_y;

}_ATTRIBUTE_PACKED_ navdata_vision_t;


// ID14
typedef struct {

  float  time_szo;
  float  time_corners;
  float  time_compute;
  float  time_tracking;
  float  time_trans;
  float  time_update;
	float  time_custom[20];

}_ATTRIBUTE_PACKED_ navdata_vision_perf_t;



// ID15
typedef struct {

  int32_t locked[30];
  screen_point_t point[30];
  
}_ATTRIBUTE_PACKED_ navdata_trackers_send_t;


// ID16
typedef struct {
	/* !! Change the function 'navdata_server_reset_vision_detect()' if this structure is modified !! */

  uint32_t   nb_detected;
  uint32_t   type[4];
  uint32_t   xc[4];
  uint32_t   yc[4];
  uint32_t   width[4];
  uint32_t   height[4];
  uint32_t   dist[4];
  float  orientation_angle[4];
  matrix33_t rotation[4];
  vector31_t translation[4];
  uint32_t   camera_source[4];
}_ATTRIBUTE_PACKED_ navdata_vision_detect_t;

// ID17
typedef struct {

  // +4 bytes
  int32_t    watchdog;
}_ATTRIBUTE_PACKED_ navdata_watchdog_t;


// ID18
typedef struct {

  uint32_t  version;
  uint8_t   data_frame[32];
}_ATTRIBUTE_PACKED_ navdata_adc_data_frame_t;

// ID19
typedef struct {

  uint8_t 	quant;			// quantizer reference used to encode frame [1:31]
  uint32_t	frame_size;		// frame size (bytes)
  uint32_t	frame_number;	// frame index
  uint32_t	atcmd_ref_seq;  // atmcd ref sequence number
  uint32_t	atcmd_mean_ref_gap;	// mean time between two consecutive atcmd_ref (ms)
  float atcmd_var_ref_gap;
  uint32_t	atcmd_ref_quality; // estimator of atcmd link quality

  // drone2
  uint32_t  out_bitrate;     // measured out throughput from the video tcp socket
  uint32_t  desired_bitrate; // last frame size generated by the video encoder

  // misc temporary data
  int32_t  data1;
  int32_t  data2;
  int32_t  data3;
  int32_t  data4;
  int32_t  data5;

  // queue usage
  uint32_t tcp_queue_level;
  uint32_t fifo_queue_level;

}_ATTRIBUTE_PACKED_ navdata_video_stream_t;


// ID20
typedef struct {
  uint32_t  double_tap_counter;
  uint32_t  finish_line_counter;
}_ATTRIBUTE_PACKED_ navdata_games_t;

// ID21
typedef struct{
  int32_t   up;
  int16_t   ut;
  int32_t   temperature_meas;
  int32_t   pression_meas;
}_ATTRIBUTE_PACKED_ navdata_pressure_raw_t;



// ID22
typedef struct {
  int16_t   	mx;
  int16_t   	my;
  int16_t   	mz;
  vector31_t 	magneto_raw;       // magneto in the body frame, in mG
  vector31_t 	magneto_rectified;
  vector31_t 	magneto_offset;
  float 	heading_unwrapped;
  float 	heading_gyro_unwrapped;
  float 	heading_fusion_unwrapped;
  char 			magneto_calibration_ok;
  uint32_t      magneto_state;
  float 	magneto_radius;
  float     error_mean;
  float     error_var;

}_ATTRIBUTE_PACKED_ navdata_magneto_t;

// ID23
typedef struct {

  float wind_speed;			// estimated wind speed [m/s]
  float wind_angle;			// estimated wind direction in North-East frame [rad] e.g. if wind_angle is pi/4, wind is from South-West to North-East
  float wind_compensation_theta;
  float wind_compensation_phi;
  float state_x1;
  float state_x2;
  float state_x3;
  float state_x4;
  float state_x5;
  float state_x6;
  float magneto_debug1;
  float magneto_debug2;
  float magneto_debug3;
}_ATTRIBUTE_PACKED_ navdata_wind_speed_t;

// ID24
typedef struct {

  float offset_pressure;
  float est_z;
  float est_zdot;
  float est_bias_PWM;
  float est_biais_pression;
  float offset_US;
  float prediction_US;
  float cov_alt;
  float cov_PWM;
  float cov_vitesse;
  bool    bool_effet_sol;
  float somme_inno;
  bool    flag_rejet_US;
  float u_multisinus;
  float gaz_altitude;
  bool    Flag_multisinus;
  bool    Flag_multisinus_debut;
}_ATTRIBUTE_PACKED_ navdata_kalman_pressure_t;


// ID25
typedef struct {

  uint32_t hdvideo_state;
  uint32_t storage_fifo_nb_packets;
  uint32_t storage_fifo_size;
  uint32_t usbkey_size;         /*! USB key in kbytes - 0 if no key present */
  uint32_t usbkey_freespace;    /*! USB key free space in kbytes - 0 if no key present */
  uint32_t frame_number;        /*! 'frame_number' PaVE field of the frame starting to be encoded for the HD stream */
  uint32_t usbkey_remaining_time; /*! time in seconds */

}_ATTRIBUTE_PACKED_ navdata_hdvideo_stream_t;

// ID26
typedef struct {
  uint32_t link_quality;
}_ATTRIBUTE_PACKED_ navdata_wifi_t;

// ID27
typedef struct {
  double latitude;
  double longitude;
  double elevation;
  double hdop;
  uint32_t   data_available;
  
  bool zero ;
  bool wpt;
  
  double late ;
  double longe ;
  double lat_fused ;
  double long_fused ;
  uint32_t gps_state ;
  float X_traj ;
  float X_ref ;
  float Y_traj ;
  float Y_ref ;
  float theta_p ;
  float phi_p ;
  float theta_i ;
  float phi_i ;
  float theta_d ;
  float phi_d ;
  double vdop;
  double pdop;
  float speeds;
  uint32_t  lastFrameTimestamp;
  float degree;
  float degree_magnetic;
  float ehpe ;
  float ehve ;
  float c_n0; //Signal to noise ratio (average of the four best satellites);
  uint32_t  nbsat; //Number of acquired satellites;
  navdata_gps_channel channels[12]; //channel
  bool is_gps_plugged;
  uint32_t ephemerisStatus;
  float vx_traj ;
  float vy_traj ;
  uint32_t firmwareStatus;
} navdata_gps_t ;

// ID NOT KNOWN
typedef struct {

  int32_t vzimmuLSB;
  float vzfind;

} navdata_zimmu_3000_t;

//#pragma pack(pop)

// NAVDATA BLOCK
typedef struct {
  navdata_demo_t            navdata_demo; 
  navdata_time_t            navdata_time;
  navdata_raw_measures_t    navdata_raw_measures;
  navdata_phys_measures_t   navdata_phys_measures;
  navdata_gyros_offsets_t   navdata_gyros_offsets;
  navdata_euler_angles_t    navdata_euler_angles;
  navdata_references_t      navdata_references;
  navdata_trims_t           navdata_trims;
  navdata_rc_references_t   navdata_rc_references;
  navdata_pwm_t             navdata_pwm;
  navdata_altitude_t        navdata_altitude;
  navdata_vision_raw_t      navdata_vision_raw;
  navdata_vision_of_t       navdata_vision_of;
  navdata_vision_t          navdata_vision;
  navdata_vision_perf_t     navdata_vision_perf;
  navdata_trackers_send_t   navdata_trackers_send;
  navdata_vision_detect_t   navdata_vision_detect;
  navdata_watchdog_t        navdata_watchdog;
  navdata_adc_data_frame_t  navdata_adc_data_frame;
  navdata_video_stream_t    navdata_video_stream;
  navdata_games_t           navdata_games;
  navdata_pressure_raw_t    navdata_pressure_raw;
  navdata_magneto_t         navdata_magneto;
  navdata_wind_speed_t      navdata_wind_speed;
  navdata_kalman_pressure_t navdata_kalman_pressure;
  navdata_hdvideo_stream_t  navdata_hdvideo_stream;
  navdata_wifi_t            navdata_wifi;
  navdata_gps_t             navdata_gps;

} navdata_block_t;

// NAVDATA GERAL COMPLETE
typedef struct {
  uint32_t    head;      /*!< Always set to NAVDATA_HEADER */
  uint32_t    ardrone_state;    /*!< Bit mask built from def_ardrone_state_mask_t */
  uint32_t    sequence;         /*!< Sequence number, incremented for each sent packet */
  int32_t     vision;
  uint16_t    *id;
  uint16_t    *siz;
  navdata_block_t  block;

} navdata_t;



#endif
