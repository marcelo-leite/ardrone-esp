#ifndef HEADER_H   // guardas de cabeçalho, impedem inclusões cíclicas
#define HEADER_H

#include <stdint.h>
#include <navdata.h>
#include <videostream.h>
#define _ATTRIBUTE_PACKED_  __attribute__((packed, aligned(1)))


typedef struct {
    
    uint32_t    sequence;
    // uint32_t    ctrl_state;             /*!< Flying state (landed, flying, hovering, etc.) defined in CTRL_STATES enum. */
    uint32_t    adrone_state;
    uint32_t    baterry; /*!< battery voltage filtered (mV) */

    float   theta;                  /*!< UAV's pitch in milli-degrees */
    float   phi;                    /*!< UAV's roll  in milli-degrees */
    float   psi;                    /*!< UAV's yaw   in milli-degrees */

    int32_t     altitude;               /*!< UAV's altitude in centimeters */
    int32_t   pression;

    vector31_t   v;                     /*!< UAV's estimated linear velocity */
  

    vector31_t   phys_accs;
    vector31_t   phys_gyros;

    float wind_speed;			// estimated wind speed [m/s]
    float wind_angle;	


    uint8_t     motor[4];
    
    uint32_t link_quality;
    
    double latitude;
    double longitude;
    double elevation;
    uint32_t gps_state;
    uint32_t  nbsat; //Number of acquired satellites;

} _ATTRIBUTE_PACKED_  fligth_data_t;

typedef struct {
    
    char    signature[7];
    fligth_data_t fligth_data;
    // navdata_demo_t* demo;
    PaVE_t pave;
    


} _ATTRIBUTE_PACKED_ ardata_t;


#endif