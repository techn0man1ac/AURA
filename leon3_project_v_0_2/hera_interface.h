#ifndef HERA_INTERFACE_H
#define HERA_INTERFACE_H

#define AFC_IMG_WIDTH  1020
#define AFC_IMG_HEIGHT 1020
#define AFC_IMG_SIZE   (AFC_IMG_WIDTH * AFC_IMG_HEIGHT)

/*! Unsigned integer 8 bits  (size: 1 bytes) */
typedef unsigned char uint8;

/*! Unsigned integer 16 bits  (size: 2 bytes) */
typedef unsigned short uint16;

/*! Unsigned integer 32 bits  (size: 4 bytes) */
typedef unsigned int uint32;

typedef enum {
    HERA_OK = 0,
    HERA_ERR_BUSY,          // Core0 is busy or State machine violation
    HERA_ERR_FAILURE,       // Core0 reported a failure
    HERA_ERR_INVALID_PARAM  // Invalid parameter passed
} error_code_t;

// AFC states
typedef enum {
    AFC_IDLE = 0,
    AFC_IMAGE_REQUESTED,
    AFC_IMAGE_REQUEST_COMPLETED,
    AFC_IMAGE_COPY_REQUESTED,
    AFC_IMAGE_COPY_COMPLETED,
    AFC_ERROR
} AFC_states_t;

// Hyperscout states
typedef enum {
    HS_IDLE = 0,
    HS_ACQUIRE_COPY_REQUESTED,
    HS_ACQUIRE_COPY_COMPLETED,
    HS_ERROR
} HS_states_t;

// TIRI states
typedef enum {
    TIRI_IDLE = 0,
    TIRI_ACQUIRE_COPY_REQUESTED,
    TIRI_ACQUIRE_COPY_COMPLETED,
    TIRI_ERROR
} TIRI_states_t;

// Generic Request States for Reports (HK, Event, Science)
typedef enum {
    REQ_IDLE = 0,
    REQ_PENDING,
    REQ_PROCESSED,
    REQ_ERROR
} Request_states_t;

#define MAX_HK_SIZE      256 //including headers
#define MAX_EVENT_SIZE   50 //including headers
#define MAX_SCIENCE_SIZE 2048 //including headers

typedef struct {
    uint16 sid;
    uint16 size;
    uint8  data[MAX_HK_SIZE];
    volatile uint8 state;
} Housekeeping_struct;

typedef struct {
    uint16 event_id;
    uint16 size;
    uint8  data[MAX_EVENT_SIZE];
    volatile uint8 state;
} Event_struct;

typedef struct {
    uint16 apid;
    uint8  type;
    uint8  subtype;
    uint16 size;
    uint8  data[MAX_SCIENCE_SIZE];
    volatile uint8 state;
} Science_struct;

typedef struct {
    // AFC
    volatile uint8  afc_state;
    volatile uint32 afc_exposure_time_us;
    uint8 afc_image_buffer[AFC_IMG_SIZE];

    // Hyperscout
    volatile uint8  hs_state;

    // TIRI
    volatile uint8  tiri_state;

    // Sleep
    volatile uint32 sleep_counter_sec; 
    volatile uint8  sleep_active_flag; // 1 if Core1 is sleeping waiting for timer

    // Data
    Housekeeping_struct hk_req;
    Event_struct        evt_req;
    Science_struct      sci_req;

} ControlBlock_t;

// Payloads
error_code_t Hera_AFC_AcquireSingleImage(uint32 exp_us);
error_code_t Hera_AFC_StoreImage(void);

uint8* Hera_AFC_GetImageBuffer(void);

error_code_t Hera_HS_AcquireImage(uint32 exp_ms);
error_code_t Hera_TIRI_AcquireImage(void);

// Sleep
error_code_t Hera_Sleep(uint32 seconds);

// Reporting
error_code_t Hera_HK_Report(uint16 sid, void* pData, uint16 size);
error_code_t Hera_Event_Report(uint16 event_id, void* pData, uint16 size);
error_code_t Hera_Science_Report(uint16 apid, uint8 type, uint8 subtype, void* pData, uint16 size);

// Data Bank
uint8 Hera_Read_Parameter_uint8(uint16 parameter_ref);
uint16 Hera_Read_Parameter_uint16(uint16 parameter_ref);
uint32 Hera_Read_Parameter_uint32(uint16 parameter_ref);
int8 Hera_Read_Parameter_int8(uint16 parameter_ref);
int16 Hera_Read_Parameter_int16(uint16 parameter_ref);
int32 Hera_Read_Parameter_int32(uint16 parameter_ref);
float32 Hera_Read_Parameter_float32(uint16 parameter_ref);
float64 Hera_Read_Parameter_float64(uint16 parameter_ref);

void IPSS_power_down(void);

#endif
