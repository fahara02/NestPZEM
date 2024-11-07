/* Automatically generated nanopb header */
/* Generated by nanopb-1.0.0-dev */

#ifndef PB_PZ_PDATA_PB_H_INCLUDED
#define PB_PZ_PDATA_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Enum definitions */
typedef enum _pz_PZEMModel {
    pz_PZEMModel_PZEM004T = 0,
    pz_PZEMModel_PZEM003 = 1,
    pz_PZEMModel_NOT_SET = 2
} pz_PZEMModel;

typedef enum _pz_Phase {
    pz_Phase_SINGLE_PHASE = 0,
    pz_Phase_RED_PHASE = 1,
    pz_Phase_YELLOW_PHASE = 2,
    pz_Phase_BLUE_PHASE = 3
} pz_Phase;

typedef enum _pz_State {
    pz_State_INIT = 0,
    pz_State_ACTIVE = 1,
    pz_State_IDLE = 2,
    pz_State_BUSY = 3,
    pz_State_POLL = 4,
    pz_State_UPDATE_FAIL = 5,
    pz_State_NOLOAD = 6,
    pz_State_NO_VOLTAGE = 7
} pz_State;

/* Struct definitions */
typedef struct _pz_NamePlate {
    pz_PZEMModel model;
    uint32_t id;
    uint32_t slaveAddress;
    uint32_t lineNo;
    pz_Phase phase;
    char meterName[10];
} pz_NamePlate;

typedef struct _pz_PowerMeasure {
    float voltage;
    float current;
    float power;
    float energy;
    float frequency;
    float pf;
    float alarms;
} pz_PowerMeasure;

typedef struct _pz_JobCard {
    bool has_info;
    pz_NamePlate info;
    bool has_pm;
    pz_PowerMeasure pm;
    int64_t poll_us;
    int64_t lastUpdate_us;
    int64_t dataAge_ms;
    bool dataStale;
    pz_State deviceState; /* Updated to use State enum directly */
} pz_JobCard;


#ifdef __cplusplus
extern "C" {
#endif

/* Helper constants for enums */
#define _pz_PZEMModel_MIN pz_PZEMModel_PZEM004T
#define _pz_PZEMModel_MAX pz_PZEMModel_NOT_SET
#define _pz_PZEMModel_ARRAYSIZE ((pz_PZEMModel)(pz_PZEMModel_NOT_SET+1))

#define _pz_Phase_MIN pz_Phase_SINGLE_PHASE
#define _pz_Phase_MAX pz_Phase_BLUE_PHASE
#define _pz_Phase_ARRAYSIZE ((pz_Phase)(pz_Phase_BLUE_PHASE+1))

#define _pz_State_MIN pz_State_INIT
#define _pz_State_MAX pz_State_NO_VOLTAGE
#define _pz_State_ARRAYSIZE ((pz_State)(pz_State_NO_VOLTAGE+1))

#define pz_NamePlate_model_ENUMTYPE pz_PZEMModel
#define pz_NamePlate_phase_ENUMTYPE pz_Phase


#define pz_JobCard_deviceState_ENUMTYPE pz_State


/* Initializer values for message structs */
#define pz_NamePlate_init_default                {_pz_PZEMModel_MIN, 0, 0, 0, _pz_Phase_MIN, ""}
#define pz_PowerMeasure_init_default             {0, 0, 0, 0, 0, 0, 0}
#define pz_JobCard_init_default                  {false, pz_NamePlate_init_default, false, pz_PowerMeasure_init_default, 0, 0, 0, 0, _pz_State_MIN}
#define pz_NamePlate_init_zero                   {_pz_PZEMModel_MIN, 0, 0, 0, _pz_Phase_MIN, ""}
#define pz_PowerMeasure_init_zero                {0, 0, 0, 0, 0, 0, 0}
#define pz_JobCard_init_zero                     {false, pz_NamePlate_init_zero, false, pz_PowerMeasure_init_zero, 0, 0, 0, 0, _pz_State_MIN}

/* Field tags (for use in manual encoding/decoding) */
#define pz_NamePlate_model_tag                   1
#define pz_NamePlate_id_tag                      2
#define pz_NamePlate_slaveAddress_tag            3
#define pz_NamePlate_lineNo_tag                  4
#define pz_NamePlate_phase_tag                   5
#define pz_NamePlate_meterName_tag               6
#define pz_PowerMeasure_voltage_tag              1
#define pz_PowerMeasure_current_tag              2
#define pz_PowerMeasure_power_tag                3
#define pz_PowerMeasure_energy_tag               4
#define pz_PowerMeasure_frequency_tag            5
#define pz_PowerMeasure_pf_tag                   6
#define pz_PowerMeasure_alarms_tag               7
#define pz_JobCard_info_tag                      1
#define pz_JobCard_pm_tag                        2
#define pz_JobCard_poll_us_tag                   3
#define pz_JobCard_lastUpdate_us_tag             4
#define pz_JobCard_dataAge_ms_tag                5
#define pz_JobCard_dataStale_tag                 6
#define pz_JobCard_deviceState_tag               7

/* Struct field encoding specification for nanopb */
#define pz_NamePlate_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, UENUM,    model,             1) \
X(a, STATIC,   SINGULAR, UINT32,   id,                2) \
X(a, STATIC,   SINGULAR, UINT32,   slaveAddress,      3) \
X(a, STATIC,   SINGULAR, UINT32,   lineNo,            4) \
X(a, STATIC,   SINGULAR, UENUM,    phase,             5) \
X(a, STATIC,   SINGULAR, STRING,   meterName,         6)
#define pz_NamePlate_CALLBACK NULL
#define pz_NamePlate_DEFAULT NULL

#define pz_PowerMeasure_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, FLOAT,    voltage,           1) \
X(a, STATIC,   SINGULAR, FLOAT,    current,           2) \
X(a, STATIC,   SINGULAR, FLOAT,    power,             3) \
X(a, STATIC,   SINGULAR, FLOAT,    energy,            4) \
X(a, STATIC,   SINGULAR, FLOAT,    frequency,         5) \
X(a, STATIC,   SINGULAR, FLOAT,    pf,                6) \
X(a, STATIC,   SINGULAR, FLOAT,    alarms,            7)
#define pz_PowerMeasure_CALLBACK NULL
#define pz_PowerMeasure_DEFAULT NULL

#define pz_JobCard_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, MESSAGE,  info,              1) \
X(a, STATIC,   OPTIONAL, MESSAGE,  pm,                2) \
X(a, STATIC,   SINGULAR, INT64,    poll_us,           3) \
X(a, STATIC,   SINGULAR, INT64,    lastUpdate_us,     4) \
X(a, STATIC,   SINGULAR, INT64,    dataAge_ms,        5) \
X(a, STATIC,   SINGULAR, BOOL,     dataStale,         6) \
X(a, STATIC,   SINGULAR, UENUM,    deviceState,       7)
#define pz_JobCard_CALLBACK NULL
#define pz_JobCard_DEFAULT NULL
#define pz_JobCard_info_MSGTYPE pz_NamePlate
#define pz_JobCard_pm_MSGTYPE pz_PowerMeasure

extern const pb_msgdesc_t pz_NamePlate_msg;
extern const pb_msgdesc_t pz_PowerMeasure_msg;
extern const pb_msgdesc_t pz_JobCard_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define pz_NamePlate_fields &pz_NamePlate_msg
#define pz_PowerMeasure_fields &pz_PowerMeasure_msg
#define pz_JobCard_fields &pz_JobCard_msg

/* Maximum encoded size of messages (where known) */
#define PZ_PDATA_PB_H_MAX_SIZE                   pz_JobCard_size
#define pz_JobCard_size                          109
#define pz_NamePlate_size                        33
#define pz_PowerMeasure_size                     35

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
