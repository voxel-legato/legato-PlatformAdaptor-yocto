/** @file gnss_yocto.c
 *
 * Legato @ref c_gnss_yocto include file.
 *
 * Copyright (C) Sierra Wireless Inc.
 */

#include "legato.h"

#include "pa_gnss.h"

#include <gps.h>


le_thread_Ref_t gpsdThreadRef; ///< the reader thread


typedef struct _m {
    int keepRunning;
    char *gpsd_host;  // todo
    char* gpsd_port;    // todo
    struct gpsduitto *client; 
} gpsdData;


le_thread_Ref_t gpsdThreadRef; ///< the reader thread

gpsdData gpsdClientData = {
    .keepRunning = 1,
    .client = NULL,
    .gpsd_host = "localhost",
    .gpsd_port = "2947",
};

//--------------------------------------------------------------------------------------------------
/**
 * Set the NMEA string for NMEA handler
 */
//--------------------------------------------------------------------------------------------------
#define NMEA_STR_LEN                  128

//--------------------------------------------------------------------------------------------------
/**
 * Set the suplCertificateId length
 */
//--------------------------------------------------------------------------------------------------
#define SUPL_CERTIFICATE_ID_LEN        9

//--------------------------------------------------------------------------------------------------
/**
 * Position event ID used to report position events to the registered event handlers.
 */
//--------------------------------------------------------------------------------------------------
static le_event_Id_t        GnssEventId;

//--------------------------------------------------------------------------------------------------
/**
 * Nmea event ID used to report Nmea events to the registered event handlers.
 */
//--------------------------------------------------------------------------------------------------
static le_event_Id_t        NmeaEventId;

//--------------------------------------------------------------------------------------------------
/**
 * The computed position data.
 */
//--------------------------------------------------------------------------------------------------
static pa_Gnss_Position_t   GnssPositionData;

//--------------------------------------------------------------------------------------------------
/**
 * Memory pool for position event data.
 */
//--------------------------------------------------------------------------------------------------
static le_mem_PoolRef_t     PositionEventDataPool;

//--------------------------------------------------------------------------------------------------
/**
 * Memory pool for Nmea event data.
 */
//--------------------------------------------------------------------------------------------------
static le_mem_PoolRef_t     NmeaEventDataPool;

//--------------------------------------------------------------------------------------------------
/**
 * The configured SUPL assisted mode.
 */
//--------------------------------------------------------------------------------------------------
static le_gnss_AssistedMode_t SuplAssistedMode = LE_GNSS_STANDALONE_MODE;

//--------------------------------------------------------------------------------------------------
/**
 * The configured bit mask for NMEA.
 */
//--------------------------------------------------------------------------------------------------
static le_gnss_NmeaBitMask_t NmeaBitMask = LE_GNSS_NMEA_MASK_GPGGA;

//--------------------------------------------------------------------------------------------------
/**
 * Multiplying factor accuracy
 */
//--------------------------------------------------------------------------------------------------
#define ONE_DECIMAL_PLACE_ACCURACY   (10)
#define TWO_DECIMAL_PLACE_ACCURACY   (100)
#define THREE_DECIMAL_PLACE_ACCURACY (1000)
#define SIX_DECIMAL_PLACE_ACCURACY   (1000000)

//--------------------------------------------------------------------------------------------------
/**
 * Inline function to convert and round to the nearest
 *
 * The function firstly converts the double value to int according to the requested place after the
 * decimal given by place parameter. Secondly, a round to the nearest is done the int value.
 */
//--------------------------------------------------------------------------------------------------
static inline int32_t ConvertAndRoundToNearest
(
    double value,    ///< [IN] value to round to the nearest
    int32_t place    ///< [IN] the place after the decimal in power of 10
)
{
    return (int32_t)((int64_t)((place*value*10) + ((value > 0) ? 5 : -5))/10);
}

//--------------------------------------------------------------------------------------------------
/**
 * GNSS position default pointer initialization.
 */
//--------------------------------------------------------------------------------------------------
static void InitializeDefaultGnssPositionData
(
    pa_Gnss_Position_t* posDataPtr  // [IN/OUT] Pointer to the position data.
)
{
    int i;
    posDataPtr->fixState = LE_GNSS_STATE_FIX_NO_POS;
    posDataPtr->altitudeValid = false;
    posDataPtr->altitudeAssumedValid = false;
    posDataPtr->altitudeOnWgs84Valid = false;
    posDataPtr->dateValid = false;
    posDataPtr->hdopValid = false;
    posDataPtr->hSpeedUncertaintyValid = false;
    posDataPtr->hSpeedValid = false;
    posDataPtr->hUncertaintyValid = false;
    posDataPtr->latitudeValid = false;
    posDataPtr->longitudeValid = false;
    posDataPtr->timeValid = false;
    posDataPtr->gpsTimeValid = false;
    posDataPtr->timeAccuracyValid = false;
    posDataPtr->positionLatencyValid = false;
    posDataPtr->directionUncertaintyValid = false;
    posDataPtr->directionValid = false;
    posDataPtr->vdopValid = false;
    posDataPtr->vSpeedUncertaintyValid = false;
    posDataPtr->vSpeedValid = false;
    posDataPtr->vUncertaintyValid = false;
    posDataPtr->pdopValid = false;
    posDataPtr->satMeasValid = false;
    posDataPtr->leapSecondsValid = false;
    posDataPtr->gdopValid = false;
    posDataPtr->tdopValid = false;

    for (i=0; i<LE_GNSS_SV_INFO_MAX_LEN; i++)
    {
        posDataPtr->satMeas[i].satId = 0;
        posDataPtr->satMeas[i].satLatency = 0;
    }
}


//--------------------------------------------------------------------------------------------------
/**
 * GNSS position valid pointer initialization.
 */
//--------------------------------------------------------------------------------------------------
static void InitializeValidGnssPositionData
(
    pa_Gnss_Position_t* posDataPtr  // [IN/OUT] Pointer to the position data.
)
{
    int i;
    posDataPtr->fixState = LE_GNSS_STATE_FIX_NO_POS;
    posDataPtr->altitudeValid = true;
    posDataPtr->altitudeAssumed = false;
    posDataPtr->altitude = 10;
    posDataPtr->altitudeOnWgs84Valid = true;
    posDataPtr->altitudeOnWgs84 = 10378;
    posDataPtr->dateValid = true;
    posDataPtr->date.year = 2017;
    posDataPtr->date.month = 10;
    posDataPtr->date.day = 4;
    posDataPtr->hdopValid = true;
    posDataPtr->hdop = 5000;
    posDataPtr->hSpeedUncertaintyValid = true;
    posDataPtr->hSpeedUncertainty = 1000;
    posDataPtr->hSpeedValid = true;
    posDataPtr->hSpeed = 20;
    posDataPtr->hUncertaintyValid = true;
    posDataPtr->hUncertainty = 100;
    posDataPtr->latitudeValid = true;
    posDataPtr->latitude = 37981;
    posDataPtr->longitudeValid = true;
    posDataPtr->longitude = 91078;
    posDataPtr->timeValid = true;
    posDataPtr->epochTime = 1000;
    posDataPtr->gpsTimeValid = true;
    posDataPtr->gpsWeek = 7;
    posDataPtr->gpsTimeOfWeek = 5;
    posDataPtr->time.hours = 23;
    posDataPtr->time.minutes = 59;
    posDataPtr->time.seconds = 50;
    posDataPtr->time.milliseconds = 100;
    posDataPtr->timeAccuracyValid = true;
    posDataPtr->timeAccuracy = 100000;
    posDataPtr->positionLatencyValid = true;
    posDataPtr->positionLatency = 109831;
    posDataPtr->directionUncertaintyValid = true;
    posDataPtr->directionUncertainty = 21987;
    posDataPtr->directionValid = true;
    posDataPtr->direction = 11576;
    posDataPtr->vdopValid = true;
    posDataPtr->vdop = 6000;
    posDataPtr->vSpeedUncertaintyValid = true;
    posDataPtr->vSpeedUncertainty = 5000;
    posDataPtr->vSpeedValid = true;
    posDataPtr->vSpeed = 50;
    posDataPtr->vUncertaintyValid = true;
    posDataPtr->vUncertainty = 8000;
    posDataPtr->pdopValid = true;
    posDataPtr->pdop = 7000;
    posDataPtr->leapSecondsValid = true;
    posDataPtr->leapSeconds = 30;
    posDataPtr->gdopValid = true;
    posDataPtr->gdop = 8000;
    posDataPtr->tdopValid = true;
    posDataPtr->tdop = 9000;

    posDataPtr->satMeasValid = true;
    for (i=0; i<LE_GNSS_SV_INFO_MAX_LEN; i++)
    {
        posDataPtr->satMeas[i].satId = 0;
        posDataPtr->satMeas[i].satLatency = 0;
    }
}

//--------------------------------------------------------------------------------------------------
/**
 * Initialize default satellites info that are updated in SV information report indication.
 */
//--------------------------------------------------------------------------------------------------
static void InitializeDefaultSatInfo
(
    pa_Gnss_Position_t* posDataPtr  // [IN/OUT] Pointer to the position data.
)
{
    int i;
    posDataPtr->satsInViewCountValid = false;
    posDataPtr->satsTrackingCountValid = false;
    posDataPtr->satInfoValid = false;
    posDataPtr->magneticDeviationValid = false;

    for (i=0; i<LE_GNSS_SV_INFO_MAX_LEN; i++)
    {
        posDataPtr->satInfo[i].satId = 0;
        posDataPtr->satInfo[i].satConst = LE_GNSS_SV_CONSTELLATION_UNDEFINED;
        posDataPtr->satInfo[i].satTracked = 0;
        posDataPtr->satInfo[i].satSnr = 0;
        posDataPtr->satInfo[i].satAzim = 0;
        posDataPtr->satInfo[i].satElev = 0;
    }
}

//--------------------------------------------------------------------------------------------------
/**
 * Initialize valid satellites info that are updated in SV information report indication.
 */
//--------------------------------------------------------------------------------------------------
static void InitializeValidSatInfo
(
    pa_Gnss_Position_t* posDataPtr  // [IN/OUT] Pointer to the position data.
)
{
    int i;
    posDataPtr->satsInViewCountValid = true;
    posDataPtr->satsInViewCount = 10;
    posDataPtr->satsTrackingCountValid = true;
    posDataPtr->satsTrackingCount = 8;
    posDataPtr->magneticDeviationValid = true;
    posDataPtr->magneticDeviation = 20;
    posDataPtr->satInfoValid = true;

    for (i=0; i<LE_GNSS_SV_INFO_MAX_LEN; i++)
    {
        posDataPtr->satInfo[i].satId = 0;
        posDataPtr->satInfo[i].satConst = LE_GNSS_SV_CONSTELLATION_UNDEFINED;
        posDataPtr->satInfo[i].satTracked = 0;
        posDataPtr->satInfo[i].satSnr = 0;
        posDataPtr->satInfo[i].satAzim = 0;
        posDataPtr->satInfo[i].satElev = 0;
    }
}

//--------------------------------------------------------------------------------------------------
/**
 * Initialize default satellites info that are updated as "used" in SV information report
 * indication.
 */
//--------------------------------------------------------------------------------------------------
static void InitializeDefaultSatUsedInfo
(
    pa_Gnss_Position_t* posDataPtr  // [IN/OUT] Pointer to the position data.
)
{
    int i;
    // Reset the SV marked as used in position data list.
    posDataPtr->satsUsedCountValid = false;
    for (i=0; i<LE_GNSS_SV_INFO_MAX_LEN; i++)
    {
        posDataPtr->satInfo[i].satUsed = false;
    }
}

//--------------------------------------------------------------------------------------------------
/**
 * Initialize valid satellites info that are updated as "used" in SV information report
 * indication.
 */
//--------------------------------------------------------------------------------------------------
static void InitializeValidSatUsedInfo
(
    pa_Gnss_Position_t* posDataPtr  // [IN/OUT] Pointer to the position data.
)
{
    int i;
    // Reset the SV marked as used in position data list.
    posDataPtr->satsUsedCountValid = true;
    posDataPtr->satsUsedCount = 5;
    for (i=0; i<LE_GNSS_SV_INFO_MAX_LEN; i++)
    {
        posDataPtr->satInfo[i].satUsed = 5;
    }
}

// scale degrees from double to integer values 
#define ScaleMegaDeg(x) ( (int32_t)(x*1000000))
#define ScaleAlti(x) ( (int32_t)(x*1000))
#define ScaleSpeed(x) ( (int32_t)(x*100))

void gpsdHandleData(void *context, struct gps_data_t* data)
{
    char line[1024] = "";

    gps_unpack(line, data);
    
    // LE_INFO("gpsd data %04llx fix %d %s", data->set, data->status, line); 

    /* Display data from the GPS receiver. */
    if ((data->status == STATUS_FIX) && 
	(data->fix.mode == MODE_2D || data->fix.mode == MODE_3D) &&
	!isnan(data->fix.latitude) && 
	!isnan(data->fix.longitude)) {
	//gettimeofday(&tv, NULL); EDIT: tv.tv_sec isn't actually the timestamp!
	// printf("latitude: %f, longitude: %f, speed: %f, timestamp: %lf\n",
	//       data->fix.latitude, data->fix.longitude,
	//       data->fix.speed, data->fix.time);

	// allocate memory for the position data
	pa_Gnss_Position_t* posDataPtr = le_mem_ForceAlloc(PositionEventDataPool);
	memcpy(posDataPtr, &GnssPositionData, sizeof(pa_Gnss_Position_t));


	posDataPtr->altitudeValid = (data->fix.mode == MODE_3D);

	// init all 2D fix data here which is relevant for 3D as well..:
	//posDataPtr->hAccuracyValid = true;  ///< if true, horizontal accuracy is set
	//posDataPtr->hAccuracy = 500;          ///< horizontal accuracy

	posDataPtr->altitudeValid = true;
	posDataPtr->altitudeAssumed = false;
	posDataPtr->altitude = 10;
	posDataPtr->altitudeOnWgs84Valid = true;
	posDataPtr->altitudeOnWgs84 = 10378;
	posDataPtr->dateValid = true;
	posDataPtr->date.year = 2017;
	posDataPtr->date.month = 10;
	posDataPtr->date.day = 4;
	posDataPtr->hdopValid = true;
	posDataPtr->hdop = 5000;
	posDataPtr->hSpeedUncertaintyValid = true;
	posDataPtr->hSpeedUncertainty = 1000;
	posDataPtr->hSpeedValid = true;
	posDataPtr->hSpeed = 20;
	posDataPtr->hUncertaintyValid = true;
	posDataPtr->hUncertainty = 100;
	posDataPtr->latitudeValid = true;
	posDataPtr->latitude = 37981;
	posDataPtr->longitudeValid = true;
	posDataPtr->longitude = 91078;
	posDataPtr->timeValid = true;
	posDataPtr->epochTime = 1000;
	posDataPtr->gpsTimeValid = true;
	posDataPtr->gpsWeek = 7;
	posDataPtr->gpsTimeOfWeek = 5;
	posDataPtr->time.hours = 23;
	posDataPtr->time.minutes = 59;
	posDataPtr->time.seconds = 50;
	posDataPtr->time.milliseconds = 100;
	posDataPtr->timeAccuracyValid = true;
	posDataPtr->timeAccuracy = 100000;
	posDataPtr->positionLatencyValid = true;
	posDataPtr->positionLatency = 109831;
	posDataPtr->directionUncertaintyValid = true;
	posDataPtr->directionUncertainty = 21987;
	posDataPtr->directionValid = true;
	posDataPtr->direction = 11576;
	posDataPtr->vdopValid = true;
	posDataPtr->vdop = 6000;
	posDataPtr->vSpeedUncertaintyValid = true;
	posDataPtr->vSpeedUncertainty = 5000;
	posDataPtr->vSpeedValid = true;
	posDataPtr->vSpeed = 50;
	posDataPtr->vUncertaintyValid = true;
	posDataPtr->vUncertainty = 8000;
	posDataPtr->pdopValid = true;
	posDataPtr->pdop = 7000;
	posDataPtr->leapSecondsValid = true;
	posDataPtr->leapSeconds = 30;
	posDataPtr->gdopValid = true;
	posDataPtr->gdop = 8000;
	posDataPtr->tdopValid = true;
	posDataPtr->tdop = 9000;

	posDataPtr->latitudeValid = true;
	posDataPtr->longitudeValid = true;
	posDataPtr->hUncertaintyValid = true;
	posDataPtr->hUncertainty = 100;

	posDataPtr->latitude = ScaleMegaDeg(data->fix.latitude);
	posDataPtr->longitude = ScaleMegaDeg(data->fix.longitude);
	posDataPtr->hSpeed = ScaleSpeed(data->fix.speed);
	posDataPtr->epochTime = (int)data->fix.time;

	

	
	// init 3D fix specific data here:
	if (data->fix.mode == MODE_2D) {
	    posDataPtr->fixState = LE_GNSS_STATE_FIX_2D;
	} else { // MODE_3D
	    posDataPtr->fixState = LE_GNSS_STATE_FIX_3D;

	    posDataPtr->altitude = ScaleAlti(data->fix.altitude);
	}


	// Build the data for the user's event handler.
	le_event_ReportWithRefCounting(GnssEventId, posDataPtr);
	//	LE_INFO("reported gnss! %d %d   %d  %d ", posDataPtr->fixState,  posDataPtr->latitude, posDataPtr->longitude, posDataPtr->altitude);
    } else {
      // printf("no GPS data available\n");
      // posDataPtr->fixState = LE_GNSS_STATE_FIX_NO_POS;
    }
}

int keepRunning=1;

void* gpsdFunc(void *context) {
    struct gps_data_t gps_data;

    int ret = gps_open("localhost", "2947", &gps_data);
    if (ret) {
	LE_CRIT("ERROR STARTING GPS! %s", gps_errstr(ret));
	return NULL;
    }
    
    (void) gps_stream(&gps_data, WATCH_ENABLE | WATCH_JSON, NULL);

    /* Put this in a loop with a call to a high resolution sleep () in it. */
    while (keepRunning) {
	if (gps_waiting(&gps_data, 1000 * 1000)) {
	    errno = 0;
	    int num = gps_read(&gps_data);
	    if (num == -1) {
		LE_ERROR("gps_read failed! Stopping GPS loop!");
		keepRunning = false;
	    } else {
		/* Display data from the GPS receiver. */
		//if (gps_data.set & ...
		if (gps_data.set & LATLON_SET) {
		    gpsdHandleData(context, &gps_data);
		} else {
		    gpsdHandleData(context, &gps_data);
		}
	    }
	}
	while (le_event_ServiceLoop() == LE_OK) {}
    }

    /* When you are done... */
    (void) gps_stream(&gps_data, WATCH_DISABLE, NULL);
    (void) gps_close (&gps_data);
    return NULL;
}

//--------------------------------------------------------------------------------------------------
/**
 * This function must be called to initialize the PA gnss Module.
 *
 * @return LE_FAULT  The function failed.
 * @return LE_OK     The function succeed.
 */
//--------------------------------------------------------------------------------------------------
le_result_t pa_gnss_Init
(
    void
)
{
    InitializeDefaultGnssPositionData(&GnssPositionData);
    InitializeDefaultSatInfo(&GnssPositionData);
    InitializeDefaultSatUsedInfo(&GnssPositionData);

    LE_INFO("pa_gnss_Init");
    
    GnssEventId = le_event_CreateIdWithRefCounting("GnssEventId");
    NmeaEventId = le_event_CreateIdWithRefCounting("GnssNmeaEventId");

    PositionEventDataPool = le_mem_CreatePool("PositionEventDataPool", sizeof(pa_Gnss_Position_t));
    NmeaEventDataPool = le_mem_CreatePool("NmeaEventDataPool", NMEA_STR_LEN * sizeof(char));

    gpsdThreadRef = le_thread_Create("gpsdConn", gpsdFunc, &gpsdClientData);

    le_thread_Start(gpsdThreadRef);

    return LE_OK;
}

//--------------------------------------------------------------------------------------------------
/**
 * This function must be called to release the PA gnss Module.
 *
 * @return LE_FAULT  The function failed.
 * @return LE_OK     The function succeed.
 */
//--------------------------------------------------------------------------------------------------
le_result_t pa_gnss_Release
(
    void
)
{
    return LE_OK;
}

//--------------------------------------------------------------------------------------------------
/**
 * Set the GNSS constellation bit mask
 *
 * @return
 *  - LE_OK on success
 *  - LE_FAULT on failure
 *  - LE_UNSUPPORTED request not supported
 */
//--------------------------------------------------------------------------------------------------
le_result_t pa_gnss_SetConstellation
(
    le_gnss_ConstellationBitMask_t constellationMask  ///< [IN] GNSS constellation used in solution.
)
{
    return LE_OK;
}

//--------------------------------------------------------------------------------------------------
/**
 * Get the GNSS constellation bit mask
 *
* @return
*  - LE_OK on success
*  - LE_FAULT on failure
 */
//--------------------------------------------------------------------------------------------------
le_result_t pa_gnss_GetConstellation
(
    le_gnss_ConstellationBitMask_t *constellationMaskPtr ///< [OUT] GNSS constellation used in
                                                         ///< solution
)
{
    return LE_OK;
}

//--------------------------------------------------------------------------------------------------
/**
 * This function must be called to start the gnss acquisition.
 *
 * @return LE_FAULT  The function failed.
 * @return LE_OK     The function succeed.
 */
//--------------------------------------------------------------------------------------------------
le_result_t pa_gnss_Start
(
    void
)
{
    return LE_OK;
}

//--------------------------------------------------------------------------------------------------
/**
 * This function must be called to stop the gnss acquisition.
 *
 * @return LE_FAULT  The function failed.
 * @return LE_OK     The function succeed.
 */
//--------------------------------------------------------------------------------------------------
le_result_t pa_gnss_Stop
(
    void
)
{
    return LE_OK;
}

//--------------------------------------------------------------------------------------------------
/**
 * This function sets the GNSS device acquisition rate.
 *
 * @return
 *  - LE_OK on success
 *  - LE_OK on failure
 *  - LE_UNSUPPORTED request not supported
 *  - LE_TIMEOUT a time-out occurred
 */
//--------------------------------------------------------------------------------------------------
le_result_t pa_gnss_SetAcquisitionRate
(
    uint32_t rate     ///< [IN] rate in milliseconds
)
{
    return LE_OK;
}

//--------------------------------------------------------------------------------------------------
/**
 * This function must be called to get the rate of GNSS fix reception
 *
 *
 * @return LE_OK         The function failed.
 * @return LE_OK            The function succeeded.
 */
//--------------------------------------------------------------------------------------------------
le_result_t pa_gnss_GetAcquisitionRate
(
    uint32_t* ratePtr     ///< [IN] rate in milliseconds
)
{

    return LE_OK;
}

//--------------------------------------------------------------------------------------------------
/**
 * Initialize valid position data
 */
//--------------------------------------------------------------------------------------------------
void pa_gnss_SetGnssValidPositionData
(
    void
)
{
    InitializeValidGnssPositionData(&GnssPositionData);
    InitializeValidSatInfo(&GnssPositionData);
    InitializeValidSatUsedInfo(&GnssPositionData);
}

//--------------------------------------------------------------------------------------------------
/**
 * Report the position event
 *
 */
//--------------------------------------------------------------------------------------------------
void pa_gnssTODO_ReportEvent
(
    void
)
{
    // Build the data for the user's event handler.
    pa_Gnss_Position_t* posDataPtr = le_mem_ForceAlloc(PositionEventDataPool);
    memcpy(posDataPtr, &GnssPositionData, sizeof(pa_Gnss_Position_t));
    le_event_ReportWithRefCounting(GnssEventId, posDataPtr);


    char* strDataPtr = le_mem_ForceAlloc(NmeaEventDataPool);
    strncpy(strDataPtr, "nmea", NMEA_STR_LEN);
    le_event_ReportWithRefCounting(NmeaEventId, strDataPtr);
}

//--------------------------------------------------------------------------------------------------
/**
 * This function must be called to register an handler for gnss position data notifications.
 *
 * @return A handler reference, which is only needed for later removal of the handler.
 *
 * @note Doesn't return on failure, so there's no need to check the return value for errors.
 */
//--------------------------------------------------------------------------------------------------
le_event_HandlerRef_t pa_gnss_AddPositionDataHandler
(
    pa_gnss_PositionDataHandlerFunc_t handler ///< [IN] The handler function.
)
{
    LE_INFO("pa_gnss_AddPositionDataHandler hdl %p", handler);
    LE_FATAL_IF((handler==NULL),"gnss module cannot set handler");

    le_event_HandlerRef_t newHandlerPtr = le_event_AddHandler(
                                                            "gpsInformationHandler",
                                                            GnssEventId,
                                                            (le_event_HandlerFunc_t) handler);

    return newHandlerPtr;
}

//--------------------------------------------------------------------------------------------------
/**
 * This function must be called to remove a handler for gnss position data notifications.
 *
 * @note Doesn't return on failure, so there's no need to check the return value for errors.
 */
//--------------------------------------------------------------------------------------------------
void pa_gnss_RemovePositionDataHandler
(
    le_event_HandlerRef_t    handlerRef ///< [IN] The handler reference.
)
{
    le_event_RemoveHandler(handlerRef);
}

//--------------------------------------------------------------------------------------------------
/**
 * This function must be called to load an 'Extended Ephemeris' file into the GNSS device.
 *
 * @return LE_OK         The function failed to inject the 'Extended Ephemeris' file.
 * @return LE_TIMEOUT       A time-out occurred.
 * @return LE_FORMAT_ERROR  'Extended Ephemeris' file format error.
 * @return LE_OK            The function succeeded.
 *
 * @TODO    implementation
 */
//--------------------------------------------------------------------------------------------------
le_result_t pa_gnss_LoadExtendedEphemerisFile
(
    int32_t       fd      ///< [IN] extended ephemeris file descriptor
)
{
    return LE_OK;
}

//--------------------------------------------------------------------------------------------------
/**
 * This function must be called to get the validity of the last injected Extended Ephemeris.
 *
 * @return LE_OK         The function failed to get the validity
 * @return LE_OK            The function succeeded.
 *
 * @TODO    implementation
 */
//--------------------------------------------------------------------------------------------------
le_result_t pa_gnss_GetExtendedEphemerisValidity
(
    uint64_t *startTimePtr,    ///< [OUT] Start time in seconds (since Jan. 1, 1970)
    uint64_t *stopTimePtr      ///< [OUT] Stop time in seconds (since Jan. 1, 1970)
)
{
    return LE_OK;
}

//--------------------------------------------------------------------------------------------------
/**
 * This function enables the use of the 'Extended Ephemeris' file into the GNSS device.
 *
 * @return LE_OK         The function failed to enable the 'Extended Ephemeris' file.
 * @return LE_OK            The function succeeded.
 *
 */
//--------------------------------------------------------------------------------------------------
le_result_t pa_gnss_EnableExtendedEphemerisFile
(
    void
)
{
    return LE_OK;
}

//--------------------------------------------------------------------------------------------------
/**
 * This function disables the use of the 'Extended Ephemeris' file into the GNSS device.
 *
 * @return LE_OK         The function failed to disable the 'Extended Ephemeris' file.
 * @return LE_OK            The function succeeded.
 *
 */
//--------------------------------------------------------------------------------------------------
le_result_t pa_gnss_DisableExtendedEphemerisFile
(
    void
)
{
    return LE_OK;
}

//--------------------------------------------------------------------------------------------------
/**
 * This function must be called to inject UTC time into the GNSS device.
 *
 * @return
 *  - LE_OK            The function succeeded.
 *  - LE_OK         The function failed to inject the UTC time.
 *  - LE_TIMEOUT       A time-out occurred.
 *
 */
//--------------------------------------------------------------------------------------------------
le_result_t pa_gnss_InjectUtcTime
(
    uint64_t timeUtc,      ///< [IN] UTC time since Jan. 1, 1970 in milliseconds
    uint32_t timeUnc       ///< [IN] Time uncertainty in milliseconds
)
{
    return LE_OK;
}

//--------------------------------------------------------------------------------------------------
/**
 * This function delete GNSS assistance data for warm/cold/factory start
 *
 * @return LE_OK           The function failed.
 * @return LE_OK              The function succeeded.
 * @return LE_UNSUPPORTED     Restart type not supported.
 * @return LE_BAD_PARAMETER   Bad input parameter.
 */
//--------------------------------------------------------------------------------------------------
le_result_t pa_gnss_DeleteAssistData
(
    le_gnss_StartMode_t mode    ///< [IN] Start mode
)
{
    return LE_OK;
}

//--------------------------------------------------------------------------------------------------
/**
 * This function must be called to stop the GNSS engine.
 *
 * @return LE_OK  The function failed.
 * @return LE_OK     The function succeed.
 */
//--------------------------------------------------------------------------------------------------
le_result_t pa_gnss_ForceEngineStop
(
    void
)
{
    return LE_OK;
}

//--------------------------------------------------------------------------------------------------
/**
 * Get the TTFF in milliseconds.
 *
 * @return LE_BUSY          The position is not fixed and TTFF can't be measured.
 * @return LE_OK            The function succeeded.
 */
//--------------------------------------------------------------------------------------------------
le_result_t pa_gnss_GetTtff
(
    uint32_t* ttffPtr     ///< [OUT] TTFF in milliseconds
)
{
    return LE_OK;
}

//--------------------------------------------------------------------------------------------------
/**
 * This function enables the GNSS device.
 *
 * @return LE_OK         The function failed.
 * @return LE_OK            The function succeeded.
 *
 */
//--------------------------------------------------------------------------------------------------
le_result_t pa_gnss_Enable
(
    void
)
{
    return LE_OK;
}

//--------------------------------------------------------------------------------------------------
/**
 * This function disables the GNSS device.
 *
 * @return LE_OK         The function failed.
 * @return LE_OK            The function succeeded.
 *
 */
//--------------------------------------------------------------------------------------------------
le_result_t pa_gnss_Disable
(
    void
)
{
    return LE_OK;
}

//--------------------------------------------------------------------------------------------------
/**
 * This function sets the SUPL Assisted-GNSS mode.
 *
 * @return
 *  - LE_OK on success
 *  - LE_OK on failure
 *  - LE_UNSUPPORTED request not supported
 *  - LE_TIMEOUT a time-out occurred
 */
//--------------------------------------------------------------------------------------------------
le_result_t pa_gnss_SetSuplAssistedMode
(
    le_gnss_AssistedMode_t  assistedMode      ///< [IN] Assisted-GNSS mode.
)
{
    return LE_UNSUPPORTED;

}

//--------------------------------------------------------------------------------------------------
/**
 * This function gets the SUPL Assisted-GNSS mode.
 *
 * @return
 *  - LE_OK on success
 *  - LE_OK on failure
 */
//--------------------------------------------------------------------------------------------------
le_result_t pa_gnss_GetSuplAssistedMode
(
    le_gnss_AssistedMode_t *assistedModePtr      ///< [OUT] Assisted-GNSS mode.
)
{
    if (assistedModePtr == NULL)
    {
        return LE_FAULT;
    }

    // Get the SUPL assisted mode
    *assistedModePtr = SuplAssistedMode;
    return LE_OK;
}

//--------------------------------------------------------------------------------------------------
/**
 * This function sets the SUPL server URL.
 * That server URL is a NULL-terminated string with a maximum string length (including NULL
 * terminator) equal to 256. Optionally the port number is specified after a colon.
 *
 * @return
 *  - LE_OK on success
 *  - LE_OK on failure
 *  - LE_BUSY service is busy
 *  - LE_TIMEOUT a time-out occurred
 */
//--------------------------------------------------------------------------------------------------
le_result_t pa_gnss_SetSuplServerUrl
(
    const char*  suplServerUrlPtr      ///< [IN] SUPL server URL.
)
{
    return LE_OK;
}

//--------------------------------------------------------------------------------------------------
/**
 * This function injects the SUPL certificate to be used in A-GNSS sessions.
 *
 * @return
 *  - LE_OK on success
 *  - LE_BAD_PARAMETER on invalid parameter
 *  - LE_OK on failure
 *  - LE_BUSY service is busy
 *  - LE_TIMEOUT a time-out occurred
 */
//--------------------------------------------------------------------------------------------------
le_result_t pa_gnss_InjectSuplCertificate
(
    uint8_t  suplCertificateId,      ///< [IN] ID of the SUPL certificate.
                                     ///< Certificate ID range is 0 to 9
    uint16_t suplCertificateLen,     ///< [IN] SUPL certificate size in Bytes.
    const char*  suplCertificatePtr  ///< [IN] SUPL certificate contents.
)
{
    return LE_OK;
}

//--------------------------------------------------------------------------------------------------
/**
 * This function deletes the SUPL certificate.
 *
 * @return
 *  - LE_OK on success
 *  - LE_BAD_PARAMETER on invalid parameter
 *  - LE_OK on failure
 *  - LE_BUSY service is busy
 *  - LE_TIMEOUT a time-out occurred
 */
//--------------------------------------------------------------------------------------------------
le_result_t pa_gnss_DeleteSuplCertificate
(
    uint8_t  suplCertificateId  ///< [IN]  ID of the SUPL certificate.
                                ///< Certificate ID range is 0 to 9
)
{
    return LE_OK;
}

//--------------------------------------------------------------------------------------------------
/**
 * Set the enabled NMEA sentences bit mask
 *
 * @return
 *  - LE_OK on success
 *  - LE_OK on failure
 *  - LE_BUSY service is busy
 *  - LE_TIMEOUT a time-out occurred
 */
//--------------------------------------------------------------------------------------------------
le_result_t pa_gnss_SetNmeaSentences
(
    le_gnss_NmeaBitMask_t nmeaMask ///< [IN] Bit mask for enabled NMEA sentences.
)
{
    NmeaBitMask = nmeaMask;
    return LE_OK;
}

//--------------------------------------------------------------------------------------------------
/**
 * Get the enabled NMEA sentences bit mask
 *
* @return
*  - LE_OK on success
*  - LE_OK on failure
*  - LE_BUSY service is busy
*  - LE_TIMEOUT a time-out occurred
 */
//--------------------------------------------------------------------------------------------------
le_result_t pa_gnss_GetNmeaSentences
(
    le_gnss_NmeaBitMask_t* nmeaMaskPtr ///< [OUT] Bit mask for enabled NMEA sentences.
)
{
    if (NULL == nmeaMaskPtr)
    {
        LE_ERROR("NULL pointer");
        return LE_FAULT;
    }

    *nmeaMaskPtr = NmeaBitMask;
    return LE_OK;
}


//--------------------------------------------------------------------------------------------------
/**
 * This function must be called to register an handler for NMEA frames notifications.
 *
 * @return A handler reference, which is only needed for later removal of the handler.
 *
 * @note Doesn't return on failure, so there's no need to check the return value for errors.
 */
//--------------------------------------------------------------------------------------------------
le_event_HandlerRef_t pa_gnss_AddNmeaHandler
(
    pa_gnss_NmeaHandlerFunc_t handler ///< [IN] The handler function.
)
{
    LE_FATAL_IF((handler==NULL),"gnss module cannot set handler");
    LE_DEBUG("pa_gnss_AddNmeaHandler %p", handler);

    le_event_HandlerRef_t newHandlerPtr = le_event_AddHandler(
                                                            "gnssNmeaHandler",
                                                            NmeaEventId,
                                                            (le_event_HandlerFunc_t) handler);
    return newHandlerPtr;
}

//--------------------------------------------------------------------------------------------------
/**
 * This function gets leap seconds information
 *
 * @return
 *  - LE_OK           The function successed
 *  - LE_OK        The function failed
 *  - LE_TIMEOUT      The function timeout
 *  - LE_UNSUPPORTED  Not supported on this platform
 */
//--------------------------------------------------------------------------------------------------
le_result_t pa_gnss_GetLeapSeconds
(
    uint64_t* gpsTimePtr,              ///< [OUT] The number of milliseconds of GPS time since
                                       ///<       Jan. 6, 1980
    int32_t* currentLeapSecondsPtr,    ///< [OUT] Current UTC leap seconds value in milliseconds
    uint64_t* changeEventTimePtr,      ///< [OUT] The number of milliseconds since Jan. 6, 1980
                                       ///<       to the next leap seconds change event
    int32_t* nextLeapSecondsPtr        ///< [OUT] UTC leap seconds value to be applied at the
                                       ///<       change event time in milliseconds
)
{
    return LE_OK;
}

//--------------------------------------------------------------------------------------------------
/**
 * Component initializer called automatically by the Legato application framework.
 * This is not used because we want to make sure that GNSS is available before initializing
 * the platform adapter.
 *
 * See pa_gnss_Init().
 */
//--------------------------------------------------------------------------------------------------
COMPONENT_INIT
{
}
