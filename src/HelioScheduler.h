
/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Scheduler
*/

#ifndef HelioScheduler_H
#define HelioScheduler_H

class HelioScheduler;
struct HelioSchedulerSubData;
struct HelioProcess;
struct HelioTracking;

#include "Helioduino.h"

// Scheduler
// The Scheduler acts as the system's main scheduling attendant, who looks through all
// the various equipment and crops you have programmed in, and figures out the best
// case tracking and lighting sequences that should occur to support them. It is also
// responsible for setting up and maintaining the system balancers that get assigned to
// track panels, which drives the various driving processes in use, as well as
// determining when significant time changes have occurred and broadcasting such out.
class HelioScheduler {
public:
    HelioScheduler();
    ~HelioScheduler();

    void update();

    void setupServoDriver(HelioPanel *panel, SharedPtr<HelioDriver> servoDriver);
    
    void setBaseTrackMultiplier(float trackMultiplier);
    void setAirReportInterval(TimeSpan interval);

    inline void setNeedsScheduling();
    inline bool needsScheduling() { return _needsScheduling; }

    inline bool inDaytimeMode() const { return _inDaytimeMode; }

    float getBaseTrackMultiplier() const;
    TimeSpan getAirReportInterval() const;

protected:
    bool _inDaytimeMode;                                    // Whenever in daytime tracking mode or not
    bool _needsScheduling;                                  // Needs rescheduling tracking flag
    int _lastDayNum;                                        // Last day number tracking for daily rescheduling tracking
    Map<hkey_t, HelioTracking *, HELIO_SCH_TRACKRES_MAXSIZE> _trackings; // Trackings in progress

    friend class Helioduino;

    inline HelioSchedulerSubData *schedulerData() const;
    inline bool hasSchedulerData() const;

    void updateDayTracking();
    void performScheduling();
    void broadcastDayChange();
};


// Scheduler Tracking Process Log Type
enum HelioTrackingLogType : signed char {
    HelioTrackingLogType_WaterSetpoints,                     // Water setpoints
    HelioTrackingLogType_WaterMeasures,                      // Water measurements
    HelioTrackingLogType_AirSetpoints,                       // Air setpoints
    HelioTrackingLogType_AirMeasures                         // Air measurements
};

// Scheduler Tracking Process Broadcast Type
enum HelioTrackingBroadcastType : signed char {
    HelioTrackingBroadcastType_Began,                        // Began main process
    HelioTrackingBroadcastType_Ended                         // Ended main process
};

// Scheduler Process Base
// Processes are created and managed by Scheduler to manage the tracking and lighting
// sequences necessary for crops to grow.
struct HelioProcess {
    SharedPtr<HelioTrackPanel> trackRes;                      // Track panel
    Vector<HelioActuatorAttachment, HELIO_SCH_REQACTUATORS_MAXSIZE> actuatorReqs; // Actuators required for this stage (keep-enabled list)

    time_t stageStart;                                      // Stage start time

    HelioProcess(SharedPtr<HelioTrackPanel> trackRes);

    void clearActuatorReqs();
    void setActuatorReqs(const Vector<HelioActuatorAttachment, HELIO_SCH_REQACTUATORS_MAXSIZE> &actuatorReqsIn);
};

// Scheduler Tracking Process
struct HelioTracking : public HelioProcess {
    enum : signed char {Init,TopOff,PreTrack,Track,Drain,Done,Unknown = -1} stage; // Current tracking stage

    time_t canTrackAfter;                                    // Time next tracking can occur (UTC)
    time_t lastAirReport;                                   // Last time an air report was generated (UTC)

    float phSetpoint;                                       // Calculated pH setpoint for attached crops
    float tdsSetpoint;                                      // Calculated TDS setpoint for attached crops
    float waterTempSetpoint;                                // Calculated water temp setpoint for attached crops
    float airTempSetpoint;                                  // Calculated air temp setpoint for attached crops
    float co2Setpoint;                                      // Calculated co2 level setpoint for attached crops

    HelioTracking(SharedPtr<HelioTrackPanel> trackRes);
    ~HelioTracking();

    void recalcTracking();
    void setupStaging();
    void update();

private:
    void reset();
    void logTracking(HelioTrackingLogType logType);
    void broadcastTracking(HelioTrackingBroadcastType broadcastType);
};


// Scheduler Serialization Sub Data
// A part of HSYS system data.
struct HelioSchedulerSubData : public HelioSubData {
    float baseTrackMultiplier;                               // Track aggressiveness base TDS/EC multiplier (applies to *ALL* tracking solutions in use - default: 1)
    float weeklyDosingRates[HELIO_CROP_GROWWEEKS_MAX];      // Nutrient dosing rate percentages (applies to any nutrient premixes in use - default: 1)
    float stdDosingRates[3];                                // Standard dosing rates for fresh water, pH up, and pH down (default: 1,1/2,1/2)
    uint8_t totalTrackingsDay;                               // Total number of trackings per day, if any (else 0 for disable - default: 0)
    uint8_t preTrackAeratorMins;                             // Minimum time to run aerators (if present) before track motors turn on, in minutes (default: 30)
    uint8_t preLightSprayMins;                              // Minimum time to run sprayers/sprinklers (if present/needed) before grow lights turn on, in minutes (default: 60)
    time_t airReportInterval;                               // Interval between air sensor reports, in seconds (default: 8hrs)

    HelioSchedulerSubData();
    void toJSONObject(JsonObject &objectOut) const;
    void fromJSONObject(JsonObjectConst &objectIn);
};

#endif // /ifndef HelioScheduler_H
