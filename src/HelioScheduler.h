
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
    Twilight _dailyTwilight;                                // Daily twilight settings
    bool _needsScheduling;                                  // Needs rescheduling tracking flag
    bool _inDaytimeMode;                                    // Daytime mode flag
    int _lastDayNum;                                        // Last day number tracking for daily rescheduling tracking
    Map<hkey_t, HelioTracking *, HELIO_SCH_PROCS_MAXSIZE> _trackings; // Trackings in progress

    friend class Helioduino;

    inline HelioSchedulerSubData *schedulerData() const;
    inline bool hasSchedulerData() const;

    void updateDayTracking();
    void performScheduling();
    void broadcastDayChange();
};


// Scheduler Tracking Process Log Type
enum HelioTrackingLogType : signed char {
    HelioTrackingLogType_Setpoints,                         // Setpoints
    HelioTrackingLogType_Measures,                          // Measurements
};

// Scheduler Tracking Process Broadcast Type
enum HelioTrackingBroadcastType : signed char {
    HelioTrackingBroadcastType_Began,                       // Began main process
    HelioTrackingBroadcastType_Ended                        // Ended main process
};

// Scheduler Process Base
// Processes are created and managed by Scheduler to manage the day tracking
// sequences necessary for panel alignment.
struct HelioProcess {
    SharedPtr<HelioPanel> panel;                            // Panel

    Vector<HelioActuatorAttachment, HELIO_SCH_REQACTS_MAXSIZE> actuatorReqs; // Actuators required for this stage (keep-enabled list)

    time_t stageStart;                                      // Stage start time

    HelioProcess(SharedPtr<HelioPanel> panel);

    void clearActuatorReqs();
    void setActuatorReqs(const Vector<HelioActuatorAttachment, HELIO_SCH_REQACTS_MAXSIZE> &actuatorReqsIn);
};

// Scheduler Tracking Process
struct HelioTracking : public HelioProcess {
    enum : signed char {Init,TopOff,PreTrack,Track,Drain,Done,Unknown = -1} stage; // Current tracking stage

    time_t canTrackAfter;                                   // Time next tracking can occur (unix/UTC)
    time_t lastAirReport;                                   // Last time an air report was generated (unix/UTC)

    float axisSetpoints[2];                                 // Calculated axis setpoints for attached panel

    HelioTracking(SharedPtr<HelioPanel> panel);
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
    time_t airReportInterval;                               // Interval between air sensor reports, in seconds (default: 8hrs)

    HelioSchedulerSubData();
    void toJSONObject(JsonObject &objectOut) const;
    void fromJSONObject(JsonObjectConst &objectIn);
};

#endif // /ifndef HelioScheduler_H
