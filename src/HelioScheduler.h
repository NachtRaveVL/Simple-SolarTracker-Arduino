
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
// the various equipment and panels you have programmed in, and figures out the best
// case tracking processes that should occur to support them. It is also responsible
// for setting up and maintaining the drivers that get assigned to panels (such as the
// various mechanical orientation devices in use), as well as determining when
// significant time or event changes have occurred and broadcasting such out.
class HelioScheduler {
public:
    HelioScheduler();
    ~HelioScheduler();

    void update();

    void setupPanelDriver(HelioPanel *panel, hposi_t axisIndex, SharedPtr<HelioDriver> panelDriver);

    void setAirReportInterval(TimeSpan interval);

    inline void setNeedsScheduling();
    inline bool needsScheduling() { return _needsScheduling; }
    inline bool inDaytimeMode() const { return _inDaytimeMode; }

    TimeSpan getAirReportInterval() const;

    inline const Twilight &getDailyTwilight() const { return _dailyTwilight; }

protected:
    Twilight _dailyTwilight;                                // Daily twilight settings
    bool _needsScheduling;                                  // Needs rescheduling tracking flag
    bool _inDaytimeMode;                                    // Daytime mode flag
    hposi_t _lastDay[3];                                    // Last day tracking for rescheduling (Y,M,D)
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
    enum : signed char {Init,Preheat,Uncover,Clean,Track,Cover,Unknown = -1} stage; // Current tracking stage

    time_t canProcessAfter;                                 // Time next processing can occur (unix/UTC), else 0/disabled
    time_t lastAirReport;                                   // Last time an air report was generated (unix/UTC)

    HelioTracking(SharedPtr<HelioPanel> panel);
    ~HelioTracking();

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
