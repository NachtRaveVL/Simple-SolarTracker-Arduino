
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

    inline void setNeedsScheduling() { _needsScheduling = hasSchedulerData(); }
    inline bool needsScheduling() { return _needsScheduling; }
    inline bool inDaytimeMode() const { return _inDaytimeMode; }

    void setCleaningIntervalDays(unsigned int cleaningIntDays);
    void setPreDawnCleaningMins(unsigned int cleaningMins);
    void setPreDawnHeatingMins(unsigned int heatingMins);
    void setReportInterval(TimeSpan reportInterval);

    unsigned int getCleaningIntervalDays() const;
    unsigned int getPreDawnCleaningMins() const;
    unsigned int getPreDawnHeatingMins() const;
    TimeSpan getReportInterval() const;

    inline const Twilight &getDailyTwilight() const { return _dailyTwilight; }

protected:
    Twilight _dailyTwilight;                                // Daily twilight settings
    bool _needsScheduling;                                  // Needs rescheduling tracking flag
    bool _inDaytimeMode;                                    // Daytime mode flag
    hposi_t _lastDay[3];                                    // Last day tracking for rescheduling (Y-2k,M,D)
    Map<hkey_t, HelioTracking *, HELIO_SCH_PROCS_MAXSIZE> _trackings; // Panel tracking processes

    friend class Helioduino;
    friend struct HelioProcess;
    friend struct HelioTracking;

    inline HelioSchedulerSubData *schedulerData() const;
    inline bool hasSchedulerData() const;

    void updateDayTracking();
    void performScheduling();
    void broadcastDayChange();
};

// Scheduler Process Base
// Processes are created and managed by Scheduler to manage the daily control
// sequences necessary for solar panel alignment.
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
    enum : signed char {Init,Warm,Uncover,Clean,Track,Cover} stage; // Current tracking stage

    time_t canProcessAfter;                                 // Time next processing can occur (unix/UTC), else 0/disabled
    time_t lastEnvReport;                                   // Last time an environment report was generated (unix/UTC)
    bool stormingReported;                                  // Flag for storming reported
    bool nightSeqReported;                                  // Flag for night sequence reported
    bool coverSeqReported;                                  // Flag for cover sequence reported

    HelioTracking(SharedPtr<HelioPanel> panel);
    ~HelioTracking();

    void setupStaging();
    void update();

private:
    void reset();
};


// Scheduler Serialization Sub Data
// A part of HSYS system data.
struct HelioSchedulerSubData : public HelioSubData {
    uint8_t cleaningIntervalDays;                           // Interval between panel cleanings, in days (default: 14)
    uint8_t preDawnCleaningMins;                            // Time to run panel cleaning sprayers/wipers (if present/needed) before daily tracking starts, in minutes (default: 1)
    uint8_t preDawnHeatingMins;                             // Duration to run panel heaters/de-icers (if present/needed) before daily tracking starts, in minutes (default: 30)
    time_t reportInterval;                                  // Interval between environmental sensor reports, in seconds (default: 8hrs)

    HelioSchedulerSubData();
    void toJSONObject(JsonObject &objectOut) const;
    void fromJSONObject(JsonObjectConst &objectIn);
};

#endif // /ifndef HelioScheduler_H
