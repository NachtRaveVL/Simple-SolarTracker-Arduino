
/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Scheduler
*/

#ifndef HelioScheduler_H
#define HelioScheduler_H

class HelioScheduler;
struct HelioSchedulerSubData;
struct HelioFeeding;
struct HelioLighting;

#include "Helioduino.h"

// Scheduler
// The Scheduler acts as the system's main scheduling attendant, who looks through all
// the various equipment and crops you have programmed in, and figures out the best
// case feeding and lighting sequences that should occur to support them. It is also
// responsible for setting up and maintaining the system balancers that get assigned to
// feed panels, which drives the various balancing processes in use, as well as
// determining when significant time changes have occurred and broadcasting such out.
class HelioScheduler {
public:
    HelioScheduler();
    ~HelioScheduler();

    void update();

    void setupWaterPHBalancer(HelioPanel *panel, SharedPtr<HelioBalancer> waterPHBalancer);
    void setupWaterTDSBalancer(HelioPanel *panel, SharedPtr<HelioBalancer> waterTDSBalancer);
    void setupWaterTemperatureBalancer(HelioPanel *panel, SharedPtr<HelioBalancer> waterTempBalancer);
    void setupAirTemperatureBalancer(HelioPanel *panel, SharedPtr<HelioBalancer> airTempBalancer);
    void setupAirCO2Balancer(HelioPanel *panel, SharedPtr<HelioBalancer> airCO2Balancer);

    void setBaseFeedMultiplier(float feedMultiplier);
    void setWeeklyDosingRate(int weekIndex, float dosingRate, Helio_PanelType panelType = Helio_PanelType_NutrientPremix);
    void setStandardDosingRate(float dosingRate, Helio_PanelType panelType);
    void setFlushWeek(int weekIndex);
    void setTotalFeedingsDay(unsigned int feedingsDay);
    void setPreFeedAeratorMins(unsigned int aeratorMins);
    void setPreLightSprayMins(unsigned int sprayMins);
    void setAirReportInterval(TimeSpan interval);

    inline void setNeedsScheduling();
    inline bool needsScheduling() { return _needsScheduling; }

    float getCombinedDosingRate(HelioPanel *panel, Helio_PanelType panelType = Helio_PanelType_NutrientPremix);
    inline bool inDaytimeMode() const { return _inDaytimeMode; }

    float getBaseFeedMultiplier() const;
    float getWeeklyDosingRate(int weekIndex, Helio_PanelType panelType = Helio_PanelType_NutrientPremix) const;
    float getStandardDosingRate(Helio_PanelType panelType) const;
    bool isFlushWeek(int weekIndex);
    unsigned int getTotalFeedingsDay() const;
    unsigned int getPreFeedAeratorMins() const;
    unsigned int getPreLightSprayMins() const;
    TimeSpan getAirReportInterval() const;

protected:
    bool _inDaytimeMode;                                    // Whenever in daytime feeding mode or not
    bool _needsScheduling;                                  // Needs rescheduling tracking flag
    int _lastDayNum;                                        // Last day number tracking for daily rescheduling tracking
    Map<Helio_KeyType, HelioFeeding *, HELIO_SCH_FEEDRES_MAXSIZE> _feedings; // Feedings in progress
    Map<Helio_KeyType, HelioLighting *, HELIO_SCH_FEEDRES_MAXSIZE> _lightings; // Lightings in progress

    friend class Helioduino;

    inline HelioSchedulerSubData *schedulerData() const;
    inline bool hasSchedulerData() const;

    void updateDayTracking();
    void performScheduling();
    void broadcastDayChange();
};


// Scheduler Feeding Process Log Type
enum HelioFeedingLogType : signed char {
    HelioFeedingLogType_WaterSetpoints,                     // Water setpoints
    HelioFeedingLogType_WaterMeasures,                      // Water measurements
    HelioFeedingLogType_AirSetpoints,                       // Air setpoints
    HelioFeedingLogType_AirMeasures                         // Air measurements
};

// Scheduler Feeding Process Broadcast Type
enum HelioFeedingBroadcastType : signed char {
    HelioFeedingBroadcastType_Began,                        // Began main process
    HelioFeedingBroadcastType_Ended                         // Ended main process
};

// Scheduler Process Base
// Processes are created and managed by Scheduler to manage the feeding and lighting
// sequences necessary for crops to grow.
struct HelioProcess {
    SharedPtr<HelioFeedPanel> feedRes;                      // Feed panel
    Vector<SharedPtr<HelioActuator>, HELIO_SCH_REQACTUATORS_MAXSIZE> actuatorReqs; // Actuators required for this stage (keep-enabled list)

    time_t stageStart;                                      // Stage start time

    HelioProcess(SharedPtr<HelioFeedPanel> feedRes);

    void clearActuatorReqs();
    void setActuatorReqs(const Vector<SharedPtr<HelioActuator>, HELIO_SCH_REQACTUATORS_MAXSIZE> &actuatorReqsIn);
};

// Scheduler Feeding Process
struct HelioFeeding : public HelioProcess {
    enum : signed char {Init,TopOff,PreFeed,Feed,Drain,Done,Unknown = -1} stage; // Current feeding stage

    time_t canFeedAfter;                                    // Time next feeding can occur (UTC)
    time_t lastAirReport;                                   // Last time an air report was generated (UTC)

    float phSetpoint;                                       // Calculated pH setpoint for attached crops
    float tdsSetpoint;                                      // Calculated TDS setpoint for attached crops
    float waterTempSetpoint;                                // Calculated water temp setpoint for attached crops
    float airTempSetpoint;                                  // Calculated air temp setpoint for attached crops
    float co2Setpoint;                                      // Calculated co2 level setpoint for attached crops

    HelioFeeding(SharedPtr<HelioFeedPanel> feedRes);
    ~HelioFeeding();

    void recalcFeeding();
    void setupStaging();
    void update();

private:
    void reset();
    void logFeeding(HelioFeedingLogType logType);
    void broadcastFeeding(HelioFeedingBroadcastType broadcastType);
};

// Scheduler Lighting Process
struct HelioLighting : public HelioProcess {
    enum : signed char {Init,Spray,Light,Done,Unknown = -1} stage; // Current lighting stage

    time_t sprayStart;                                      // Time when spraying should start (TZ)
    time_t lightStart;                                      // Time when lighting should start / spraying should end (TZ, same as sprayStart when no spraying needed)
    time_t lightEnd;                                        // Time when lighting should finish (TZ)

    float lightHours;                                       // Calculated light hours for attached crops

    HelioLighting(SharedPtr<HelioFeedPanel> feedRes);
    ~HelioLighting();

    void recalcLighting();
    void setupStaging();
    void update();
};


// Scheduler Serialization Sub Data
// A part of HSYS system data.
struct HelioSchedulerSubData : public HelioSubData {
    float baseFeedMultiplier;                               // Feed aggressiveness base TDS/EC multiplier (applies to *ALL* feeding solutions in use - default: 1)
    float weeklyDosingRates[HELIO_CROP_GROWWEEKS_MAX];      // Nutrient dosing rate percentages (applies to any nutrient premixes in use - default: 1)
    float stdDosingRates[3];                                // Standard dosing rates for fresh water, pH up, and pH down (default: 1,1/2,1/2)
    uint8_t totalFeedingsDay;                               // Total number of feedings per day, if any (else 0 for disable - default: 0)
    uint8_t preFeedAeratorMins;                             // Minimum time to run aerators (if present) before feed pumps turn on, in minutes (default: 30)
    uint8_t preLightSprayMins;                              // Minimum time to run sprayers/sprinklers (if present/needed) before grow lights turn on, in minutes (default: 60)
    time_t airReportInterval;                               // Interval between air sensor reports, in seconds (default: 8hrs)

    HelioSchedulerSubData();
    void toJSONObject(JsonObject &objectOut) const;
    void fromJSONObject(JsonObjectConst &objectIn);
};

#endif // /ifndef HelioScheduler_H
