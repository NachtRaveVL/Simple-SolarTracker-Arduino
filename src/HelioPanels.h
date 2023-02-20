/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Panels
*/

#ifndef HelioPanels_H
#define HelioPanels_H

class HelioPanel;
class HelioBalancingPanel;
class HelioTrackingPanel;
class HelioReflectingPanel;

struct HelioPanelData;
struct HelioBalancingPanelData;
struct HelioTrackingPanelData;
struct HelioReflectingPanelData;

#include "Helioduino.h"
#include "HelioTriggers.h"

// Creates panel object from passed panel data (return ownership transfer - user code *must* delete returned object)
extern HelioPanel *newPanelObjectFromData(const HelioPanelData *dataIn);


// Panel Base
// This is the base class for all panels, which defines how the panel is identified,
// where it lives, what's attached to it, if it is aligned or not, and who can
// activate under it.
class HelioPanel : public HelioObject, public HelioPanelObjectInterface, public HelioPowerUnitsInterfaceStorage, public HelioPowerProductionSensorAttachmentInterface {
public:
    const enum : signed char { Balancing, Tracking, Reflecting, Unknown = -1 } classType; // Panel class type (custom RTTI)
    inline bool isBalancingClass() const { return classType == Balancing; }
    inline bool isTrackingClass() const { return classType == Tracking; }
    inline bool isReflectingClass() const { return classType == Reflecting; }
    inline bool isAnyTrackingClass() const { return isTrackingClass() || isReflectingClass(); }
    inline bool isUnknownClass() const { return classType <= Unknown; }

    HelioPanel(Helio_PanelType panelType,
               hposi_t panelIndex,
               float alignedTolerance,
               int classType = Unknown);
    HelioPanel(const HelioPanelData *dataIn);
    virtual ~HelioPanel();

    virtual void update() override;

    virtual bool canActivate(HelioActuator *actuator) override;

    inline void setHomePosition(const float *homePosition) { _homePosition[0] = homePosition[0]; _homePosition[1] = homePosition[1]; }
    inline void setAxisOffset(const float *axisOffset) { _axisOffset[0] = axisOffset[0]; _axisOffset[1] = axisOffset[1]; }
    inline const float *getHomePosition() const { return _homePosition; }
    inline const float *getAxisOffset() const { return _axisOffset; }

    inline void setInDaytimeMode(bool inDaytimeMode) { _inDaytimeMode = inDaytimeMode; }
    inline bool getInDaytimeMode() const { return _inDaytimeMode; }

    inline hposi_t getAxisCount() const { return getPanelAxisCountFromType(getPanelType()); }
    inline bool isEquatorialCoords() const { return getIsEquatorialCoordsFromType(getPanelType()); }
    inline bool isHorizontalCoords() const { return getIsHorizontalCoordsFromType(getPanelType()); }
    inline bool drivesHorizontalAxis() const { return getDrivesHorizontalAxis(getPanelType()); }
    inline bool drivesVerticalAxis() const { return getDrivesVerticalAxis(getPanelType()); }

    template<typename T> inline void setAxisDriver(T axisDriver, hposi_t axisIndex = 0) { _axisDriver[axisIndex].setObject(axisDriver); }
    inline SharedPtr<HelioDriver> getAxisDriver(hposi_t axisIndex = 0) { return _axisDriver[axisIndex].getObject(); }
    inline HelioDriverAttachment &getAxisDriverAttachment(hposi_t axisIndex = 0) { return _axisDriver[axisIndex]; }

    virtual void setPowerUnits(Helio_UnitsType powerUnits) override;

    virtual HelioSensorAttachment &getPowerProductionSensorAttachment() override;

    inline Helio_PanelType getPanelType() const { return _id.objTypeAs.panelType; }
    inline hposi_t getPanelIndex() const { return _id.posIndex; }
    inline Helio_PanelState getPanelState() const { return _panelState; }

    Signal<Helio_PanelState, HELIO_PANEL_SIGNAL_SLOTS> &getStateSignal();

protected:
    Helio_PanelState _panelState;                           // Panel state (last handled)
    float _alignedTolerance;                                // Alignment tolerance
    float _homePosition[2];                                 // Home position (azi,ele or RA,dec)
    float _axisOffset[2];                                   // Axis position calibration offset (azi,ele or RA,dec)
    bool _inDaytimeMode;                                    // Daytime mode flag (copied from scheduler)
    HelioSensorAttachment _powerProd;                       // Power production sensor attachment
    HelioDriverAttachment _axisDriver[2];                   // Axis driver attachments
    Signal<Helio_PanelState, HELIO_PANEL_SIGNAL_SLOTS> _stateSignal; // Panel state signal

    virtual HelioData *allocateData() const override;
    virtual void saveToData(HelioData *dataOut) override;

    virtual void handleState(Helio_PanelState panelState) = 0;
};


// LDR Balancing Panel
// A simple panel that positions itself by balancing two opposing LDR sensors.
// Cheap and easy way to orient a panel, requiring least amount of effort.
class HelioBalancingPanel : public HelioPanel {
public:
    HelioBalancingPanel(Helio_PanelType panelType,
                       hposi_t panelIndex,
                       float intTolerance = HELIO_PANEL_ALIGN_LDRTOL,
                       float minIntensity = HELIO_PANEL_ALIGN_LDRMIN,
                       int classType = Balancing);
    HelioBalancingPanel(const HelioBalancingPanelData *dataIn);
    virtual ~HelioBalancingPanel();

    virtual void update() override;

    virtual bool isAligned(bool poll = false) override;

    template<typename T> inline void setLDRSensor(T ldrSensor, hposi_t ldrIndex = 0) { _ldrSensors[ldrIndex].setObject(ldrSensor); }
    inline SharedPtr<HelioSensor> getLDRSensor(hposi_t ldrIndex = 0, bool poll = false) { _ldrSensors[ldrIndex].updateIfNeeded(poll); return _ldrSensors[ldrIndex].getObject(); }
    inline HelioSensorAttachment &getLDRSensorAttachment(hposi_t ldrIndex = 0) { return _ldrSensors[ldrIndex]; }
    inline hposi_t getLDRCount() const { return (getPanelAxisCountFromType(getPanelType()) << 1); }

protected:
    float _minIntensity;                                    // Minimum LDR light intensity for alignment query
    HelioSensorAttachment _ldrSensors[4];                   // LDR light intensity sensor attachments

    virtual void saveToData(HelioData *dataOut) override;

    virtual void handleState(Helio_PanelState panelState) override;
};


// Smart Tracking Panel
// A panel that is able to calculate the sun's position in the sky and orient
// towards it, maximizing the energy output of the panel across the day.
class HelioTrackingPanel : public HelioPanel, public HelioPowerUsageSensorAttachmentInterface {
public:
    HelioTrackingPanel(Helio_PanelType panelType,
                       hposi_t panelIndex,
                       float angleTolerance = HELIO_PANEL_ALIGN_DEGTOL,
                       int classType = Tracking);
    HelioTrackingPanel(const HelioTrackingPanelData *dataIn);
    virtual ~HelioTrackingPanel();

    virtual void update() override;

    virtual bool isAligned(bool poll = false) override;

    template<typename T> inline void setAxisAngleSensor(T angleSensor, hposi_t axisIndex = 0) { _axisAngle[axisIndex].setObject(angleSensor); }
    inline SharedPtr<HelioSensor> getAxisAngleSensor(hposi_t axisIndex = 0, bool poll = false) { _axisAngle[axisIndex].updateIfNeeded(poll); return _axisAngle[axisIndex].getObject(poll); }
    inline HelioSensorAttachment &getAxisAngleSensorAttachment(hposi_t axisIndex = 0) { return _axisAngle[axisIndex]; }

    inline DateTime getLastAlignmentChangeTime() const { return localTime(_lastChangeTime); }
    inline void notifyAlignmentChanged() { _lastChangeTime = unixNow(); }

    virtual void setPowerUnits(Helio_UnitsType powerUnits) override;

    virtual HelioSensorAttachment &getPowerUsageSensorAttachment() override;

    inline void notifyDayChanged() { recalcSunPosition(); recalcFacingPosition(); }

protected:
    time_t _lastChangeTime;                                 // Last panel alignment/maintenance date (UTC)
    double _sunPosition[2];                                 // Sun position (azi,ele or RA,dec)
    float _facingPosition[2];                               // Resolved facing position (azi,ele or RA,dec)
    HelioSensorAttachment _powerUsage;                      // Power usage sensor attachment
    HelioSensorAttachment _axisAngle[2];                    // Axis angle sensor attachments

    virtual void saveToData(HelioData *dataOut) override;

    virtual void handleState(Helio_PanelState panelState) override;

    void recalcSunPosition();
    virtual void recalcFacingPosition();
};


// Reflecting Mirror Panel
// A panel that is also able to calculate the sun's position in the sky, but instead
// treats the panel as a mirror that reflects the sun's light across the day to a preset
// direction (that has, e.g., statically installed panels or a shared heat exchanger).
class HelioReflectingPanel : public HelioTrackingPanel {
public:
    HelioReflectingPanel(Helio_PanelType panelType,
                         hposi_t panelIndex,
                         float angleTolerance = HELIO_PANEL_ALIGN_DEGTOL,
                         int classType = Reflecting);
    HelioReflectingPanel(const HelioReflectingPanelData *dataIn);
    virtual ~HelioReflectingPanel();

    inline void setReflectPosition(const float *reflectPosition) { _reflectPosition[0] = reflectPosition[0]; _reflectPosition[1] = reflectPosition[1]; }

protected:
    float _reflectPosition[2];                              // Position to reflect towards (azi,ele or RA,dec)

    virtual void saveToData(HelioData *dataOut) override;

    virtual void recalcFacingPosition() override;
};


// Panel Serialization Data
struct HelioPanelData : public HelioObjectData {
    Helio_UnitsType powerUnits;                             // Power units
    float alignedTolerance;                                 // Alignment tolerance (ldr% or axis deg)
    float homePosition[2];                                  // Home/return position (azi,ele or RA,dec)
    float axisOffset[2];                                    // Axis position calibration offset (azi,ele or RA,dec)
    char powerProdSensor[HELIO_NAME_MAXSIZE];               // Power production sensor

    HelioPanelData();
    virtual void toJSONObject(JsonObject &objectOut) const override;
    virtual void fromJSONObject(JsonObjectConst &objectIn) override;
};

// Balancing Panel Serialization Data
struct HelioBalancingPanelData : HelioPanelData {
    float minIntensity;                                     // Minimum LDR intensity
    char ldrSensor1[HELIO_NAME_MAXSIZE];                    // LDR for horz axis, min/neg-side
    char ldrSensor2[HELIO_NAME_MAXSIZE];                    // LDR for horz axis, max/pos-side
    char ldrSensor3[HELIO_NAME_MAXSIZE];                    // LDR for vert axis, min/neg-side
    char ldrSensor4[HELIO_NAME_MAXSIZE];                    // LDR for vert axis, max/pos-side

    HelioBalancingPanelData();
    virtual void toJSONObject(JsonObject &objectOut) const override;
    virtual void fromJSONObject(JsonObjectConst &objectIn) override;
};

// Tracking Panel Serialization Data
struct HelioTrackingPanelData : HelioPanelData {
    float axisPosition[2];                                  // Axis angle position (azi,ele or RA,dec)
    time_t lastChangeTime;                                  // Last alignment change/maintenance time (UTC)
    char axisSensor1[HELIO_NAME_MAXSIZE];                   // Horizontal axis sensor
    char axisSensor2[HELIO_NAME_MAXSIZE];                   // Vertical axis sensor
    char powerUsageSensor[HELIO_NAME_MAXSIZE];              // Power usage sensor

    HelioTrackingPanelData();
    virtual void toJSONObject(JsonObject &objectOut) const override;
    virtual void fromJSONObject(JsonObjectConst &objectIn) override;
};

// Reflecting Panel Serialization Data
struct HelioReflectingPanelData : HelioTrackingPanelData {
    float reflectPosition[2];                               // Reflect towards position (azi,ele or RA,dec)

    HelioReflectingPanelData();
    virtual void toJSONObject(JsonObject &objectOut) const override;
    virtual void fromJSONObject(JsonObjectConst &objectIn) override;
};

#endif // /ifndef HelioPanels_H
