/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Panels
*/

#ifndef HelioPanels_H
#define HelioPanels_H

class HelioPanel;

struct HelioPanelData;

#include "Helioduino.h"
#include "HelioTriggers.h"

// Creates panel object from passed panel data (return ownership transfer - user code *must* delete returned object)
extern HelioPanel *newPanelObjectFromData(const HelioPanelData *dataIn);


// Panel Base
// This is the base class for all panels, which defines how the panel is
// identified, where it lives, what's attached to it, if it is full or empty, and
// who can activate under it.
class HelioPanel : public HelioObject, public HelioPanelObjectInterface, public HelioPowerProductionSensorAttachmentInterface {
public:
    const enum : signed char { Tracking, Reflecting, Unknown = -1 } classType; // Panel class type (custom RTTI)
    inline bool isTrackingClass() const { return classType == Tracking; }
    inline bool isReflectingClass() const { return classType == Reflecting; }
    inline bool isUnknownClass() const { return classType <= Unknown; }

    HelioPanel(Helio_PanelType panelType,
               hposi_t panelIndex,
               int classType = Unknown);
    HelioPanel(const HelioPanelData *dataIn);
    virtual ~HelioPanel();

    virtual void update() override;

    virtual bool canActivate(HelioActuator *actuator) override;
    virtual bool isAligned() override;

    inline void setInDaytimeMode(bool inDaytimeMode) { _inDaytimeMode = inDaytimeMode; }
    inline void setHomePosition(float homePosition[2]) { _homePosition[0] = homePosition[0]; _homePosition[1] = homePosition[1]; }
    inline void setSunPosition(double sunPosition[2]) { _sunPosition[0] = sunPosition[0]; _sunPosition[1] = sunPosition[1]; }

    void recalcSunPosition();

    template<typename T> inline void setAxisDriver(T axisDriver, hposi_t axisIndex = 0) { _axisDriver[axisIndex].setObject(axisDriver); }
    inline SharedPtr<HelioDriver> getAxisDriver(hposi_t axisIndex = 0) { return _axisDriver[axisIndex].getObject(); }

    template<typename T> inline void setAxisAngleSensor(T angleSensor, hposi_t axisIndex = 0) { axisAngle[axisIndex].setObject(angleSensor); }
    inline SharedPtr<HelioSensor> getAxisAngleSensor(hposi_t axisIndex = 0) { return _axisAngle[axisIndex].getObject(); }
    inline HelioSensorAttachment &getAxisAngle(hposi_t axisIndex = 0) { return _axisAngle[axisIndex]; }

    inline hposi_t getAxisCount() { return getPanelAxisCountFromType(getPanelType()); }
    inline bool isEquatorialCoords() { return getIsEquatorialCoordsFromType(getPanelType()); }

    virtual void setPowerUnits(Helio_UnitsType powerUnits);
    virtual Helio_UnitsType getPowerUnits() const { return definedUnitsElse(_powerUnits, defaultPowerUnits()); }

    virtual HelioSensorAttachment &getPowerProduction(bool poll = false) override;

    inline Helio_PanelType getPanelType() const { return _id.objTypeAs.panelType; }
    inline hposi_t getPanelIndex() const { return _id.posIndex; }
    inline Helio_PanelState getPanelState() const { return _panelState; }

    Signal<Helio_PanelState, HELIO_PANEL_SIGNAL_SLOTS> &getStateSignal();

protected:
    Helio_UnitsType _powerUnits;                            // Power units preferred
    Helio_PanelState _panelState;                           // Current panel state
    float _homePosition[2];                                 // Home position vector
    double _sunPosition[2];                                 // Sun position vector
    bool _inDaytimeMode;                                    // Daytime mode flag
    HelioSensorAttachment _powerProd;                       // Power production sensor attachment
    HelioDriverAttachment _axisDriver[2];                   // Axis driver attachments
    HelioSensorAttachment _axisAngle[2];                    // Axis angle sensor attachments
    Signal<Helio_PanelState, HELIO_PANEL_SIGNAL_SLOTS> _stateSignal; // Panel state signal

    virtual HelioData *allocateData() const override;
    virtual void saveToData(HelioData *dataOut) override;

    virtual void handleState(Helio_PanelState panelState);
};


// Tracking Panel
class HelioTrackingPanel : public HelioPanel {
public:
    HelioTrackingPanel(Helio_PanelType panelType,
                       hposi_t panelIndex,
                       int classType = Tracking);
    HelioTrackingPanel(const HelioTrackingPanelData *dataIn);
    virtual ~HelioTrackingPanel();

    // Following methods only relevant if the system is in brightness balancing (LDR) operation mode
    template<typename T> inline void setLDRSensor(T ldrSensor, hposi_t ldrIndex = 0) { _ldrSensors[ldrIndex].setObject(ldrSensor); }
    inline SharedPtr<HelioSensor> getLDRSensor(hposi_t ldrIndex = 0) { return _ldrSensors[ldrIndex].getObject(); }
    inline HelioSensorAttachment &getLDR(hposi_t ldrIndex = 0) { return _ldrSensors[ldrIndex]; }

protected:
    HelioSensorAttachment *_ldrSensors;                     // LDR sensors (only allocated if in LDR mode)

    void updateState();
};


// Reflecting Panel
class HelioReflectingPanel : public HelioPanel {
public:
    HelioReflectingPanel(Helio_PanelType panelType,
                         hposi_t panelIndex,
                         HelioDoubleMeasurement reflectTowards,
                         int classType = Tracking);
    HelioReflectingPanel(const HelioReflectingPanelData *dataIn);
    virtual ~HelioReflectingPanel();

    virtual bool isAligned() override;

protected:
    float _reflectPosition[2];                              // Vector direction to reflect towards
    float _facingPosition[2];                               // Vector direction of mirror facing

    void updateState();
};


// Panel Serialization Data
struct HelioPanelData : public HelioObjectData {
    Helio_UnitsType powerUnits;
    char powerSensor[HELIO_NAME_MAXSIZE];
    char angleSensor1[HELIO_NAME_MAXSIZE];
    char angleSensor2[HELIO_NAME_MAXSIZE];

    HelioPanelData();
    virtual void toJSONObject(JsonObject &objectOut) const override;
    virtual void fromJSONObject(JsonObjectConst &objectIn) override;
};

// Tracking Panel Serialization Data
struct HelioTrackingPanelData : HelioPanelData {
    char ldrSensor1[HELIO_NAME_MAXSIZE];                    // LDR for axis 1, min-side
    char ldrSensor2[HELIO_NAME_MAXSIZE];                    // LDR for axis 1, max-side
    char ldrSensor3[HELIO_NAME_MAXSIZE];                    // LDR for axis 2, min-side
    char ldrSensor4[HELIO_NAME_MAXSIZE];                    // LDR for axis 2, max-side

    HelioTrackingPanelData();
    virtual void toJSONObject(JsonObject &objectOut) const override;
    virtual void fromJSONObject(JsonObjectConst &objectIn) override;
};

// Reflecting Panel Serialization Data
struct HelioReflectingPanelData : HelioPanelData {
    float reflectAzimuth;                                   // Azimuth/RightAscension [0,360)
    float reflectElevation;                                 // Elevation/Declination [90,-90)

    HelioReflectingPanelData();
    virtual void toJSONObject(JsonObject &objectOut) const override;
    virtual void fromJSONObject(JsonObjectConst &objectIn) override;
};

#endif // /ifndef HelioPanels_H
