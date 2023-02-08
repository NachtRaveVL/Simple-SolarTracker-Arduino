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

    virtual void update() override;

    virtual bool canActivate(HelioActuator *actuator) override;

    template<typename T> inline void setAxisDriver(T axisDriver, hposi_t axisIndex = 0) { _axisDriver[axisIndex].setObject(axisDriver); }
    inline SharedPtr<HelioDriver> getAxisDriver(hposi_t axisIndex = 0) { return _axisDriver[axisIndex].getObject(); }

    template<typename T> inline void setAxisAngleSensor(T angleSensor, hposi_t axisIndex = 0) { axisAngle[axisIndex].setObject(angleSensor); }
    inline SharedPtr<HelioSensor> getAxisAngleSensor(hposi_t axisIndex = 0) { return _axisAngle[axisIndex].getObject(); }
    inline HelioSensorAttachment &getAxisAngle(hposi_t axisIndex = 0) { return _axisAngle[axisIndex]; }

    inline hposi_t getAxisCount() { return getPanelAxisCountFromType(getPanelType()); }

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
    HelioSensorAttachment _powerProd;                       // Power production sensor attachment
    HelioDriverAttachment _axisDriver[2];                   // Axis driver attachments
    HelioSensorAttachment _axisAngle[2];                    // Axis angle sensor attachments

    Signal<Helio_PanelState, HELIO_PANEL_SIGNAL_SLOTS> _stateSignal; // Panel state signal

    virtual HelioData *allocateData() const override;
    virtual void saveToData(HelioData *dataOut) override;

    virtual void handleState(Helio_PanelState panelState);
};


class HelioTrackingPanel : public HelioPanel {
public:

    // Following methods only relevant if the system is in Opposing LDR operation mode
    template<typename T> inline void setAxisAngleSensor(T angleSensor, hposi_t axisIndex = 0) { axisAngle[axisIndex].setObject(angleSensor); }
    inline SharedPtr<HelioSensor> getAxisAngleSensor(hposi_t axisIndex = 0) { return _axisAngle[axisIndex].getObject(); }
    inline HelioSensorAttachment &getAxisAngle(hposi_t axisIndex = 0) { return _axisAngle[axisIndex]; }

protected:
    HelioSensorAttachment *ldrSensors;                      // LDR sensors (only allocated if in LDR mode)
};


class HelioReflectingPanel : public HelioPanel {
public:

protected:
    float _reflectAzimuth;
    float _reflectHeading;
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
    char ldrSensor1[HELIO_NAME_MAXSIZE];
    char ldrSensor2[HELIO_NAME_MAXSIZE];
    char ldrSensor3[HELIO_NAME_MAXSIZE];
    char ldrSensor4[HELIO_NAME_MAXSIZE];

    HelioTrackingPanelData();
    virtual void toJSONObject(JsonObject &objectOut) const override;
    virtual void fromJSONObject(JsonObjectConst &objectIn) override;
};

// Reflecting Panel Serialization Data
struct HelioReflectingPanelData : HelioPanelData {
    float reflectAzimuth;                                   // Azimuth [0,360]
    float reflectElevation;                                 // Elevation [-90,90]

    HelioReflectingPanelData();
    virtual void toJSONObject(JsonObject &objectOut) const override;
    virtual void fromJSONObject(JsonObjectConst &objectIn) override;
};

#endif // /ifndef HelioPanels_H
