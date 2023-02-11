/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Panels
*/

#ifndef HelioPanels_H
#define HelioPanels_H

class HelioPanel;
class HelioTrackingPanel;
class HelioReflectingPanel;

struct HelioPanelData;
struct HelioTrackingPanelData;
struct HelioReflectingPanelData;

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
    virtual bool isAligned(bool poll = false) override;

    inline void setInDaytimeMode(bool inDaytimeMode) { _inDaytimeMode = inDaytimeMode; }
    inline void setHomePosition(const float *homePosition) { _homePosition[0] = homePosition[0]; _homePosition[1] = homePosition[1]; }

    // Recalculates sun's position in sky, called during polling loop if in sunPos operation mode
    void recalcSunPosition();

    template<typename T> inline void setAxisDriver(T axisDriver, hposi_t axisIndex = 0) { _axisDriver[axisIndex].setObject(axisDriver); }
    inline SharedPtr<HelioDriver> getAxisDriver(hposi_t axisIndex = 0) { return _axisDriver[axisIndex].getObject(); }

    template<typename T> inline void setAxisAngleSensor(T angleSensor, hposi_t axisIndex = 0) { _axisAngle[axisIndex].setObject(angleSensor); }
    inline SharedPtr<HelioSensor> getAxisAngleSensor(hposi_t axisIndex = 0) { return _axisAngle[axisIndex].getObject(); }
    inline HelioSensorAttachment &getAxisAngle(hposi_t axisIndex = 0, bool resolve = true) { if (resolve) { _axisAngle[axisIndex].resolve(); } return _axisAngle[axisIndex]; }

    inline hposi_t getAxisCount() const { return getPanelAxisCountFromType(getPanelType()); }
    inline bool isSingleAxis() const { return getAxisCount() == 1; }
    inline bool isMultiAxis() const { return getAxisCount() > 1; }
    inline bool isEquatorialCoords() const { return getIsEquatorialCoordsFromType(getPanelType()); }
    inline bool isHorizontalCoords() const { return !isEquatorialCoords(); }

    // LDR methods only relevant if in brightness balancing (LDR) operation mode
    template<typename T> inline void setLDRSensor(T ldrSensor, hposi_t ldrIndex = 0) { getLDR(ldrIndex).setObject(ldrSensor); }
    inline SharedPtr<HelioSensor> getLDRSensor(hposi_t ldrIndex = 0) { return getLDR(ldrIndex).getObject(); }
    inline HelioSensorAttachment &getLDR(hposi_t ldrIndex = 0, bool resolve = true) { if (resolve) { _ldrSensors[ldrIndex].resolve(); } return _ldrSensors[ldrIndex]; }
    inline hposi_t getLDRCount() const { return (getPanelAxisCountFromType(getPanelType()) << 1); }

    inline bool isLDRMode() const { return _ldrSensors; }
    inline bool isSPCMode() const { return !isLDRMode(); }

    virtual void setPowerUnits(Helio_UnitsType powerUnits) override;
    virtual Helio_UnitsType getPowerUnits() const override;

    virtual HelioSensorAttachment &getPowerProduction(bool poll = false) override;

    inline Helio_PanelType getPanelType() const { return _id.objTypeAs.panelType; }
    inline hposi_t getPanelIndex() const { return _id.posIndex; }
    inline Helio_PanelState getPanelState() const { return _panelState; }

    Signal<Helio_PanelState, HELIO_PANEL_SIGNAL_SLOTS> &getStateSignal();

protected:
    Helio_UnitsType _powerUnits;                            // Power units preferred
    Helio_PanelState _panelState;                           // Current panel state
    float _homePosition[2];                                 // Home position vector (azi,ele or RA,dec)
    double *_sunPosition;                                   // Sun position vector (azi,ele or RA,dec, owned/only allocated in sunPos mode)
    bool _inDaytimeMode;                                    // Daytime mode flag (copied from scheduler)
    HelioSensorAttachment _powerProd;                       // Power production sensor attachment
    HelioDriverAttachment _axisDriver[2];                   // Axis driver attachments
    HelioSensorAttachment _axisAngle[2];                    // Axis angle sensor attachments
    HelioSensorAttachment *_ldrSensors;                     // LDR sensors (owned/only allocated in LDR mode)
    Signal<Helio_PanelState, HELIO_PANEL_SIGNAL_SLOTS> _stateSignal; // Panel state signal

    inline void allocateLDRSensors() { if (!_ldrSensors) { _ldrSensors = new HelioSensorAttachment[getLDRCount()]; for (hposi_t ldrIndex = 0; _ldrSensors && ldrIndex < getLDRCount(); ++ldrIndex) { _ldrSensors[ldrIndex].setParent(this); } } }
    inline void allocateSPCStorage() { if (!_sunPosition) { _sunPosition = new double[2]; if (_sunPosition) { _sunPosition[0] = DBL_UNDEF; _sunPosition[1] = DBL_UNDEF; } } }

    virtual HelioData *allocateData() const override;
    virtual void saveToData(HelioData *dataOut) override;

    virtual void handleState(Helio_PanelState panelState) = 0;
};


// Tracking Panel
class HelioTrackingPanel : public HelioPanel {
public:
    HelioTrackingPanel(Helio_PanelType panelType,
                       hposi_t panelIndex,
                       int classType = Tracking);
    HelioTrackingPanel(const HelioTrackingPanelData *dataIn);
    virtual ~HelioTrackingPanel();

    virtual bool isAligned(bool poll = false) override;

protected:

    virtual void handleState(Helio_PanelState panelState) override;
};


// Reflecting Panel
class HelioReflectingPanel : public HelioPanel {
public:
    HelioReflectingPanel(Helio_PanelType panelType,
                         hposi_t panelIndex,
                         int classType = Reflecting);
    HelioReflectingPanel(const HelioReflectingPanelData *dataIn);
    virtual ~HelioReflectingPanel();

    virtual bool isAligned(bool poll = false) override;

    inline void setReflectPosition(const float *reflectPosition) { _reflectPosition[0] = reflectPosition[0]; _reflectPosition[1] = reflectPosition[1]; }

protected:
    float _reflectPosition[2];                              // Vector direction to reflect towards (azi,ele or RA,dec)
    float _facingPosition[2];                               // Vector direction of mirror facing (azi,ele or RA,dec)

    virtual void handleState(Helio_PanelState panelState) override;
};


// Panel Serialization Data
struct HelioPanelData : public HelioObjectData {
    Helio_UnitsType powerUnits;                             // Power units
    float homePosition[2];                                  // Home position vector (azi,ele or RA,dec)
    char powerSensor[HELIO_NAME_MAXSIZE];                   // Power production sensor
    char angleSensor1[HELIO_NAME_MAXSIZE];                  // Angle sensor for axis 1
    char angleSensor2[HELIO_NAME_MAXSIZE];                  // Angle sensor for axis 2
    char ldrSensor1[HELIO_NAME_MAXSIZE];                    // LDR for axis 1, min-side
    char ldrSensor2[HELIO_NAME_MAXSIZE];                    // LDR for axis 1, max-side
    char ldrSensor3[HELIO_NAME_MAXSIZE];                    // LDR for axis 2, min-side
    char ldrSensor4[HELIO_NAME_MAXSIZE];                    // LDR for axis 2, max-side

    HelioPanelData();
    virtual void toJSONObject(JsonObject &objectOut) const override;
    virtual void fromJSONObject(JsonObjectConst &objectIn) override;
};

// Tracking Panel Serialization Data
struct HelioTrackingPanelData : HelioPanelData {
    
    HelioTrackingPanelData();
    virtual void toJSONObject(JsonObject &objectOut) const override;
    virtual void fromJSONObject(JsonObjectConst &objectIn) override;
};

// Reflecting Panel Serialization Data
struct HelioReflectingPanelData : HelioPanelData {
    float reflectTowards[2];                                // Vector to reflect towards (azi,ele or RA,dec)

    HelioReflectingPanelData();
    virtual void toJSONObject(JsonObject &objectOut) const override;
    virtual void fromJSONObject(JsonObjectConst &objectIn) override;
};

#endif // /ifndef HelioPanels_H
