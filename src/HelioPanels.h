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
class HelioPanel : public HelioObject, public HelioPanelObjectInterface {
public:
    const enum : signed char { Single, Unknown = -1 } classType; // Panel class type (custom RTTI)
    inline bool isSingleClass() const { return classType == Single; }
    inline bool isUnknownClass() const { return classType <= Unknown; }

    HelioPanel(Helio_PanelType panelType,
               Helio_PositionIndex panelIndex,
               int classType = Unknown);
    HelioPanel(const HelioPanelData *dataIn);

    virtual void update() override;

    virtual bool canActivate(HelioActuator *actuator);
    //virtual bool isFilled() = 0;
    //virtual bool isEmpty() = 0;

    //virtual void setVolumeUnits(Helio_UnitsType volumeUnits);
    //virtual Helio_UnitsType getVolumeUnits() const { return definedUnitsElse(_volumeUnits, defaultLiquidVolumeUnits()); }

    //virtual HelioSensorAttachment &getWaterVolume(bool poll = false) = 0;

    inline Helio_PanelType getPanelType() const { return _id.objTypeAs.panelType; }
    inline Helio_PositionIndex getPanelIndex() const { return _id.posIndex; }

    //Signal<HelioPanel *, HELIO_PANEL_STATE_SLOTS> &getFilledSignal();
    //Signal<HelioPanel *, HELIO_PANEL_STATE_SLOTS> &getEmptySignal();

protected:
    Helio_UnitsType _volumeUnits;                     // Volume units preferred

    //Helio_TriggerState _filledState;                  // Current filled state
    //Helio_TriggerState _emptyState;                   // Current empty state

    //Signal<HelioPanel *, HELIO_PANEL_STATE_SLOTS> _filledSignal; // Filled state signal
    //Signal<HelioPanel *, HELIO_PANEL_STATE_SLOTS> _emptySignal; // Empty state signal

    virtual HelioData *allocateData() const override;
    virtual void saveToData(HelioData *dataOut) override;

    //virtual void handleFilled(Helio_TriggerState filledState);
    //virtual void handleEmpty(Helio_TriggerState emptyState);
};

// Panel Serialization Data
struct HelioPanelData : public HelioObjectData {

    HelioPanelData();
    virtual void toJSONObject(JsonObject &objectOut) const override;
    virtual void fromJSONObject(JsonObjectConst &objectIn) override;
};

#endif // /ifndef HelioPanels_H
