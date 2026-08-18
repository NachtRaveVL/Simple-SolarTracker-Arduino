/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Core Logic Helpers
*/

#ifndef HelioCoreLogic_H
#define HelioCoreLogic_H

#include <stdint.h>
#include <stddef.h>

inline uint32_t helioElapsedTime(uint32_t now, uint32_t start)
{
    return (uint32_t)(now - start);
}

inline bool helioHasElapsed(uint32_t now, uint32_t start, uint32_t duration)
{
    return helioElapsedTime(now, start) >= duration;
}

inline bool helioUpdateStableBinaryState(bool acceptedState, bool sampledState, uint32_t nowMillis,
                                         uint16_t stableTimeMillis, bool &pendingState,
                                         bool &hasPendingState, uint32_t &pendingStateStart)
{
    if (sampledState == acceptedState) {
        hasPendingState = false;
    } else if (!stableTimeMillis) {
        hasPendingState = false;
        return sampledState;
    } else if (!hasPendingState || pendingState != sampledState) {
        pendingState = sampledState;
        pendingStateStart = nowMillis;
        hasPendingState = true;
    } else if (helioHasElapsed(nowMillis, pendingStateStart, stableTimeMillis)) {
        hasPendingState = false;
        return sampledState;
    }

    return acceptedState;
}

struct HelioBinaryDataReadPlan
{
    size_t copyBytes;
    size_t skipBytes;
};

inline HelioBinaryDataReadPlan helioBinaryDataReadPlan(size_t serializedSize, size_t currentSize, size_t baseSize)
{
    if (serializedSize < baseSize || currentSize < baseSize) { return {0, 0}; }

    const size_t serializedRemaining = serializedSize - baseSize;
    const size_t currentRemaining = currentSize - baseSize;
    const size_t copyBytes = serializedRemaining < currentRemaining ? serializedRemaining : currentRemaining;
    return {copyBytes, serializedRemaining - copyBytes};
}

#endif // /ifndef HelioCoreLogic_H
