/*  Helioduino: Simple automation controller for solar tracking systems.
    Copyright (C) 2023 NachtRaveVL          <nachtravevl@gmail.com>
    Helioduino Core Logic Helpers
*/

#ifndef HelioCoreLogic_H
#define HelioCoreLogic_H

#include <stdint.h>
#include <stddef.h>
#include <math.h>

inline uint32_t helioElapsedTime(uint32_t now, uint32_t start)
{
    return (uint32_t)(now - start);
}

inline bool helioHasElapsed(uint32_t now, uint32_t start, uint32_t duration)
{
    return helioElapsedTime(now, start) >= duration;
}

inline float helioLargerMagnitude(float current, float candidate)
{
    return fabsf(candidate) > fabsf(current) ? candidate : current;
}

inline int helioDirectionForOffset(float offset, float epsilon = 0.000001f)
{
    return offset > epsilon ? 1 : offset < -epsilon ? -1 : 0;
}

inline float helioTravelDistanceForTime(float speedPerMinute, uint32_t timeMillis, int direction)
{
    if (!direction || fabsf(speedPerMinute) <= 0.000001f || !timeMillis) { return 0.0f; }
    return fabsf(speedPerMinute) * (timeMillis / 60000.0f) * (direction > 0 ? 1.0f : -1.0f);
}

inline uint32_t helioPoweredTravelTime(uint32_t totalTravelTimeMillis, uint32_t coastTimeMillis)
{
    return totalTravelTimeMillis > coastTimeMillis ? totalTravelTimeMillis - coastTimeMillis
                                                    : totalTravelTimeMillis;
}

inline bool helioShouldHoldIncrementalMotor(float offset, int lastDirection,
                                            float alignedRange, float nearbyRange,
                                            float coastDistance)
{
    const float magnitude = fabsf(offset);
    if (magnitude <= fabsf(alignedRange)) { return true; }

    const int targetDirection = helioDirectionForOffset(offset);
    if (!lastDirection || !targetDirection) { return false; }

    const float coast = fabsf(coastDistance);
    if (targetDirection == lastDirection) {
        return magnitude <= coast;
    }

    return magnitude <= fmaxf(fabsf(nearbyRange), coast);
}

inline float helioWrappedAngleDelta(float value, float reference)
{
    float delta = fmodf(value - reference, 360.0f);
    if (delta > 180.0f) { delta -= 360.0f; }
    else if (delta < -180.0f) { delta += 360.0f; }
    return delta;
}

inline float helioUpdateRunningCorrection(float correction, float sample, uint16_t sampleCount)
{
    if (!sampleCount) { return correction; }
    return correction + ((sample - correction) / sampleCount);
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
