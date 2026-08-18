#include <cassert>
#include <cmath>
#include <cstdint>
#include <cfloat>

#include "HelioCoreLogic.h"

static bool nearlyEqual(float lhs, float rhs, float epsilon = 0.0001f)
{
    return std::fabs(lhs - rhs) <= epsilon;
}

static void testElapsedTime()
{
    assert(helioElapsedTime(150, 100) == 50);
    assert(!helioHasElapsed(149, 100, 50));
    assert(helioHasElapsed(150, 100, 50));

    const uint32_t start = UINT32_MAX - 24;
    assert(helioElapsedTime(25, start) == 50);
    assert(!helioHasElapsed(24, start, 50));
    assert(helioHasElapsed(25, start, 50));
}

static void testSignedOffsetMagnitude()
{
    assert(nearlyEqual(helioLargerMagnitude(2.0f, -5.0f), -5.0f));
    assert(nearlyEqual(helioLargerMagnitude(-5.0f, 3.0f), -5.0f));
    assert(nearlyEqual(helioLargerMagnitude(-2.0f, 5.0f), 5.0f));

    assert(helioDirectionForOffset(10.0f) == 1);
    assert(helioDirectionForOffset(-10.0f) == -1);
    assert(helioDirectionForOffset(0.0f) == 0);

    // Attachment direction selection uses FLT_EPSILON to match actuator stop semantics.
    assert(helioDirectionForOffset(FLT_EPSILON, FLT_EPSILON) == 0);
    assert(helioDirectionForOffset(-FLT_EPSILON, FLT_EPSILON) == 0);
    assert(helioDirectionForOffset(FLT_EPSILON * 2.0f, FLT_EPSILON) == 1);
    assert(helioDirectionForOffset(-FLT_EPSILON * 2.0f, FLT_EPSILON) == -1);
}

static void testMotorCoastMath()
{
    assert(nearlyEqual(helioTravelDistanceForTime(2.0f, 3000, 1), 0.1f));
    assert(nearlyEqual(helioTravelDistanceForTime(2.0f, 3000, -1), -0.1f));
    assert(nearlyEqual(helioTravelDistanceForTime(2.0f, 0, 1), 0.0f));

    assert(helioPoweredTravelTime(10000, 2000) == 8000);
    // Short direct moves retain their requested powered time because a full-speed coast
    // model is not meaningful before the motor has had time to get moving.
    assert(helioPoweredTravelTime(1000, 2000) == 1000);
}

static void testIncrementalMotorCoastHold()
{
    // Stop early while approaching so the expected coast can finish the movement.
    assert(helioShouldHoldIncrementalMotor(0.08f, 1, 0.05f, 0.5f, 0.1f));

    // Small overshoot after forward motion should wait for the sun to catch up.
    assert(helioShouldHoldIncrementalMotor(-0.20f, 1, 0.05f, 0.5f, 0.1f));

    // Overshoot outside the nearby/coast band must reverse and correct.
    assert(!helioShouldHoldIncrementalMotor(-0.75f, 1, 0.05f, 0.5f, 0.1f));

    // A large morning reposition must never be mistaken for harmless overshoot.
    assert(!helioShouldHoldIncrementalMotor(-160.0f, 1, 0.05f, 0.5f, 0.1f));

    // Normal forward travel outside the coast range continues driving.
    assert(!helioShouldHoldIncrementalMotor(1.0f, 1, 0.05f, 0.5f, 0.1f));

    // With no known previous direction, ordinary error is still corrected.
    assert(!helioShouldHoldIncrementalMotor(0.2f, 0, 0.05f, 0.5f, 0.1f));
}

static void testNetworkCorrection()
{
    assert(nearlyEqual(helioWrappedAngleDelta(1.0f, 359.0f), 2.0f));
    assert(nearlyEqual(helioWrappedAngleDelta(359.0f, 1.0f), -2.0f));

    float correction = 0.0f;
    correction = helioUpdateRunningCorrection(correction, 2.0f, 1);
    correction = helioUpdateRunningCorrection(correction, 4.0f, 2);
    correction = helioUpdateRunningCorrection(correction, 3.0f, 3);
    assert(nearlyEqual(correction, 3.0f));
}

static void testBinaryDebounce()
{
    bool pendingState = false;
    bool hasPendingState = false;
    uint32_t pendingStart = 0;
    bool accepted = false;

    accepted = helioUpdateStableBinaryState(accepted, true, 10, 100, pendingState, hasPendingState, pendingStart);
    assert(!accepted && hasPendingState && pendingState && pendingStart == 10);

    accepted = helioUpdateStableBinaryState(accepted, true, 109, 100, pendingState, hasPendingState, pendingStart);
    assert(!accepted && hasPendingState);

    // Input bounce restarts the stability requirement.
    accepted = helioUpdateStableBinaryState(accepted, false, 110, 100, pendingState, hasPendingState, pendingStart);
    assert(!accepted && !hasPendingState);
    accepted = helioUpdateStableBinaryState(accepted, true, 120, 100, pendingState, hasPendingState, pendingStart);
    assert(!accepted && hasPendingState && pendingStart == 120);
    accepted = helioUpdateStableBinaryState(accepted, true, 220, 100, pendingState, hasPendingState, pendingStart);
    assert(accepted && !hasPendingState);

    // Zero stability time preserves immediate-response behavior when explicitly requested.
    accepted = helioUpdateStableBinaryState(accepted, false, 221, 0, pendingState, hasPendingState, pendingStart);
    assert(!accepted && !hasPendingState);

    // The elapsed check is safe across millis() rollover.
    pendingState = true;
    hasPendingState = true;
    pendingStart = UINT32_MAX - 50;
    accepted = helioUpdateStableBinaryState(false, true, 60, 100, pendingState, hasPendingState, pendingStart);
    assert(accepted && !hasPendingState);
}

static void testBinaryDataReadPlan()
{
    auto same = helioBinaryDataReadPlan(100, 100, 20);
    assert(same.copyBytes == 80 && same.skipBytes == 0);

    // Older append-only records copy what exists and leave new fields at constructor defaults.
    auto older = helioBinaryDataReadPlan(80, 100, 20);
    assert(older.copyBytes == 60 && older.skipBytes == 0);

    // Newer append-only records copy the known prefix and skip unknown trailing fields.
    auto newer = helioBinaryDataReadPlan(120, 100, 20);
    assert(newer.copyBytes == 80 && newer.skipBytes == 20);

    auto invalidSerialized = helioBinaryDataReadPlan(10, 100, 20);
    assert(invalidSerialized.copyBytes == 0 && invalidSerialized.skipBytes == 0);

    auto invalidCurrent = helioBinaryDataReadPlan(100, 10, 20);
    assert(invalidCurrent.copyBytes == 0 && invalidCurrent.skipBytes == 0);
}

int main()
{
    testElapsedTime();
    testSignedOffsetMagnitude();
    testMotorCoastMath();
    testIncrementalMotorCoastHold();
    testNetworkCorrection();
    testBinaryDataReadPlan();
    testBinaryDebounce();
    return 0;
}
