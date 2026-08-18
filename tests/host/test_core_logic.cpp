#include <cassert>
#include <cstdint>

#include "HelioCoreLogic.h"

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
    testBinaryDataReadPlan();
    testBinaryDebounce();
    return 0;
}
