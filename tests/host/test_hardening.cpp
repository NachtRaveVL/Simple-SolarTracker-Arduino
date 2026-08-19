#include <cassert>
#include <cmath>

#include "HelioCoreLogic.h"

static void testWrappedAngleInvariance()
{
    assert(std::fabs(helioWrappedAngleDelta(721.0f, 359.0f) - 2.0f) < 0.0001f);
    assert(std::fabs(helioWrappedAngleDelta(-359.0f, 1.0f)) < 0.0001f);
    assert(std::fabs(helioWrappedAngleDelta(359.0f, 721.0f) + 2.0f) < 0.0001f);
}

static void testRunningCorrectionZeroCount()
{
    assert(std::fabs(helioUpdateRunningCorrection(3.5f, 100.0f, 0) - 3.5f) < 0.0001f);
}

static void testTravelDirectionOwnsSign()
{
    assert(std::fabs(helioTravelDistanceForTime(-2.0f, 3000, 1) - 0.1f) < 0.0001f);
    assert(std::fabs(helioTravelDistanceForTime(-2.0f, 3000, -1) + 0.1f) < 0.0001f);
}

static void testCoastHoldDirectionSymmetry()
{
    assert(helioShouldHoldIncrementalMotor(-0.08f, -1, 0.05f, 0.5f, 0.1f));
    assert(helioShouldHoldIncrementalMotor(0.20f, -1, 0.05f, 0.5f, 0.1f));
    assert(!helioShouldHoldIncrementalMotor(0.75f, -1, 0.05f, 0.5f, 0.1f));
}

static void testBaseOnlyBinaryRecord()
{
    auto baseOnly = helioBinaryDataReadPlan(20, 100, 20);
    assert(baseOnly.copyBytes == 0);
    assert(baseOnly.skipBytes == 0);
}

int main()
{
    testWrappedAngleInvariance();
    testRunningCorrectionZeroCount();
    testTravelDirectionOwnsSign();
    testCoastHoldDirectionSymmetry();
    testBaseOnlyBinaryRecord();
    return 0;
}
