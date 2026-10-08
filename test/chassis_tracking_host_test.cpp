#include "ChassisTracking.h"

#include <cassert>
#include <cmath>
#include <iostream>

using namespace chassis_tracking;

static void expectNear(float actual, float expected, float tolerance)
{
    assert(std::fabs(actual - expected) <= tolerance);
}

static void testJerkIntegration()
{
    JerkState state = {0.0F, 2.0F, 3.0F};
    integrateJerk(state, 4.0F, 0.5F);
    expectNear(state.position, 1.4583333F, 1.0e-5F);
    expectNear(state.velocity, 4.0F, 1.0e-5F);
    expectNear(state.acceleration, 5.0F, 1.0e-5F);
}

static void testJerkStepBounds()
{
    JerkState state = {};
    for (int i = 0; i < 3000 && !jerkFinished(state, 1000.0F); ++i)
    {
        jerkStep(state, 1000.0F, 500.0F, 200.0F, 1000.0F, 0.002F);
        assert(state.position <= 1000.0F + 1.0e-4F);
        assert(state.velocity >= -1.0e-4F);
        assert(state.velocity <= 500.0F + 1.0e-4F);
        assert(std::fabs(state.acceleration) <= 200.0F + 1.0e-3F);
    }
    assert(jerkFinished(state, 1000.0F));
}

static void testJerkStepPeriods()
{
    for (float period : {0.001F, 0.005F, 0.02F})
    {
        JerkState state = {};
        for (int i = 0; i < 3000 && !jerkFinished(state, 100.0F); ++i)
            jerkStep(state, 100.0F, 300.0F, 150.0F, 800.0F, period);
        assert(jerkFinished(state, 100.0F));
    }
}

static void testYawGating()
{
    expectNear(yawGatedRemaining(100.0F, 500.0F, 90.0F, 180.0F), 0.0F, 1.0e-5F);
    expectNear(yawGatedRemaining(100.0F, 500.0F, 0.0F, 180.0F), 100.0F, 1.0e-5F);
    expectNear(yawGatedRemaining(100.0F, 500.0F, 180.0F, 180.0F), 0.0F, 1.0e-5F);
    expectNear(yawGatedRemaining(-5.0F, 500.0F, 90.0F, 180.0F), -5.0F, 1.0e-5F);
    expectNear(yawGatedRemaining(100.0F, 500.0F, 90.0F, 0.0F), 100.0F, 1.0e-5F);
}

static void testPathError()
{
    const PathError error = pathError(
        {0.0F, 0.0F},
        {1.0F, 0.0F},
        {100.0F, 0.0F},
        {25.0F, 10.0F});
    expectNear(error.along, 75.0F, 1.0e-5F);
    expectNear(error.cross, 10.0F, 1.0e-5F);
}

int main()
{
    testJerkIntegration();
    testJerkStepBounds();
    testJerkStepPeriods();
    testYawGating();
    testPathError();
    std::cout << "chassis_tracking_host_test: PASS\n";
}
