#include <gtest/gtest.h>

#include <stdexcept>

#include "ekf_localizer/velocity_gate.hpp"

using ekf_localizer::VelocityGate;

TEST(VelocityGate, FirstSampleIsAlwaysAccepted)
{
    VelocityGate gate(0.5, 0.2, 3.0);
    const auto result = gate.update(2.0, 0.1, 0.05, 0.02, 0.0);

    EXPECT_TRUE(result.passed);
    EXPECT_DOUBLE_EQ(result.velocity, 2.0);
    EXPECT_DOUBLE_EQ(result.yaw_rate, 0.1);
}

TEST(VelocityGate, AcceptsConsistentSubsequentSample)
{
    VelocityGate gate(0.5, 0.2, 3.0);
    gate.update(2.0, 0.1, 0.05, 0.02, 0.0);

    const auto result = gate.update(2.05, 0.11, 0.05, 0.02, 0.02);
    EXPECT_TRUE(result.passed);
}

TEST(VelocityGate, RejectsOutlierSpike)
{
    VelocityGate gate(0.05, 0.02, 3.0);
    gate.update(2.0, 0.1, 0.01, 0.005, 0.0);

    const auto result = gate.update(50.0, 0.1, 0.01, 0.005, 0.02);
    EXPECT_FALSE(result.passed);
    EXPECT_DOUBLE_EQ(result.velocity, 2.0);
}

TEST(VelocityGate, RejectedSampleDoesNotCorruptInternalState)
{
    VelocityGate gate(0.05, 0.02, 3.0);
    gate.update(2.0, 0.1, 0.01, 0.005, 0.0);
    gate.update(50.0, 0.1, 0.01, 0.005, 0.02);

    const auto result = gate.update(2.05, 0.1, 0.01, 0.005, 0.02);
    EXPECT_TRUE(result.passed);
}

TEST(VelocityGate, ThrowsOnNonPositiveVariance)
{
    VelocityGate gate(0.5, 0.2, 3.0);
    EXPECT_THROW(gate.update(2.0, 0.1, 0.0, 0.02, 0.0), std::invalid_argument);
}
