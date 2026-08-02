#include <gtest/gtest.h>

#include "ekf_localizer/velocity_gate.hpp"

using ekf_localizer::VelocityGate;

TEST(VelocityGate, AcceptsFirstMeasurementAndInitializes)
{
    VelocityGate gate(0.5, 0.2, 3.0, 0.0, 0.0, 1.0);
    const auto result = gate.update(1.0, 0.1, 0.05, 0.02, 0.0);
    EXPECT_TRUE(result.passed);
    EXPECT_NEAR(result.velocity, 1.0, 1e-9);
    EXPECT_NEAR(result.yaw_rate, 0.1, 1e-9);
}

TEST(VelocityGate, RejectsSuddenOutlierFarFromTrackedVelocity)
{
    VelocityGate gate(0.5, 0.2, 3.0, 0.0, 0.0, 1.0);
    gate.update(1.0, 0.0, 0.05, 0.02, 0.0);
    const auto result = gate.update(20.0, 0.0, 0.05, 0.02, 0.02);
    EXPECT_FALSE(result.passed);
}

TEST(VelocityGate, RecoversAfterSustainedRealVelocityChangeInsteadOfLockingOutForever)
{
    // 実走行でv=1.0->5.0m/sへ急加速したケースを模擬。
    // 最初の1ステップはmahalanobisゲートで棄却されて当然だが、
    // 新しい速度(5.0)が「真値」として複数ステップ後も観測され続けるなら
    // ゲートは時間経過とともに追従して受理できなければならない
    // （internal varianceが棄却時に凍結され再び受理不能になるバグの回帰テスト）。
    // タイムアウト機構(force_accept)より先に自然な分散拡大で受理されることを
    // 検証したいテストなので、ループ上限(500*0.02=10s)より十分大きいタイムアウトを渡す。
    VelocityGate gate(0.5, 0.2, 3.0, 0.0, 0.0, 100.0);
    gate.update(1.0, 0.0, 0.05, 0.02, 0.0);

    bool eventually_passed = false;
    for (int i = 0; i < 500; ++i) {
        const auto result = gate.update(5.0, 0.0, 0.05, 0.02, 0.02);
        if (result.passed) {
            eventually_passed = true;
            break;
        }
    }

    EXPECT_TRUE(eventually_passed)
        << "velocity gate never re-accepted a sustained real velocity change; "
           "internal variance likely frozen on rejection, causing a permanent lockout";
}

TEST(VelocityGate, RejectedUpdateStillGrowsInternalVarianceOverElapsedTime)
{
    // タイムアウト機構(force_accept)より先に自然な分散拡大で受理されることを
    // 検証したいテストなので、ループ上限(2000*0.1=200s)より十分大きいタイムアウトを渡す。
    VelocityGate gate(0.5, 0.2, 3.0, 0.0, 0.0, 1000.0);
    gate.update(1.0, 0.0, 0.05, 0.02, 0.0);

    // 最初の棄却
    const auto first_reject = gate.update(20.0, 0.0, 0.05, 0.02, 0.02);
    ASSERT_FALSE(first_reject.passed);

    // 棄却され続けても、時間経過(dt累積)でゲートの許容範囲が広がっていき、
    // 十分な経過時間の後には同じ大きさの残差でも受理されるようになるべき。
    bool passed_after_time = false;
    for (int i = 0; i < 2000; ++i) {
        const auto result = gate.update(20.0, 0.0, 0.05, 0.02, 0.1);
        if (result.passed) {
            passed_after_time = true;
            break;
        }
    }

    EXPECT_TRUE(passed_after_time)
        << "gate never widened after repeated rejections despite elapsed time";
}

TEST(VelocityGate, StationaryStreakDoesNotCollapseVarianceBelowFloor)
{
    // 長時間の停止(v=0が一致し続けaccept)でvelocity_variance_が際限なく収縮すると、
    // 直後の急発進(正しい観測)がMahalanobisゲートで棄却され続けるロックに陥る
    // （実走行ログで確認された不具合の回帰テスト）。分散フロアがあれば、
    // 1万回の停止観測の後でも急発進をわずかな棄却回数で速やかに受理できるはず。
    // 停止継続中はacceptが続きrejected_elapsed_s_は常に0にリセットされるため、
    // タイムアウト機構(force_accept)は無関係。十分大きい値を渡しておけばよい。
    constexpr double min_velocity_variance = 1.0;
    VelocityGate gate(0.5, 0.2, 3.0, min_velocity_variance, 0.0, 1.0);
    gate.update(0.0, 0.0, 0.01, 0.01, 0.0);
    for (int i = 0; i < 10000; ++i) {
        gate.update(0.0, 0.0, 0.01, 0.01, 0.02);
    }

    int accept_step = -1;
    for (int i = 0; i < 10; ++i) {
        const auto result = gate.update(1.0, 0.0, 0.05, 0.02, 0.02);
        if (result.passed) {
            accept_step = i;
            break;
        }
    }

    EXPECT_NE(accept_step, -1) << "velocity gate did not recover after a long stationary streak";
    EXPECT_LE(accept_step, 1)
        << "variance floor should let the gate recover within a step or two, not hundreds";
}

TEST(VelocityGate, ForcesAcceptanceOnceRejectedElapsedTimeExceedsTimeout)
{
    // 実車ログ再現: 停止(v=0)から急加速(v=5.0)した直後、mahalanobisゲートが
    // 連続棄却し続けるとEKFが静止したまま自己位置を積分し実位置と乖離してしまう。
    // 連続棄却の累積経過時間がmax_reject_duration_sを超えたら強制的に受理し、
    // 危険な長時間ロックを防がなければならない。
    constexpr double max_reject_duration_s = 0.3;
    constexpr double dt = 0.02;
    VelocityGate gate(0.5, 0.2, 3.0, 0.0, 0.0, max_reject_duration_s);
    gate.update(0.0, 0.0, 0.05, 0.02, 0.0);

    double elapsed = 0.0;
    bool force_accepted = false;
    for (int i = 0; i < 30; ++i) {
        const auto result = gate.update(5.0, 0.0, 0.05, 0.02, dt);
        elapsed += dt;
        if (result.passed) {
            force_accepted = true;
            break;
        }
    }

    ASSERT_TRUE(force_accepted);
    EXPECT_LE(elapsed, max_reject_duration_s + dt + 1e-9);
}

TEST(VelocityGate, DoesNotForceAcceptBeforeTimeoutElapsed)
{
    // タイムアウト到達前の単発の大外れ値は、今回の変更後もこれまで通り
    // mahalanobisゲートで棄却されなければならない（外れ値除去能力の回帰テスト）。
    VelocityGate gate(0.5, 0.2, 3.0, 0.0, 0.0, 0.3);
    gate.update(1.0, 0.0, 0.05, 0.02, 0.0);
    const auto result = gate.update(20.0, 0.0, 0.05, 0.02, 0.02);
    EXPECT_FALSE(result.passed);
}
