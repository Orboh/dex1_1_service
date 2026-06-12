#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/idl/go2/MotorStates_.hpp>
#include "dds/Publisher.h"
#include "dds/Subscription.h"
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/common/thread/thread.hpp>
#include "param.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <limits>
#include <mutex>
#include <thread>
#include <spdlog/spdlog.h>

// ============================================================
//  動作範囲パラメータ
// ============================================================
static constexpr float Q_OPEN        = 5.2f;  // 開き位置 [rad]  (機械的限界5.4より0.2手前)
static constexpr float Q_CLOSE       = 4.4f;  // 閉じ位置 [rad]
static constexpr float APPROACH_TIME = 2.0f;  // 起動時に現在位置からQ_OPENへ移動する時間 [sec]
// ============================================================

std::atomic<float> cmd_q(Q_OPEN);
std::atomic<bool>  running(true);

std::atomic<bool>  measuring(false);
std::chrono::steady_clock::time_point measure_start;
std::atomic<float> elapsed_ms(-1.0f);
std::atomic<int>   stable_count(0);

std::mutex io_mutex;

static void clear_line_tail() { std::cout << "\033[K"; }

// 入力スレッド: o=開く, q=閉じる, x=終了
void input_thread()
{
    while (running) {
        {
            std::lock_guard<std::mutex> lk(io_mutex);
            std::cout << "\033[4;1H";
            std::cout << "command [o=open(" << Q_OPEN << "), q=close(" << Q_CLOSE << "), x=quit]: ";
            clear_line_tail();
            std::cout.flush();
        }

        std::string line;
        if (!std::getline(std::cin, line)) {
            running = false;
            break;
        }
        if (line.empty()) continue;

        char cmd = line[0];
        float target = cmd_q.load();

        if (cmd == 'o' || cmd == 'O') {
            target = Q_OPEN;
        } else if (cmd == 'q' || cmd == 'Q') {
            target = Q_CLOSE;
        } else if (cmd == 'x' || cmd == 'X') {
            running = false;
            break;
        } else {
            std::lock_guard<std::mutex> lk(io_mutex);
            std::cout << "\033[5;1H";
            std::cout << "unknown command. use o / q / x";
            clear_line_tail();
            std::cout.flush();
            continue;
        }

        cmd_q        = target;
        measuring    = true;
        elapsed_ms   = -1.0f;
        stable_count = 0;
        measure_start = std::chrono::steady_clock::now();
    }
}

void print_status(bool use_left, float q_actual, float q_target, float elapsed)
{
    std::lock_guard<std::mutex> lk(io_mutex);
    std::cout << "\033[s";

    char buf[64];
    std::cout << "\033[1;1H";
    std::snprintf(buf, sizeof(buf), "%.3f", q_actual);
    std::cout << (use_left ? "Left " : "Right") << "  actual: " << buf << " rad";
    clear_line_tail();

    std::cout << "\033[2;1H";
    std::snprintf(buf, sizeof(buf), "%.3f", q_target);
    std::cout << "       target: " << buf << " rad";
    clear_line_tail();

    std::cout << "\033[3;1H";
    std::cout << "Time to target (ms): ";
    if (elapsed >= 0.0f)
        std::cout << static_cast<int>(elapsed);
    else
        std::cout << "   ";
    clear_line_tail();

    std::cout << "\033[u";
    std::cout.flush();
}

int main(int argc, char** argv)
{
    auto vm = param::helper_test(argc, argv);
    unitree::robot::ChannelFactory::Instance()->Init(0, vm["network"].as<std::string>());

    bool use_left  = vm.count("left")  > 0;
    bool use_right = vm.count("right") > 0;

    if (use_left == use_right) {
        spdlog::warn("Please specify either --left or --right (but not both).");
        return 1;
    }

    std::shared_ptr<unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorCmds_>>  pub;
    std::shared_ptr<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorStates_>> sub;

    if (use_left) {
        sub = std::make_shared<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorStates_>>("rt/dex1/left/state");
        sub->msg_.states().resize(1);
        sub->wait_for_connection();
        pub = std::make_shared<unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorCmds_>>("rt/dex1/left/cmd");
    } else {
        sub = std::make_shared<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorStates_>>("rt/dex1/right/state");
        sub->msg_.states().resize(1);
        sub->wait_for_connection();
        pub = std::make_shared<unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorCmds_>>("rt/dex1/right/cmd");
    }
    pub->msg_.cmds().resize(1);
    pub->msg_.cmds()[0].mode() = 1;
    pub->msg_.cmds()[0].kp()   = 5.0f;
    pub->msg_.cmds()[0].kd()   = 0.05f;

    float q_init = sub->msg_.states()[0].q();
    spdlog::info("{} gripper init at q = {:.3f}", use_left ? "Left" : "Right", q_init);

    // 起動時: 現在位置 → Q_OPEN へ安全に線形移動
    spdlog::info("Approaching Q_OPEN={:.3f} over {:.1f}s ...", Q_OPEN, APPROACH_TIME);
    auto t_start = std::chrono::steady_clock::now();
    while (true) {
        float elapsed = std::chrono::duration<float>(
            std::chrono::steady_clock::now() - t_start).count();
        float ratio = std::clamp(elapsed / APPROACH_TIME, 0.0f, 1.0f);
        float q = q_init + (Q_OPEN - q_init) * ratio;
        pub->msg_.cmds()[0].q() = q;
        pub->unlockAndPublish();
        if (elapsed >= APPROACH_TIME) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    cmd_q = Q_OPEN;

    // 画面クリアして入力スレッド開始
    std::cout << "\033[2J\033[1;1H" << std::flush;
    std::thread t_input(input_thread);

    // メイン制御ループ (5ms周期 = 200Hz)
    while (running) {
        float target   = cmd_q.load();
        float q_actual = sub->msg_.states()[0].q();

        pub->msg_.cmds()[0].q() = target;
        pub->unlockAndPublish();

        // 到達判定: 目標±0.05 rad 以内
        bool within = std::fabs(q_actual - target) < 0.05f;
        if (measuring.load() && within && stable_count.load() == 0) {
            auto now = std::chrono::steady_clock::now();
            elapsed_ms = static_cast<float>(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - measure_start).count());
            measuring    = false;
            stable_count = 1;
        }
        if (!within) stable_count = 0;

        print_status(use_left, q_actual, target, elapsed_ms.load());
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    if (t_input.joinable()) t_input.join();

    std::cout << "\033[6;1H\nExiting...\n" << std::flush;
    return 0;
}
