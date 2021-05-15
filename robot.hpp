//
// Created by 杨智宇 on 2021/5/15.
//

#ifndef JAKA_GUI_ROBOT_HPP
#define JAKA_GUI_ROBOT_HPP

#include <jkerr.h>
#include <jktypes.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <mutex>
#include <thread>

#define ERR_NOT_IMPL 404
#define ERR_PREV_NOT_COMPLETE 500
#define ERR_NOT_LOGIN 100

struct stEntireRobotStatus {
    bool is_login;
    RobotStatus status;
};

class AbstractRobot {
   protected:
    std::atomic_bool m_isLogin;
    std::atomic<double> m_radPerSecond;

   public:
    explicit AbstractRobot() : m_isLogin(false), m_radPerSecond(1 / 180.0 * M_PI){};

    bool is_login() { return m_isLogin.load(); };

    void set_spin_speed(double degereePerSpeed) { m_radPerSecond.exchange(degereePerSpeed / 180.0 * M_PI); };

    virtual void get_robot_status(RobotStatus *status){};

    virtual errno_t login_in(const char *ip) { return ERR_NOT_IMPL; };

    virtual errno_t login_out() { return ERR_NOT_IMPL; };

    virtual errno_t joint_move(const JointValue *joint_pos, MoveMode move_mode) { return ERR_NOT_IMPL; };

    virtual errno_t motion_abort() { return ERR_NOT_IMPL; };
};

class VirtualRobot : public AbstractRobot {
   private:
    double m_tagetJointVal[6];
    double m_startJointVal[6];
    std::chrono::high_resolution_clock::time_point m_startTime;
    std::chrono::milliseconds m_duration;
    std::atomic_bool m_isInPos;
    RobotStatus m_status;

    std::mutex m_mutex;

   public:
    VirtualRobot()
        : m_tagetJointVal(),
          m_startJointVal(),
          m_startTime(std::chrono::high_resolution_clock::now()),
          m_isInPos(true),
          m_status({}){};

    void get_robot_status(RobotStatus *status) override {
        if (!m_isInPos.load()) {
            auto delta = std::chrono::high_resolution_clock::now() - m_startTime;
            if (delta < m_duration) {
                auto t        = std::chrono::duration_cast<std::chrono::milliseconds>(delta).count();
                auto duration = std::chrono::milliseconds::zero().count();
                {
                    std::lock_guard<std::mutex> lock(m_mutex);
                    duration = m_duration.count();
                }
                double radio = (double)t / (double)duration;
                for (int i = 0; i < 6; ++i) {
                    status->joint_position[i] = (m_tagetJointVal[i] - m_startJointVal[i]) * radio + m_startJointVal[i];
                }
            } else {
                memcpy(status->joint_position, m_tagetJointVal, sizeof(m_tagetJointVal));
                memcpy(m_status.joint_position, m_tagetJointVal, sizeof(m_tagetJointVal));
                m_isInPos.exchange(true);
            }
        } else {
            memcpy(status, &m_status, sizeof(m_status));
        }
    };

    errno_t login_in(const char *ip) override {
        m_status.powered_on = true;
        m_status.enabled    = true;
        m_isLogin.exchange(true);
        return ERR_SUCC;
    };

    errno_t login_out() override {
        m_isLogin.exchange(false);
        return ERR_SUCC;
    };

    errno_t joint_move(const JointValue *joint_pos, MoveMode move_mode) override {
        if (!is_login()) return ERR_NOT_LOGIN;
        if (!m_isInPos.load()) return ERR_PREV_NOT_COMPLETE;

        double maxDeltaAngle = 0;
        if (move_mode == INCR) {
            for (int i = 0; i < 6; ++i) {
                m_tagetJointVal[i] = m_status.joint_position[i] + joint_pos->jVal[i];
                maxDeltaAngle      = std::max(maxDeltaAngle, std::fabs(joint_pos->jVal[i]));
            }
        } else if (move_mode == ABS) {
            for (int i = 0; i < 6; ++i) {
                m_tagetJointVal[i] = joint_pos->jVal[i];
                maxDeltaAngle = std::max(maxDeltaAngle, std::fabs(m_status.joint_position[i] - m_tagetJointVal[i]));
            }
        }
        memcpy(m_startJointVal, m_status.joint_position, sizeof(m_startJointVal));

        m_duration  = std::chrono::milliseconds(int(maxDeltaAngle * 1000.0 / m_radPerSecond));
        m_startTime = std::chrono::high_resolution_clock::now();
        m_isInPos.exchange(false);
        std::this_thread::sleep_for(m_duration);
        return ERR_SUCC;
    };

    errno_t motion_abort() override {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_duration = std::chrono::milliseconds::zero();
        return ERR_SUCC;
    };
};

#endif  // JAKA_GUI_ROBOT_HPP
