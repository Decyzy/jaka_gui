#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <JAKAZuRobot.h>
#include <jkerr.h>
#include <jktypes.h>

#include <QObject>
#include <atomic>
#include <functional>
#include <iostream>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#define MACOS_TEST

struct stRobotEntireStatus {
    bool isLogin       = false;
    RobotStatus status = {0};
};

class AbstractRobot {
   protected:
    std::mutex m_mutex;
    double m_radPerSecond = 5 / 180.0 * M_PI;  // rad/s

   public:
    /**
     * @brief set_revolving_speed
     * @param speed: degree/s
     */
    void set_revolving_speed(double degPerSecond) {
        assert(degPerSecond <= 0);
        m_radPerSecond = degPerSecond / 180.0 * M_PI;
    }

    /**
     * @brief 创建机器人控制句柄
     * @param ip  控制器ip地址
     * @return ERR_SUCC 成功 其他失败
     */
    virtual errno_t login_in(const char* ip) {
        throw "not implement";
        return ERR_SUCC;
    }

    /**
     * @brief 断开控制器连接
     * @return ERR_SUCC 成功 其他失败
     */
    virtual errno_t login_out() {
        throw "not implement";
        return ERR_SUCC;
    }

    /**
     * @param handle  机器人控制句柄
     * @return ERR_SUCC 成功 其他失败
     */
    virtual errno_t power_on() {
        throw "not implement";
        return ERR_SUCC;
    }

    /**
     * @brief 关闭机器人电源
     * @return ERR_SUCC 成功 其他失败
     */
    virtual errno_t power_off() {
        throw "not implement";
        return ERR_SUCC;
    }

    /**
     * @brief 控制机器人上使能
     * @return ERR_SUCC 成功 其他失败
     */
    virtual errno_t enable_robot() {
        throw "not implement";
        return ERR_SUCC;
    }

    /**
     * @brief 控制机器人下使能
     * @return ERR_SUCC 成功 其他失败
     */
    virtual errno_t disable_robot() {
        throw "not implement";
        return ERR_SUCC;
    }

    /**
     * @brief 机器人关节运动
     * @param joint_pos 机器人关节运动目标位置
     * @move_mode 指定运动模式：增量运动(相对运动)或绝对运动
     * @param is_block 设置接口是否为阻塞接口，TRUE为阻塞接口 FALSE为非阻塞接口
     * @param speed 机器人关节运动速度，单位：rad/s
     * @return ERR_SUCC 成功 其他失败
     */
    virtual errno_t joint_move(const JointValue* joint_pos, MoveMode move_mode) {
        throw "not implement";
        return ERR_SUCC;
    }

    /**
     * @brief 获取机器人状态数据
     */
    virtual stRobotEntireStatus get_robot_status() {
        throw "not implement";
        return stRobotEntireStatus();
    };
};

class RealRobot : public AbstractRobot {
   private:
#ifndef MACOS_TEST
    JAKAZuRobot m_robot;
#else
    AbstractRobot m_robot;
#endif

    bool m_isLogin = false;

   public:
    RealRobot() {}

    errno_t login_in(const char* ip) override {
        errno_t ans = m_robot.login_in(ip);
        if (ans == ERR_SUCC) m_isLogin = true;
        return ans;
    }

    errno_t login_out() override {
        errno_t ans = m_robot.login_out();
        if (ans == ERR_SUCC) m_isLogin = false;
        return ans;
    }

    errno_t power_on() override { return m_robot.power_on(); }

    errno_t power_off() override { return m_robot.power_off(); }

    errno_t enable_robot() override { return m_robot.enable_robot(); }

    errno_t disable_robot() override { return m_robot.disable_robot(); }

    errno_t joint_move(const JointValue* joint_pos, MoveMode move_mode) override {
#ifndef MACOS_TEST
        return m_robot.joint_move(joint_pos, move_mode, true, m_radPerSecond);
#else
        return m_robot.joint_move(joint_pos, move_mode);
#endif
    }

    stRobotEntireStatus get_robot_status() override {
        stRobotEntireStatus s;
        s.isLogin = m_isLogin;
#ifndef MACOS_TEST
        m_robot.get_robot_status(&s.status);
#endif
        return s;
    }
};

class VirtualRobot : public AbstractRobot {
   private:
    double m_stepSecond = 0.1;
    stRobotEntireStatus m_status;

    static void sleepMilliseconds(long long milliseconds = 1000) {
        std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
    }

    //    static bool m_checkAllReach(JointValue& cur, JointValue& target) {
    //        for (int i = 0; i < 6; ++i) {
    //            if (fabs(cur.jVal[i] - target.jVal[i]) < 1e-3) return false;
    //        }
    //        return true;
    //    }

   public:
    virtual errno_t login_in(const char* ip) override {
        sleepMilliseconds();
        m_status.isLogin = true;
        return ERR_SUCC;
    }

    virtual errno_t login_out() override {
        sleepMilliseconds();
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_status.isLogin = false;
        }
        return ERR_SUCC;
    }

    virtual errno_t power_on() override {
        sleepMilliseconds();
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_status.status.powered_on = true;
        }
        return ERR_SUCC;
    }

    virtual errno_t power_off() override {
        sleepMilliseconds();
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_status.status.powered_on = false;
        }
        return ERR_SUCC;
    }

    virtual errno_t enable_robot() override {
        sleepMilliseconds();
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_status.status.enabled = true;
        }
        return ERR_SUCC;
    }

    virtual errno_t disable_robot() override {
        sleepMilliseconds();
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_status.status.enabled = false;
        }
        return ERR_SUCC;
    }

    virtual errno_t joint_move(const JointValue* joint_pos, MoveMode move_mode) override {
        JointValue targetJointValue;
        double step[6];
        stRobotEntireStatus s;
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            s = m_status;
        }
        if (move_mode == INCR) {
            for (int i = 0; i < 6; ++i) {
                targetJointValue.jVal[i] = s.status.joint_position[i] + joint_pos->jVal[i];
                step[i] = joint_pos->jVal[i] > 0 ? m_radPerSecond * m_stepSecond : -m_radPerSecond * m_stepSecond;
            }
        } else if (move_mode == ABS) {
            for (int i = 0; i < 6; ++i) {
                targetJointValue.jVal[i] = joint_pos->jVal[i];
                step[i] = joint_pos->jVal[i] > s.status.joint_position[i] ? m_radPerSecond * m_stepSecond
                                                                          : -m_radPerSecond * m_stepSecond;
            }
        }

        while (true) {
            int reachCount = 0;
            std::lock_guard<std::mutex> lock(m_mutex);
            for (int i = 0; i < 6; ++i) {
                if ((step[i] > 0 && m_status.status.joint_position[i] + step[i] > targetJointValue.jVal[i]) ||
                    (step[i] < 0 && m_status.status.joint_position[i] + step[i] < targetJointValue.jVal[i])) {
                    m_status.status.joint_position[i] += step[i];
                } else {
                    m_status.status.joint_position[i] = targetJointValue.jVal[i];
                    reachCount += 1;
                }
            }
            if (reachCount == 6) break;
            sleepMilliseconds(m_stepSecond * 1000.0);
        }
        return ERR_SUCC;
    }

    stRobotEntireStatus get_robot_status() override { return m_status; }
};

#endif  // ROBOT_HPP
