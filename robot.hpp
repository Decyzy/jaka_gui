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

class AbstractRobot : public QObject {
    Q_OBJECT
   protected:
    bool m_isLogin = false;
    RobotStatus m_robotStatus;
    std::queue<std::function<void(void)>> m_q;
    std::thread m_t;
    std::atomic_bool m_willTerminate;
    std::mutex m_mutex;

   signals:
    void loginSignal(bool);
    void busySignal();
    void statusUpdatedSignal();

   public:
    AbstractRobot() {
        m_robotStatus            = {0};
        m_robotStatus.powered_on = false;
        m_robotStatus.enabled    = false;
        m_willTerminate.exchange(false);

        m_t = std::thread([&]() {
            while (!m_willTerminate.load()) {
                std::function<void(void)> func;
                {
                    std::lock_guard<std::mutex> lock(m_mutex);
                    if (!m_q.empty()) {
                        func = m_q.front();
                        m_q.pop();
                    }
                }
                if (func) {
                    func();
                    emit statusUpdatedSignal();
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            std::cout << "AbstractRobot::m_t quit" << std::endl;
        });
    }

    ~AbstractRobot() {
        m_willTerminate.exchange(true);
        if (m_t.joinable()) m_t.join();
    }

    void add_async_task(std::function<void(void)>&& func) {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_q.empty())
            m_q.push(func);
        else
            emit busySignal();
    }

    const std::pair<bool, RobotStatus> get_robot_status() {
        std::lock_guard<std::mutex> lock(m_mutex);
        return std::make_pair(m_isLogin, m_robotStatus);
    }

    /**
     * @brief 创建机器人控制句柄
     * @param ip  控制器ip地址
     * @return ERR_SUCC 成功 其他失败
     */
    virtual errno_t login_in(const char* ip) { return ERR_SUCC; }

    /**
     * @brief 断开控制器连接
     * @return ERR_SUCC 成功 其他失败
     */
    virtual errno_t login_out() { return ERR_SUCC; }

    /**
     * @param handle  机器人控制句柄
     * @return ERR_SUCC 成功 其他失败
     */
    virtual errno_t power_on() { return ERR_SUCC; }

    /**
     * @brief 关闭机器人电源
     * @return ERR_SUCC 成功 其他失败
     */
    virtual errno_t power_off() { return ERR_SUCC; }

    /**
     * @brief 控制机器人上使能
     * @return ERR_SUCC 成功 其他失败
     */
    virtual errno_t enable_robot() { return ERR_SUCC; }

    /**
     * @brief 控制机器人下使能
     * @return ERR_SUCC 成功 其他失败
     */
    virtual errno_t disable_robot() { return ERR_SUCC; }

    /**
     * @brief 机器人关节运动
     * @param joint_pos 机器人关节运动目标位置
     * @move_mode 指定运动模式：增量运动(相对运动)或绝对运动
     * @param is_block 设置接口是否为阻塞接口，TRUE为阻塞接口 FALSE为非阻塞接口
     * @param speed 机器人关节运动速度，单位：rad/s
     * @return ERR_SUCC 成功 其他失败
     */
    virtual errno_t joint_move(const JointValue* joint_pos, MoveMode move_mode, BOOL is_block, double speed) {
        return ERR_SUCC;
    }

    /**
     * @brief 获取控制器IP
     * @param controller_name 控制器名字
     * @param ip_list
     * 控制器ip列表，控制器名字为具体值时返回该名字所对应的控制器IP地址，控制器名字为空时，返回网段类内的所有控制器IP地址
     * @return ERR_SUCC 成功 其他失败
     */
    virtual errno_t get_controller_ip(char* controller_name, char* ip_list) { return ERR_SUCC; }
};

class VirtualRobot : public AbstractRobot {
   private:
    double moveStepRad = 0.1;

   public:
    errno_t login_in(const char* ip) override {
        add_async_task([&]() { m_isLogin = true; });
        return ERR_SUCC;
    }

    errno_t login_out() override {
        add_async_task([&]() { m_isLogin = false; });
        return ERR_SUCC;
    }

    errno_t power_on() override {
        add_async_task([&]() { m_robotStatus.powered_on = true; });
        return ERR_SUCC;
    }

    errno_t power_off() override {
        add_async_task([&]() { m_robotStatus.powered_on = false; });
        return ERR_SUCC;
    }

    errno_t enable_robot() override {
        add_async_task([&]() { m_robotStatus.enabled = true; });
        return ERR_SUCC;
    }

    errno_t disable_robot() override {
        add_async_task([&]() { m_robotStatus.enabled = false; });
        return ERR_SUCC;
    }

    errno_t joint_move(const JointValue* joint_pos, MoveMode move_mode, BOOL is_block, double speed) override {
        add_async_task([&]() {
            for (int i = 0; i < 6; i++) {
                if (m_robotStatus.joint_position[i] + moveStepRad > joint_pos->jVal[i]) {
                    m_robotStatus.joint_position[i] = joint_pos->jVal[i];
                } else {
                    m_robotStatus.joint_position[i] += moveStepRad;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(int(moveStepRad * 1000.0 / speed)));
            }
        });
        return ERR_SUCC;
    }
};

#endif  // ROBOT_HPP
