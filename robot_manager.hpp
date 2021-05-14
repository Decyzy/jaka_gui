#ifndef ROBOT_MANAGER_HPP
#define ROBOT_MANAGER_HPP

#include <QObject>
#include <atomic>
#include <mutex>
#include <thread>

#include "robot.hpp"

class RobotManager : public QObject {
    Q_OBJECT
   private:
    AbstractRobot *m_pRobot = nullptr;
    VirtualRobot m_virtualRobot;
    RealRobot m_realRobot;

    bool m_isLogin = false;
    RobotStatus m_robotStatus;
    std::queue<std::function<void(void)>> m_q;
    std::thread m_t;
    std::atomic_bool m_willTerminate;
    std::mutex m_mutex;

    void m_addAsyncTask(std::function<void(void)> &&func) {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (m_q.empty())
            m_q.push(func);
        else
            emit busySignal();
    }

   signals:
    void statusUpdatedSignal();
    void busySignal();

   public:
    RobotManager() {
        useVirtualRobot();
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

    ~RobotManager() {
        m_willTerminate.exchange(true);
        if (m_t.joinable()) m_t.join();
        m_pRobot = nullptr;
    }

    void useVirtualRobot() { m_pRobot = &m_virtualRobot; }

    void useRealRobot() { m_pRobot = &m_realRobot; }

    stRobotEntireStatus getStatus() { return m_pRobot->get_robot_status(); }

    virtual errno_t login_in(const char *ip) {
        char tmp[512];
        strcpy(tmp, ip);
        m_addAsyncTask([&, tmp]() { m_pRobot->login_in(tmp); });
        return ERR_SUCC;
    }

    virtual errno_t login_out() {
        m_addAsyncTask([&]() { m_pRobot->login_out(); });
        return ERR_SUCC;
    }

    virtual errno_t power_on() {
        m_addAsyncTask([&]() { m_pRobot->power_on(); });
        return ERR_SUCC;
    }

    virtual errno_t power_off() {
        m_addAsyncTask([&]() { return m_pRobot->power_off(); });
        return ERR_SUCC;
    }

    virtual errno_t enable_robot() {
        m_addAsyncTask([&]() { return m_pRobot->enable_robot(); });
        return ERR_SUCC;
    }

    virtual errno_t disable_robot() {
        m_addAsyncTask([&]() { return m_pRobot->disable_robot(); });
        return ERR_SUCC;
    }
};

#endif // ROBOT_MANAGER_HPP
