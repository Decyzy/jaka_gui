//
// Created by 杨智宇 on 2021/5/15.
//

#ifndef JAKA_GUI_ROBOT_MANAGER_HPP
#define JAKA_GUI_ROBOT_MANAGER_HPP

#include <QObject>
#include <QDebug>
#include <functional>

#include "robot.hpp"

class RobotManager : public QObject {
    Q_OBJECT
   private:
    AbstractRobot *m_pRobot = nullptr;
    VirtualRobot virtualRobot;
    std::thread m_listenThread;
    std::thread m_execThread;
    std::atomic_bool m_willTerminate;
    std::mutex m_mutex;

    std::function<errno_t(void)> m_asyncTask;
    std::atomic_bool m_isBusy;

    stEntireRobotStatus m_status;

    void m_init() {
        qDebug() << "start init";
        // 暂停之前的
        m_willTerminate.exchange(true);
        if (m_listenThread.joinable()) m_listenThread.join();
        if (m_execThread.joinable()) m_execThread.join();
        // 初始化
        m_status.is_login = false;
        m_willTerminate.exchange(false);
        m_listenThread = std::thread([&]() {
            RobotStatus s;
            while (!m_willTerminate.load()) {
                if (m_pRobot->is_login()) {
                    m_pRobot->get_robot_status(&s);

                    std::lock_guard<std::mutex> lock(m_mutex);
                    memcpy(&m_status.status, &s, sizeof(s));
                    m_status.is_login = true;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(30));
            }
            qDebug() << "listen thread quit";
        });

        m_execThread = std::thread([&]() {
            while (!m_willTerminate.load()) {
                if (m_isBusy.load() && m_asyncTask != nullptr) {
                    qDebug() << "exec";
                    errno_t code = m_asyncTask();
                    m_asyncTask  = nullptr;
                    m_isBusy.exchange(false);
                    qDebug() << "exec complete";
                    //                    emit updateStatusSignal(code);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            qDebug() << "exec thread quit";
        });
        qDebug() << "init";
    }

    void m_addAsyncTask(std::function<errno_t(void)> &&func) {
        if (m_isBusy.load())
            emit busySignal();
        else {
            m_asyncTask = func;
            m_isBusy.exchange(true);
        }
    }

   signals:
    void busySignal();
    void updateStatusSignal(int);

   public:
    explicit RobotManager() : m_willTerminate(false), m_status({false, {}}), m_isBusy(false) {
        useVirtualRobot();
        m_init();
    }

    ~RobotManager() {
        m_willTerminate.exchange(true);
        if (m_listenThread.joinable()) m_listenThread.join();
        if (m_execThread.joinable()) m_execThread.join();
    }

    void get_entire_robot_status(stEntireRobotStatus *s) {
        std::lock_guard<std::mutex> lock(m_mutex);
        memcpy(s, &m_status, sizeof(m_status));
    }

    bool is_login() {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_status.is_login;
    }

    void login_in(const char *ip) {
        m_addAsyncTask([&]() { return m_pRobot->login_in(ip); });
    }

    void login_out() {
        m_addAsyncTask([&]() { return m_pRobot->login_out(); });
    }

    void joint_move(const JointValue *joint_pos, MoveMode move_mode) {
        JointValue jVal;
        memcpy(jVal.jVal, joint_pos, sizeof(double) * 6);
        m_addAsyncTask([&, jVal]() { return m_pRobot->joint_move(&jVal, move_mode); });
    };

    void useVirtualRobot() { m_pRobot = &virtualRobot; };

    void useRealRobot(){};

    void set_spin_speed(double degereePerSpeed) { m_pRobot->set_spin_speed(degereePerSpeed); }
};

#endif //JAKA_GUI_ROBOT_MANAGER_HPP
