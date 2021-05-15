#include "mainwindow.h"

#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow) {
    ui->setupUi(this);
    connect(&timer, &QTimer::timeout, this, &MainWindow::onUpdatedStatus);
    connect(&robotManager, &RobotManager::busySignal, this, &MainWindow::onBusy);
    timer.start(100);
}

MainWindow::~MainWindow() {
    delete ui;
}

void MainWindow::on_logInBt_clicked() {
    if (!robotManager.is_login()) {
        ui->logInBt->setText("log in...");
        ui->logInBt->setEnabled(false);
        robotManager.login_in(ui->ipLineEdit->text().toStdString().c_str());
    } else {
        ui->logInBt->setText("log out...");
        ui->logInBt->setEnabled(false);
        robotManager.login_out();
    }
}

void MainWindow::on_powerOnBt_clicked() {
    stEntireRobotStatus s;
    robotManager.get_entire_robot_status(&s);
    if (!s.status.powered_on) {
        ui->powerOnBt->setText("power on...");
        ui->powerOnBt->setEnabled(false);
        //        robotManager.power_on();
    } else {
        ui->powerOnBt->setText("power off...");
        ui->powerOnBt->setEnabled(false);
        //        robotManager.power_off();
    }
}

void MainWindow::on_enableBt_clicked() {
    stEntireRobotStatus s;
    robotManager.get_entire_robot_status(&s);
    if (!s.status.enabled) {
        ui->enableBt->setText("enable...");
        ui->enableBt->setEnabled(false);
        //        robotManager.enable_robot();

    } else {
        ui->enableBt->setText("disable...");
        ui->enableBt->setEnabled(false);
        //        robotManager.disable_robot();
    }
}

void MainWindow::onUpdatedStatus() {
    //    qDebug() << "[slot] on_updated_status";
    stEntireRobotStatus s;
    robotManager.get_entire_robot_status(&s);
    const RobotStatus &rs = s.status;

    ui->logInBt->setText(robotManager.is_login() ? "log out" : "log in");
    ui->powerOnBt->setText(rs.powered_on ? "power off" : "power on");
    ui->enableBt->setText(rs.enabled ? "disable" : "enable");

    ui->logInBt->setEnabled(true);
    if (s.is_login) {
        ui->powerOnBt->setEnabled(true);
    } else {
        ui->powerOnBt->setEnabled(false);
        ui->enableBt->setEnabled(false);
        return;
    }
    if (rs.powered_on) {
        ui->enableBt->setEnabled(true);
    } else {
        ui->enableBt->setEnabled(false);
    }

    ui->errorCode->setText(QString::number(rs.errcode));
    ui->inpos->setText(rs.inpos == 0 ? "No" : "Yes");
    ui->dragStatus->setText(rs.drag_status == 0 ? "No" : "Yes");
    ui->emergencyStop->setText(rs.emergency_stop == 0 ? "No" : "Yes");
    ui->socket->setText(rs.is_socket_connect == 0 ? "Error" : "Ok");
    ui->powerOn->setText(rs.powered_on == 0 ? "No" : "Yes");
    ui->enabled->setText(rs.enabled == 0 ? "No" : "Yes");
    ui->rap->setText(QString::number(rs.rapidrate, 'f', 2));
    ui->protectiveStop->setText(rs.protective_stop == 0 ? "No" : "Yes");
    ui->onSoftLimit->setText(rs.on_soft_limit == 0 ? "No" : "Yes");

    ui->scbMajor->setText(QString::number(rs.robot_monitor_data.scbMajorVersion, 'f', 2));
    ui->scbMinor->setText(QString::number(rs.robot_monitor_data.scbMinorVersion, 'f', 2));

    if (rs.powered_on) {
        ui->scbTemp->setText(QString::number(rs.robot_monitor_data.cabTemperature, 'f', 2));
        ui->avgA->setText(QString::number(rs.robot_monitor_data.robotAverageCurrent, 'f', 2));
        ui->avgP->setText(QString::number(rs.robot_monitor_data.robotAveragePower, 'f', 2));

        ui->x->setText(QString::number(rs.cartesiantran_position[0], 'f', 2));
        ui->y->setText(QString::number(rs.cartesiantran_position[1], 'f', 2));
        ui->z->setText(QString::number(rs.cartesiantran_position[2], 'f', 2));
        ui->rx->setText(QString::number(rs.cartesiantran_position[3], 'f', 2));
        ui->ry->setText(QString::number(rs.cartesiantran_position[4], 'f', 2));
        ui->rz->setText(QString::number(rs.cartesiantran_position[5], 'f', 2));

        ui->jDeg0->setText(QString::number(rs.joint_position[0] / M_PI * 180.0, 'f', 2));
        ui->jDeg1->setText(QString::number(rs.joint_position[1] / M_PI * 180.0, 'f', 2));
        ui->jDeg2->setText(QString::number(rs.joint_position[2] / M_PI * 180.0, 'f', 2));
        ui->jDeg3->setText(QString::number(rs.joint_position[3] / M_PI * 180.0, 'f', 2));
        ui->jDeg4->setText(QString::number(rs.joint_position[4] / M_PI * 180.0, 'f', 2));
        ui->jDeg5->setText(QString::number(rs.joint_position[5] / M_PI * 180.0, 'f', 2));

        ui->jV0->setText(QString::number(rs.robot_monitor_data.jointMonitorData[0].instVoltage, 'f', 2));
        ui->jV1->setText(QString::number(rs.robot_monitor_data.jointMonitorData[1].instVoltage, 'f', 2));
        ui->jV2->setText(QString::number(rs.robot_monitor_data.jointMonitorData[2].instVoltage, 'f', 2));
        ui->jV3->setText(QString::number(rs.robot_monitor_data.jointMonitorData[3].instVoltage, 'f', 2));
        ui->jV4->setText(QString::number(rs.robot_monitor_data.jointMonitorData[4].instVoltage, 'f', 2));
        ui->jV5->setText(QString::number(rs.robot_monitor_data.jointMonitorData[5].instVoltage, 'f', 2));

        ui->jA0->setText(QString::number(rs.robot_monitor_data.jointMonitorData[0].instCurrent, 'f', 2));
        ui->jA1->setText(QString::number(rs.robot_monitor_data.jointMonitorData[1].instCurrent, 'f', 2));
        ui->jA2->setText(QString::number(rs.robot_monitor_data.jointMonitorData[2].instCurrent, 'f', 2));
        ui->jA3->setText(QString::number(rs.robot_monitor_data.jointMonitorData[3].instCurrent, 'f', 2));
        ui->jA4->setText(QString::number(rs.robot_monitor_data.jointMonitorData[4].instCurrent, 'f', 2));
        ui->jA5->setText(QString::number(rs.robot_monitor_data.jointMonitorData[5].instCurrent, 'f', 2));

        ui->jT0->setText(QString::number(rs.robot_monitor_data.jointMonitorData[0].instTemperature, 'f', 2));
        ui->jT1->setText(QString::number(rs.robot_monitor_data.jointMonitorData[1].instTemperature, 'f', 2));
        ui->jT2->setText(QString::number(rs.robot_monitor_data.jointMonitorData[2].instTemperature, 'f', 2));
        ui->jT3->setText(QString::number(rs.robot_monitor_data.jointMonitorData[3].instTemperature, 'f', 2));
        ui->jT4->setText(QString::number(rs.robot_monitor_data.jointMonitorData[4].instTemperature, 'f', 2));
        ui->jT5->setText(QString::number(rs.robot_monitor_data.jointMonitorData[5].instTemperature, 'f', 2));

        ui->jN0->setText(QString::number(rs.torq_sensor_monitor_data.actTorque[0], 'f', 2));
        ui->jN1->setText(QString::number(rs.torq_sensor_monitor_data.actTorque[1], 'f', 2));
        ui->jN2->setText(QString::number(rs.torq_sensor_monitor_data.actTorque[2], 'f', 2));
        ui->jN3->setText(QString::number(rs.torq_sensor_monitor_data.actTorque[3], 'f', 2));
        ui->jN4->setText(QString::number(rs.torq_sensor_monitor_data.actTorque[4], 'f', 2));
        ui->jN5->setText(QString::number(rs.torq_sensor_monitor_data.actTorque[5], 'f', 2));
    } else {
    }
}

void MainWindow::onBusy() {
    QMessageBox messageBox(QMessageBox::Warning, "Warning", "操作过于频繁", QMessageBox::Ok);
    messageBox.exec();
}

void MainWindow::on_isFakecheckBox_stateChanged(int arg1) {
    if (arg1 == Qt::Checked) {
        robotManager.useVirtualRobot();
    } else if (arg1 == Qt::Unchecked) {
        robotManager.useRealRobot();
    }
}

void MainWindow::on_addBtJ0_clicked() {
    robotManager.set_spin_speed(ui->velocity->value());
    JointValue jVal;
    memset(jVal.jVal, 0, sizeof(double) * 6);
    jVal.jVal[0] = ui->step->value() / 180.0 * M_PI;
    robotManager.joint_move(&jVal, INCR);
}

void MainWindow::on_addBtJ1_clicked() {
    robotManager.set_spin_speed(ui->velocity->value());
    JointValue jVal;
    memset(jVal.jVal, 0, sizeof(double) * 6);
    jVal.jVal[1] = ui->step->value() / 180.0 * M_PI;
    robotManager.joint_move(&jVal, INCR);
}

void MainWindow::on_addBtJ2_clicked() {
    robotManager.set_spin_speed(ui->velocity->value());
    JointValue jVal;
    memset(jVal.jVal, 0, sizeof(double) * 6);
    jVal.jVal[2] = ui->step->value() / 180.0 * M_PI;
    robotManager.joint_move(&jVal, INCR);
}

void MainWindow::on_addBtJ3_clicked() {
    robotManager.set_spin_speed(ui->velocity->value());
    JointValue jVal;
    memset(jVal.jVal, 0, sizeof(double) * 6);
    jVal.jVal[3] = ui->step->value() / 180.0 * M_PI;
    robotManager.joint_move(&jVal, INCR);
}

void MainWindow::on_addBtJ4_clicked() {
    robotManager.set_spin_speed(ui->velocity->value());
    JointValue jVal;
    memset(jVal.jVal, 0, sizeof(double) * 6);
    jVal.jVal[4] = ui->step->value() / 180.0 * M_PI;
    robotManager.joint_move(&jVal, INCR);
}

void MainWindow::on_addBtJ5_clicked() {
    robotManager.set_spin_speed(ui->velocity->value());
    JointValue jVal;
    memset(jVal.jVal, 0, sizeof(double) * 6);
    jVal.jVal[5] = ui->step->value() / 180.0 * M_PI;
    robotManager.joint_move(&jVal, INCR);
}

void MainWindow::on_subBtJ0_clicked() {
    robotManager.set_spin_speed(ui->velocity->value());
    JointValue jVal;
    memset(jVal.jVal, 0, sizeof(double) * 6);
    jVal.jVal[0] = -ui->step->value() / 180.0 * M_PI;
    robotManager.joint_move(&jVal, INCR);
}

void MainWindow::on_subBtJ1_clicked() {
    robotManager.set_spin_speed(ui->velocity->value());
    JointValue jVal;
    memset(jVal.jVal, 0, sizeof(double) * 6);
    jVal.jVal[1] = -ui->step->value() / 180.0 * M_PI;
    robotManager.joint_move(&jVal, INCR);
}

void MainWindow::on_subBtJ2_clicked() {
    robotManager.set_spin_speed(ui->velocity->value());
    JointValue jVal;
    memset(jVal.jVal, 0, sizeof(double) * 6);
    jVal.jVal[2] = -ui->step->value() / 180.0 * M_PI;
    robotManager.joint_move(&jVal, INCR);
}

void MainWindow::on_subBtJ3_clicked() {
    robotManager.set_spin_speed(ui->velocity->value());
    JointValue jVal;
    memset(jVal.jVal, 0, sizeof(double) * 6);
    jVal.jVal[3] = -ui->step->value() / 180.0 * M_PI;
    robotManager.joint_move(&jVal, INCR);
}

void MainWindow::on_subBtJ4_clicked() {
    robotManager.set_spin_speed(ui->velocity->value());
    JointValue jVal;
    memset(jVal.jVal, 0, sizeof(double) * 6);
    jVal.jVal[4] = -ui->step->value() / 180.0 * M_PI;
    robotManager.joint_move(&jVal, INCR);
}

void MainWindow::on_subBtJ5_clicked() {
    robotManager.set_spin_speed(ui->velocity->value());
    JointValue jVal;
    memset(jVal.jVal, 0, sizeof(double) * 6);
    jVal.jVal[5] = -ui->step->value() / 180.0 * M_PI;
    robotManager.joint_move(&jVal, INCR);
}
