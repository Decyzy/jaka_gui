#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QDebug>
#include <QMainWindow>
#include <QMessageBox>
#include <QTimer>

#include "robot_manager.hpp"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

   private slots:
    void on_logInBt_clicked();

    void on_isFakecheckBox_stateChanged(int arg1);

    void on_powerOnBt_clicked();

    void on_enableBt_clicked();

    void on_addBtJ0_clicked();

    void on_addBtJ1_clicked();

    void on_addBtJ2_clicked();

    void on_addBtJ3_clicked();

    void on_addBtJ4_clicked();

    void on_addBtJ5_clicked();

    void on_subBtJ0_clicked();

    void on_subBtJ1_clicked();

    void on_subBtJ2_clicked();

    void on_subBtJ3_clicked();

    void on_subBtJ4_clicked();

    void on_subBtJ5_clicked();

   private:
    Ui::MainWindow *ui;
    RobotManager robotManager;
    QTimer timer;

   public slots:
    void onUpdatedStatus();

    void onBusy();
};

#endif // MAINWINDOW_H
