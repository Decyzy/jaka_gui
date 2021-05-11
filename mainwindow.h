#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QDebug>
#include <QMainWindow>

#include "robot.hpp"

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

   private:
    Ui::MainWindow *ui;
    AbstractRobot *p_rb = nullptr;
    VirtualRobot virtualRobot;
    AbstractRobot abstractRobot;

   public slots:
    void onUpdatedStatus();
};
#endif // MAINWINDOW_H
