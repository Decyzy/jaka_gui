#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <JAKAZuRobot.h>
#include <jkerr.h>
#include <jktypes.h>

#include <QDebug>

class AbstractRobot {
   private:
   public:
    AbstractRobot() { JAKAZuRobot robot; }
};

#endif  // ROBOT_HPP
