#ifndef THREADSPD_H
#define THREADSPD_H

//=== Qt ===//
#include <QThread>
#include <QDebug>
//=== Qt(option) ===//
#include <QWaitCondition>
#include <QMutex>
#include <QReadWriteLock>
//=== C++ ===//
#include <iostream>
#include <cstdio>
using namespace std;
//=== SpDyn C++ ===//
#include "spacedyn_ros/spd/rot.h"
#include "spacedyn_ros/spd/spn.h"
#include "spacedyn_ros/matrix/matrix.h"
#include "spacedyn_ros/matrix/vector.h"
//=== Others ===//
#include "chaser_spacedyn/timer.h"
#include "chaser_spacedyn/datasave.h"
#include "chaser_spacedyn/dataext.h"        // Global and define
//#include "glwidget3.h"
//#include "draw/lookat.h"
//#include <QElapsedTimer>

class ThreadSpd : public QThread
{
//    Q_OBJECT

public:
    //===== Constructor and Destructor =====//
 //   explicit ThreadSpd(QObject *parent = 0);
    ~ThreadSpd();
    //======================================//
    void stopThreadSpd();
    void initSpd();

    Timer timer;
    double *inputVal;

    double extime;

signals:
    void signal_time();
    void signal_init();

public slots:

protected:
    //== Starting point for the thread ==//
    void run();     // It has Main loop

private:
    DataSave dataSave;
    QMutex mutexSpd;
    QMutex mutexDq;
    //QElapsedTimer et;

    //== Flag ==//
    volatile bool stopped;

    //=== SpaceDyn ===//
    //- Timer
    double time;
    double d_time;
    double f_time;
    //- Counter
    int data_counter;
    int i,k;
};

#endif // THREADSPD_H
