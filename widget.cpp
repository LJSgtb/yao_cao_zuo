#include "widget.h"
#include "ui_widget.h"

#include <QThread>
#include <QDebug>
#define DHD_DEVICE_SIGMA7_RIGHT   104

bool isok;

Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
{
    ui->setupUi(this);

    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, [=](){
        ui->lcdNumber->display(f[0]);
        ui->lcdNumber_2->display(f[1]);
        ui->lcdNumber_3->display(f[2]);
        ui->lcdNumber_4->display(f[3]);
        ui->lcdNumber_5->display(f[4]);
        ui->lcdNumber_6->display(f[5]);
    });
}

Widget::~Widget()
{
    delete ui;
}

void Widget::on_pushButton_clicked()
{
    srv_net_st * pinfo1 = new srv_net_st();
    memset(pinfo1->SrvIp, 0x00, sizeof(pinfo1->SrvIp));
    memcpy(pinfo1->SrvIp, "192.168.10.75", strlen("192.168.10.75"));
    pinfo1->LocHeartbeatPort = 0;
    pinfo1->LocRobotStatePort = 0;
    pinfo1->LocSrvPort = 0;
    int ret = initSrv(nullptr, nullptr, pinfo1);
    if (ret < 0)
    {
        printf("192.168.10.75 initSrv failed! Return value = %d\n ", ret);
    }
    if (pinfo1)
    {
        delete pinfo1;
        pinfo1 = nullptr;
    }
    releaseBrake();

}

void Widget::on_pushButton_2_clicked()
{
//    // Optimize console output performance.
//     std::cout.rdbuf()->pubsetbuf(nullptr, 512);
//     std::cout << std::nounitbuf;
//     std::ios_base::sync_with_stdio(false);

     // Display version information.
     std::cout << "Automatic Centering Example " << dhdGetSDKVersionStr() << std::endl;
     std::cout << "Copyright (C) 2001-2023 Force Dimension" << std::endl;
     std::cout << "All Rights Reserved." << std::endl << std::endl;

     // Open the first available haptic device.
     if (drdOpen() < 0)
     {
         std::cout << "error: failed to open device (" << dhdErrorGetLastStr() << ")" << std::endl;
         dhdSleep(2.0);
         return;
     }

     // Make sure the device is compatible with the robotics library (DRD).
     if (!drdIsSupported())
     {
         std::cout << "unsupported device type " << dhdGetSystemName() << std::endl;
         std::cout << "exiting..." << std::endl;
         dhdSleep(2.0);
         drdClose();
         return;
     }

     // Display the device type.
     std::cout << dhdGetSystemName() << " device detected" << std::endl << std::endl;

     // Perform automatic initialization of the device if required.
     if (!drdIsInitialized() && drdAutoInit() < 0)
     {
         std::cout << "error: failed to initialize device (" << dhdErrorGetLastStr() << ")" << std::endl;
         dhdSleep(2.0);
         return;
     }
     else if (drdStart() < 0)
     {
         std::cout << "error: failed to start robotic regulation (" << dhdErrorGetLastStr() << ")" << std::endl;
         dhdSleep(2.0);
         return;
     }

     //// Move to the center of the workspace.
     //double positionCenter[DHD_MAX_DOF] = {};
     //if (drdMoveTo(positionCenter) < 0)
     //{
     //	std::cout << "error: failed to move the device (" << dhdErrorGetLastStr() << ")" << std::endl;
     //	dhdSleep(2.0);
     //	return -1;
     //}

     // Stop the regulation thread but leaves the forces enabled on the device.
     if (drdStop(true) < 0)
     {
         std::cout << "error: failed to stop robotic regulation (" << dhdErrorGetLastStr() << ")" << std::endl;
         dhdSleep(2.0);
         return;
     }


     // Allocate and initialize haptic loop variables.
     double px = 0.0;
     double py = 0.0;
     double pz = 0.0;
     double rx = 0.0;
     double ry = 0.0;
     double rz = 0.0;
     double gd = 0.0;
     double lastDisplayUpdateTime = dhdGetTime();

     double pose[6] = { 0.0 };
     double dst_pose[6] = { 0.0 };
     getTcpPos(pose);
     getTcpPos(dst_pose);

     double px1 = 0.0;
     double py1 = 0.0;
     double pz1 = 0.0;
     double rx1 = 0.0;
     double ry1 = 0.0;
     double rz1 = 0.0;

     bool open = true;


     // Run the haptic loop.
     isok = true;
     while (isok)
     {
         // Apply zero force on the haptic device.
         if (dhdSetForceAndTorqueAndGripperForce(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0) < DHD_NO_ERROR)
         {
             std::cout << "error: failed to render force (" << dhdErrorGetLastStr() << ")" << std::endl;
             dhdSleep(2.0);
             break;
         }

         // Retrieve the haptic device position.
         if (dhdGetPositionAndOrientationRad(&px, &py, &pz, &rx, &ry, &rz) < DHD_NO_ERROR)
         {
             std::cout << std::endl << "error: failed to read position (" << dhdErrorGetLastStr() << ")" << std::endl;
             dhdSleep(2.0);
             break;
         }

         // Retrieve the haptic device force.
         if (dhdGetGripperGap(&gd) < DHD_NO_ERROR)
         {
             std::cout << std::endl << "error: failed to GetGripperGap (" << dhdErrorGetLastStr() << ")" << std::endl;
             dhdSleep(2.0);
             break;
         }


         if (gd > 0.02 && open)
         {
             qDebug() << "aaaaa";
             getTcpPos(pose);
             px1 = px; py1 = py; pz1 = pz;
             rx1 = rx; ry1 = ry; rz1 = rz;
             open = false;
         }
         if(gd < 0.001 && !open)
         {
             qDebug() << "bbbb";
             open = true;
         }
         if (open)
         {
             servoL_ex(dst_pose, 0.003, 0.08, 2000, 2, false);
             QThread::msleep(3);
         }
         else
         {
             dst_pose[1] = pose[1] + (px - px1);
             dst_pose[0] = pose[0] - (py - py1);
             dst_pose[2] = pose[2] + (pz - pz1);
             dst_pose[3] = pose[3] + (rx - rx1) / 2;
             dst_pose[4] = pose[4] + (ry - ry1) / 2;
             dst_pose[5] = pose[5] + (rz - rz1) / 2;
             servoL_ex(dst_pose, 0.003, 0.08, 2000, 2, false);
             QThread::msleep(3);
         }


//         // Periodically display the haptic device status and flush stdout.
//         double time = dhdGetTime();
//         if (time - lastDisplayUpdateTime > 0.1)
//         {
//             lastDisplayUpdateTime = time;

//             std::cout << std::showpos << std::fixed << std::setprecision(3);
//             std::cout << "p (" << std::setw(6) << px << " " << std::setw(6) << py << " " << std::setw(6) << pz << ") m  |  ";
//             std::cout << "f (" << std::setw(6) << rx << " " << std::setw(6) << ry << " " << std::setw(6) << rz << ") rad  |  ";
//             std::cout << "freq " << std::noshowpos << std::setprecision(2) << std::setw(4) << dhdGetComFreq() << " kHz  \r";
//             std::cout.flush();
//         }

//         // Process user input.
//         if (dhdKbHit())
//         {
//             switch (dhdKbGet())
//             {
//             // Quit the application.
//             case 'q':
//             {
//                 std::cout << std::endl << std::endl << "exiting at user's request" << std::endl;
//                 running = false;
//                 break;
//             }
//             }
//         }
         QApplication::processEvents();
     }

     // Close the connection to the haptic device.
     if (drdClose() < 0)
     {
         std::cout << "error: failed to close the connection (" << dhdErrorGetLastStr() << ")" << std::endl;
         dhdSleep(2.0);
     }

     // Report success.
     std::cout << "connection closed" << std::endl;
}

void Widget::on_pushButton_3_clicked()
{
    isok = false;
}

void Widget::on_pushButton_4_clicked()
{
    int value = 0;
    int ret = writeDO ("board", "do1",value);
    if(ret < 0)
    {
     qDebug("WriteDO failed!\n");
    }
}

void Widget::on_pushButton_5_clicked()
{
    int value = 1;
    int ret = writeDO ("board", "do1",value);
    if(ret < 0)
    {
     qDebug("WriteDO failed!\n");
    }
}
