#ifndef Servo_H
#define Servo_H

#pragma once

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif

#include <rviz/panel.h>

#include <std_msgs/Int32MultiArray.h>

#include <QMainWindow>
#include <QGraphicsScene>
#include <QGraphicsPathItem>

#include <rviz/message_filter_display.h>
#include <rviz/ogre_helpers/arrow.h>
#include <geometry_msgs/PointStamped.h>

namespace Ui {
class Servo;
}

namespace servo
{
class Servo: public rviz::Panel
{
    Q_OBJECT

public:
    //explicit servo(QWidget *parent = 0);
    Servo(QWidget *parent = 0);
    ~Servo() override; 

    int data = 1;

    void onInitialize() override;
    void send();
    void onEnable();
    void onDisable();
    void arrayback(const std_msgs::Int32MultiArray& msg);

private Q_SLOTS:
  void dialValueChanged(int value);
  void lineEditChanged();
  void engage();
  void idle();
  void start();
  void stop();
  void send_msg();

protected:
  Ui::Servo *ui;
  QGraphicsScene *scene;
  QGraphicsPixmapItem *pix;
  QIcon icon;
  QIcon icon_on;
  QIcon icon_off;
  QIcon icon_up;
  QIcon icon_dn;

  std::string project_path;
  QString icon_on_path;
  QString icon_off_path;

  int sendtime = 100; //ms
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  ros::Timer ti_; 
  ros::NodeHandle n;
};
}
#endif // Servo_H
