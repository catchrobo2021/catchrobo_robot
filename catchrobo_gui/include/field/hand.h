#ifndef Hand_H
#define Hand_H

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
class Hand;
}

namespace hand
{
class Hand: public rviz::Panel
{
    Q_OBJECT

public:
    //explicit Blue2_field(QWidget *parent = 0);
    Hand(QWidget *parent = 0);
    ~Hand() override; 

    int Arr[27];
    int pos[2] = {0,0};
    int flag[2] = {0,0};

    void onInitialize() override;
    void onEnable();
    void onDisable();
    void arrayback(const std_msgs::Int32MultiArray& msg);

private Q_SLOTS:
  void dialValueChanged(int value);
  void lineEditChanged();
  void move1(int);
  void move2(int);
  void send_msg();

protected:
  Ui::Hand *ui;
  QGraphicsScene *scene;
  QGraphicsPixmapItem *pix;
  QIcon icon;

  int sendtime = 100; //ms
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  ros::Timer ti_; 
  ros::NodeHandle n;
};
}
#endif // Hand_H
