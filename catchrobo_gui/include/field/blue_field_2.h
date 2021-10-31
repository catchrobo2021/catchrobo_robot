#ifndef Blue_FIELD_2_H
#define Blue_FIELD_2_H

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
class Blue2;
}

namespace field_2
{
class Blue2: public rviz::Panel
{
    Q_OBJECT

public:
    //explicit Blue2_field(QWidget *parent = 0);
    Blue2(QWidget *parent = 0);
    ~Blue2() override; 

    int Arr[27];

    void onInitialize() override;
    void onEnable();
    void onDisable();
    void arrayback(const std_msgs::Int32MultiArray& msg);

private Q_SLOTS:
  void dialValueChanged(int value);
  void lineEditChanged();
  void buttonClicked();
  void send_msg();
  void get_msg();
  void change_state();

protected:
  Ui::Blue2 *ui;
  int value_{0};
  std::string topic_name_{"obj"};
  QGraphicsScene *scene;
  QGraphicsPixmapItem *pix;

  int sendtime = 100; //ms
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  ros::Timer ti_; 
  ros::NodeHandle n;
};
}
#endif // Blue2_FIELD_2_H
