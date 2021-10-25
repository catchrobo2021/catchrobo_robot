#ifndef Blue_FIELD_H
#define Blue_FIELD_H

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
class Blue;
}

namespace field
{
class Blue: public rviz::Panel
{
    Q_OBJECT

public:
    //explicit Blue_field(QWidget *parent = 0);
    Blue(QWidget *parent = 0);
    ~Blue() override; 

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
  Ui::Blue *ui;
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
  virtual QSize sizeHint() const { return QSize( 150, 150 ); }
  //const QUrl url = "/home/okamoto/catkin_ws/src/rviz_plugin_examples-meloeic-devel/img/blue_field.png";
};
}
#endif // Blue_FIELD_H
