#ifndef Red_BOX_H
#define Red_BOX_H

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
class red_box;
}

namespace shootbox
{
class red_box: public rviz::Panel
{
    Q_OBJECT

public:
    //explicit red_box_field(QWidget *parent = 0);
    red_box(QWidget *parent = 0);
    ~red_box() override; 

    int Arr[27];
    int box[7] = {0,0,0,0,0,0};

    void onInitialize() override;
    void check_goal();
    void onEnable();
    void onDisable();
    void arrayback(const std_msgs::Int32MultiArray& msg);

private Q_SLOTS:
  void dialValueChanged(int value);
  void lineEditChanged();
  void click1_up();
  void click1_dn();
  void click2_up();
  void click2_dn();
  void click3_up();
  void click3_dn();
  void click4_up();
  void click4_dn();
  void click5_up();
  void click5_dn();
  void click6_up();
  void click6_dn();
  void send_msg();

protected:
  Ui::red_box *ui;
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
#endif // red_box_FIELD_2_H
