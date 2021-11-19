#ifndef Furifuri_H
#define Furifuri_H

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
class Furifuri;
}

namespace furifuri
{
class Furifuri: public rviz::Panel
{
    Q_OBJECT

public:
    //explicit Furifuri(QWidget *parent = 0);
    Furifuri(QWidget *parent = 0);
    ~Furifuri() override; 

    int furi = 0;
    int furi_center = 0;

    void onInitialize() override;
    void send();
    void send_center();
    void onEnable();
    void onDisable();
    void arrayback(const std_msgs::Int32MultiArray& msg);

private Q_SLOTS:
  void dialValueChanged(int value);
  void lineEditChanged();
  void furi1();
  void furi2();
  void furi3();
  void furi4();
  void furi5();
  void furi6();
  void furi7();
  void furi8();
  void furi9();
  void send_msg();

protected:
  Ui::Furifuri *ui;
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
  ros::Publisher pub2_;
  ros::Subscriber sub_;
  ros::Timer ti_; 
  ros::NodeHandle n;
};
}
#endif // Furifuri_H
