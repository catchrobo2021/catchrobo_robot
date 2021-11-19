#include "field/servo.h"
#include "ui_servo.h"

#include <pluginlib/class_list_macros.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <QTimer>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <tf/transform_listener.h>
#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>
#include <pluginlib/class_list_macros.h>
#include <ros/package.h>
#include <iostream>


namespace servo
{
Servo::Servo(QWidget *parent) :
    rviz::Panel(parent),
    ui(new Ui::Servo)
{
    ui->setupUi(this);
    scene = new QGraphicsScene;

    pub_ = nh_.advertise<std_msgs::Bool>("arm0_controller/enable_joints", 1);
    //sub_ = n.subscribe("servo_sub", sendtime, &servo::arrayback, this);
}

Servo::~Servo() = default;

void Servo::onInitialize()
{
  connect(findChild<QPushButton*>(QString("btn1")), SIGNAL(clicked()), this, SLOT(engage()));
  connect(findChild<QPushButton*>(QString("btn2")), SIGNAL(clicked()), this, SLOT(idle()));
  //connect(findChild<QPushButton*>(QString("btn3")), SIGNAL(clicked()), this, SLOT(start()));
  //connect(findChild<QPushButton*>(QString("btn4")), SIGNAL(clicked()), this, SLOT(stop()));
  //QTimer::singleShot(sendtime, this, SLOT(send_msg()));
}

void Servo::send()
{
  std_msgs::Bool send_data;
  send_data.data = ( data == 0 ) ? true : false;
  pub_.publish(send_data);
}

void Servo::onEnable()
{
  show();
  parentWidget()->show();
}

void Servo::onDisable()
{
  hide();
  parentWidget()->hide();
}

void Servo::engage(){
  data = 0;
  ui->btn1->setChecked(1);
  ui->btn2->setChecked(0);
  ui->btn3->setChecked(0);
  ui->btn4->setChecked(0);
  this->send();
}
void Servo::idle(){
  data = 1;
  ui->btn1->setChecked(0);
  ui->btn2->setChecked(1);
  ui->btn3->setChecked(0);
  ui->btn4->setChecked(0);
  this->send();
}
void Servo::start(){
  data = 2;
  ui->btn1->setChecked(0);
  ui->btn2->setChecked(0);
  ui->btn3->setChecked(1);
  ui->btn4->setChecked(0);
  this->send();
}
void Servo::stop(){
  data = 3;
  ui->btn1->setChecked(0);
  ui->btn2->setChecked(0);
  ui->btn3->setChecked(0);
  ui->btn4->setChecked(1);
  this->send();
}

}

PLUGINLIB_EXPORT_CLASS(servo::Servo, rviz::Panel )
