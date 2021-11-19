#include "field/shoot.h"
#include "ui_shoot.h"

#include <pluginlib/class_list_macros.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include <std_msgs/Float64.h>
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


namespace shoot
{
Shoot::Shoot(QWidget *parent) :
    rviz::Panel(parent),
    ui(new Ui::Shoot)
{
    ui->setupUi(this);
    scene = new QGraphicsScene;
    
    pub_ = nh_.advertise<std_msgs::Int32MultiArray>("shoot", 1);
    //sub_ = n.subscribe("shoot_sub", sendtime, &Shoot::arrayback, this);
}

Shoot::~Shoot() = default;

void Shoot::onInitialize()
{
  connect(findChild<QDial*>(QString("dial1")), SIGNAL(valueChanged(int)), this, SLOT(move1(int)));
  connect(findChild<QDial*>(QString("dial2")), SIGNAL(valueChanged(int)), this, SLOT(move2(int)));
  connect(findChild<QDial*>(QString("dial3")), SIGNAL(valueChanged(int)), this, SLOT(move3(int)));
  //QTimer::singleShot(sendtime, this, SLOT(send_msg()));
 }

void Shoot::onEnable()
{
  show();
  parentWidget()->show();
}

void Shoot::onDisable()
{
  hide();
  parentWidget()->hide();
}

void Shoot::move1(int value)
{
  pos[0] = value%4;
  if(value != 4)this->send_msg();
}

void Shoot::move2(int value)
{
  pos[1] = value%4;
  if(value != 4)this->send_msg();
}

void Shoot::move3(int value)
{
  pos[2] = value%4;
  if(value != 4)this->send_msg();
}

void Shoot::send_msg(){
  std_msgs::Int32MultiArray array;
  array.data.resize(3);
  array.data[0] = pos[0];
  array.data[1] = pos[1];
  array.data[2] = pos[2];
  pub_.publish(array);
}

void Shoot::arrayback(const std_msgs::Int32MultiArray& msg){
  // print all the remaining numbers

  int num = msg.data.size();
  ROS_INFO("I susclibed [%i]", num);
}

}

PLUGINLIB_EXPORT_CLASS(shoot::Shoot, rviz::Panel )
