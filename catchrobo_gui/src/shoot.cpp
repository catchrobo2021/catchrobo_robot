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
    std::string project_path = ros::package::getPath("catchrobo_gui");
    
    pub_ = nh_.advertise<std_msgs::Int32MultiArray>("shoot_pub", 1);
    sub_ = n.subscribe("shoot_sub", sendtime, &Shoot::arrayback, this);
}

Shoot::~Shoot() = default;

void Shoot::onInitialize()
{
  connect(findChild<QPushButton*>(QString("nob1")), SIGNAL(clicked()), this, SLOT(move()));
  connect(findChild<QPushButton*>(QString("nob2")), SIGNAL(clicked()), this, SLOT(move()));
  connect(findChild<QPushButton*>(QString("nob3")), SIGNAL(clicked()), this, SLOT(move()));
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

void Shoot::buttonClicked()
{
  std_msgs::Int32MultiArray array;
  array.data.resize(27);
  for(int i=1; i<28; i++){
      array.data[i-1] = findChild<QPushButton*>(QString("obj"+QString::number(i)))->isChecked();
  }
  pub_.publish(array);
  ROS_INFO("You pushed the button.");
  //ROS_INFO_STREAM(array);
}

void Shoot::send_msg()
{
  std_msgs::Int32MultiArray array;
  array.data.resize(27);
  for(int i=1; i<28; i++){
      array.data[i-1] = findChild<QPushButton*>(QString("obj"+QString::number(i)))->isChecked();
  }
  pub_.publish(array);
  ROS_INFO("You pushed.");

  QTimer::singleShot(sendtime, this, SLOT(send_msg()));
}

void Shoot::arrayback(const std_msgs::Int32MultiArray& msg){
  // print all the remaining numbers

  int num = msg.data.size();
  ROS_INFO("I susclibed [%i]", num);
  for (int i = 0; i < num; i++)
  {
    ROS_INFO("[%i]:%d", i, msg.data[i]);
  }
  ROS_INFO("You get msg.");
  for(int i=0; i<num; i++){
      findChild<QPushButton*>(QString("obj"+QString::number(i+1)))->setChecked(msg.data[i]);
  }
}

}

PLUGINLIB_EXPORT_CLASS(field_2::Shoot, rviz::Panel )
