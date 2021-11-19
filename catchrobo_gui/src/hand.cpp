#include "field/hand.h"
#include "ui_hand.h"

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


namespace hand
{
Hand::Hand(QWidget *parent) :
    rviz::Panel(parent),
    ui(new Ui::Hand)
{
    ui->setupUi(this);
    scene = new QGraphicsScene;
    
    pub_ = nh_.advertise<std_msgs::Int32MultiArray>("hand", 1);
    //sub_ = n.subscribe("Hand_sub", sendtime, &Hand::arrayback, this);
}

Hand::~Hand() = default;

void Hand::onInitialize()
{
  connect(findChild<QSlider*>(QString("hs1")), SIGNAL(valueChanged(int)), this, SLOT(move1(int)));
  connect(findChild<QSlider*>(QString("hs2")), SIGNAL(valueChanged(int)), this, SLOT(move2(int)));
  //QTimer::singleShot(sendtime, this, SLOT(send_msg()));
 }

void Hand::onEnable()
{
  show();
  parentWidget()->show();
}

void Hand::onDisable()
{
  hide();
  parentWidget()->hide();
}

void Hand::move1(int value)
{
  if(ui->cb1->isChecked() == 0 && value>80){
    ui->hs1->setValue(80);
    pos[0] = 80;
  }else{
    pos[0] = value;
  }
  this->send_msg();
}

void Hand::move2(int value)
{
  if(ui->cb2->isChecked() == 0 && value>80){
    ui->hs2->setValue(80);
    pos[1] = 80;
  }else{
    pos[1] = value;
  }
  this->send_msg();
}

void Hand::send_msg(){
  std_msgs::Int32MultiArray array;
  array.data.resize(2);
  array.data[0] = pos[0];
  array.data[1] = pos[1];
  pub_.publish(array);
}

}

PLUGINLIB_EXPORT_CLASS(hand::Hand, rviz::Panel )
