#include "field/menu.h"
#include "ui_menu.h"

#include <pluginlib/class_list_macros.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
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


namespace menu
{
Menu::Menu(QWidget *parent) :
    rviz::Panel(parent),
    ui(new Ui::Menu)
{
    ui->setupUi(this);
    scene = new QGraphicsScene;

    pub_ = nh_.advertise<std_msgs::Int8>("menu", 1);
}

Menu::~Menu() = default;

void Menu::onInitialize()
{
  connect(findChild<QPushButton*>(QString("btn1")), SIGNAL(clicked()), this, SLOT(start()));
  connect(findChild<QPushButton*>(QString("btn2")), SIGNAL(clicked()), this, SLOT(pause()));
  connect(findChild<QPushButton*>(QString("btn3")), SIGNAL(clicked()), this, SLOT(stop()));
}

void Menu::send()
{
  std_msgs::Int8 send_data;
  send_data.data = data;
  pub_.publish(send_data);
}

void Menu::onEnable()
{
  show();
  parentWidget()->show();
}

void Menu::onDisable()
{
  hide();
  parentWidget()->hide();
}

void Menu::start(){
  data = 1;
  ui->btn1->setChecked(1);
  ui->btn2->setChecked(0);
  ui->btn3->setChecked(0);
  this->send();
}
void Menu::pause(){
  data = 2;
  ui->btn1->setChecked(0);
  ui->btn2->setChecked(1);
  ui->btn3->setChecked(0);
  this->send();
}
void Menu::stop(){
  data = 3;
  ui->btn1->setChecked(0);
  ui->btn2->setChecked(0);
  ui->btn3->setChecked(1);
  this->send();
}

}

PLUGINLIB_EXPORT_CLASS(menu::Menu, rviz::Panel )
