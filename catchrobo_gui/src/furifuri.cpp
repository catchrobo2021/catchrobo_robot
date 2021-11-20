#include "field/furifuri.h"
#include "ui_furifuri.h"

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


namespace furifuri
{
Furifuri::Furifuri(QWidget *parent) :
    rviz::Panel(parent),
    ui(new Ui::Furifuri)
{
    ui->setupUi(this);
    scene = new QGraphicsScene;

    pub_ = nh_.advertise<std_msgs::Int8>("sorter/open_row_gui", 1);
    pub2_ = nh_.advertise<std_msgs::Int8>("sorter/close_row_gui", 1);
    sub_ = n.subscribe("sorter/open_row", sendtime, &Furifuri::arrayback, this);
    sub2_ = n.subscribe("sorter/close_row", sendtime, &Furifuri::arrayback2, this);
}

Furifuri::~Furifuri() = default;

void Furifuri::onInitialize()
{
  connect(findChild<QPushButton*>(QString("btn1")), SIGNAL(clicked()), this, SLOT(furi1()));
  connect(findChild<QPushButton*>(QString("btn2")), SIGNAL(clicked()), this, SLOT(furi2()));
  connect(findChild<QPushButton*>(QString("btn3")), SIGNAL(clicked()), this, SLOT(furi3()));
  connect(findChild<QPushButton*>(QString("btn4")), SIGNAL(clicked()), this, SLOT(furi4()));
  connect(findChild<QPushButton*>(QString("btn5")), SIGNAL(clicked()), this, SLOT(furi5()));
  connect(findChild<QPushButton*>(QString("btn6")), SIGNAL(clicked()), this, SLOT(furi6()));
  connect(findChild<QPushButton*>(QString("btn7")), SIGNAL(clicked()), this, SLOT(furi7()));
  connect(findChild<QPushButton*>(QString("btn8")), SIGNAL(clicked()), this, SLOT(furi8()));
  connect(findChild<QPushButton*>(QString("btn9")), SIGNAL(clicked()), this, SLOT(furi9()));
}

void Furifuri::send()
{
  std_msgs::Int8 send_data;
  send_data.data = furi;
  pub_.publish(send_data);
}

void Furifuri::send_center()
{
  std_msgs::Int8 send_data;
  send_data.data = furi_center;
  pub2_.publish(send_data);
}

void Furifuri::onEnable()
{
  show();
  parentWidget()->show();
}

void Furifuri::onDisable()
{
  hide();
  parentWidget()->hide();
}

void Furifuri::furi1(){
  furi = 1;
  this->send();
  ui->btn1->setChecked(1);
  ui->btn2->setChecked(0);
  ui->btn3->setChecked(0);
}
void Furifuri::furi2(){
  furi = 0;
  this->send();
  ui->btn1->setChecked(0);
  ui->btn2->setChecked(1);
  ui->btn3->setChecked(0);
}
void Furifuri::furi3(){
  furi_center = 0;
  this->send_center();
  ui->btn1->setChecked(0);
  ui->btn2->setChecked(0);
  ui->btn3->setChecked(1);
}
void Furifuri::furi4(){
  furi = 3;
  this->send();
  ui->btn4->setChecked(1);
  ui->btn5->setChecked(0);
  ui->btn6->setChecked(0);
}
void Furifuri::furi5(){
  furi = 2;
  this->send();
  ui->btn4->setChecked(0);
  ui->btn5->setChecked(1);
  ui->btn6->setChecked(0);
}
void Furifuri::furi6(){
  furi_center = 2;
  this->send_center();
  ui->btn4->setChecked(0);
  ui->btn5->setChecked(0);
  ui->btn6->setChecked(1);
}
void Furifuri::furi7(){
  furi = 5;
  this->send();
  ui->btn7->setChecked(1);
  ui->btn8->setChecked(0);
  ui->btn9->setChecked(0);
}
void Furifuri::furi8(){
  furi = 4;
  this->send();
  ui->btn7->setChecked(0);
  ui->btn8->setChecked(1);
  ui->btn9->setChecked(0);
}
void Furifuri::furi9(){
  furi_center = 4;
  this->send_center();
  ui->btn7->setChecked(0);
  ui->btn8->setChecked(0);
  ui->btn9->setChecked(1);
}


void Furifuri::arrayback(const std_msgs::Int8& msg){
  int num = msg.data;
  if(num == 0){
    ui->btn1->setChecked(0);
    ui->btn2->setChecked(1);
    ui->btn3->setChecked(0);
  }else if(num == 1){
    ui->btn1->setChecked(1);
    ui->btn2->setChecked(0);
    ui->btn3->setChecked(0);
  }else if(num == 2){
    ui->btn4->setChecked(0);
    ui->btn5->setChecked(1);
    ui->btn6->setChecked(0);
  }else if(num == 3){
    ui->btn4->setChecked(1);
    ui->btn5->setChecked(0);
    ui->btn6->setChecked(0);
  }else if(num == 4){
    ui->btn7->setChecked(0);
    ui->btn8->setChecked(1);
    ui->btn9->setChecked(0);
  }else if(num == 5){
    ui->btn7->setChecked(1);
    ui->btn8->setChecked(0);
    ui->btn9->setChecked(0);
  }
}

void Furifuri::arrayback2(const std_msgs::Int8& msg){
  int num = msg.data;
  if(num == 0){
    ui->btn1->setChecked(0);
    ui->btn2->setChecked(0);
    ui->btn3->setChecked(1);
  }else if(num == 1){
    ui->btn1->setChecked(0);
    ui->btn2->setChecked(0);
    ui->btn3->setChecked(1);
  }else if(num == 2){
    ui->btn4->setChecked(0);
    ui->btn5->setChecked(0);
    ui->btn6->setChecked(1);
  }else if(num == 3){
    ui->btn4->setChecked(0);
    ui->btn5->setChecked(0);
    ui->btn6->setChecked(1);
  }else if(num == 4){
    ui->btn7->setChecked(0);
    ui->btn8->setChecked(0);
    ui->btn9->setChecked(1);
  }else if(num == 5){
    ui->btn7->setChecked(0);
    ui->btn8->setChecked(0);
    ui->btn9->setChecked(1);
  }
}

}

PLUGINLIB_EXPORT_CLASS(furifuri::Furifuri, rviz::Panel )
