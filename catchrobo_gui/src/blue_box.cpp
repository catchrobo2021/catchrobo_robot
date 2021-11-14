#include "field/blue_box.h"
#include "ui_blue_box.h"

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


namespace shootbox
{
blue_box::blue_box(QWidget *parent) :
    rviz::Panel(parent),
    ui(new Ui::blue_box)
{
    ui->setupUi(this);
    scene = new QGraphicsScene;
    project_path = ros::package::getPath("catchrobo_gui");
    
    std::string img_path = "/img/blue_box.png";
    std::string field_img_path_str = project_path+img_path;
    QString field_img_path = QString::fromLocal8Bit(field_img_path_str.c_str());
    pix = new QGraphicsPixmapItem(QPixmap::fromImage(QImage(field_img_path)));
    pix->setScale(1.4);
    scene->addItem(pix);
    ui->graphicsView->setScene(scene);

    std::string img_on_path = "/img/red_color_2.png";
    std::string img_off_path = "/img/red_bw_2.png";
    std::string icon_on_path_str = project_path+img_on_path;
    std::string icon_off_path_str = project_path+img_off_path;
    QString icon_on_path = QString::fromLocal8Bit(icon_on_path_str.c_str());
    QString icon_off_path = QString::fromLocal8Bit(icon_off_path_str.c_str());
    icon.addFile(icon_on_path, QSize(128,128), QIcon::Normal, QIcon::On);
    icon.addFile(icon_off_path, QSize(128,128), QIcon::Normal, QIcon::Off);
    for(int i=1; i<7; i++){
      for(int j=1; j<5; j++){
        findChild<QPushButton*>(QString("ob"+QString::number(i)+QString::number(j)))->setIcon(icon);
        findChild<QPushButton*>(QString("ob"+QString::number(i)+QString::number(j)))->setChecked(0);
      }
    }

    std::string img_on_path_up = "/img/red_button.png";
    std::string img_off_path_up = "/img/red_button.png";
    std::string icon_on_path_str_up = project_path+img_on_path_up;
    std::string icon_off_path_str_up = project_path+img_off_path_up;
    QString icon_on_path_up = QString::fromLocal8Bit(icon_on_path_str_up.c_str());
    QString icon_off_path_up = QString::fromLocal8Bit(icon_off_path_str_up.c_str());
    icon_up.addFile(icon_on_path_up, QSize(64,64), QIcon::Normal, QIcon::On);
    icon_up.addFile(icon_off_path_up, QSize(64,64), QIcon::Normal, QIcon::Off);
    for(int i=1; i<7; i++){
      findChild<QPushButton*>(QString("up"+QString::number(i)))->setIcon(icon_up);
    }

    std::string img_on_path_dn = "/img/blue_button.png";
    std::string img_off_path_dn = "/img/blue_button.png";
    std::string icon_on_path_str_dn = project_path+img_on_path_dn;
    std::string icon_off_path_str_dn = project_path+img_off_path_dn;
    QString icon_on_path_dn = QString::fromLocal8Bit(icon_on_path_str_dn.c_str());
    QString icon_off_path_dn = QString::fromLocal8Bit(icon_off_path_str_dn.c_str());
    icon_dn.addFile(icon_on_path_dn, QSize(64,64), QIcon::Normal, QIcon::On);
    icon_dn.addFile(icon_off_path_dn, QSize(64,64), QIcon::Normal, QIcon::Off);
    for(int i=1; i<7; i++){
      findChild<QPushButton*>(QString("dn"+QString::number(i)))->setIcon(icon_dn);
    }

    pub_ = nh_.advertise<std_msgs::Int32MultiArray>("goal_pub", 1);
    sub_ = n.subscribe("goal_sub", sendtime, &blue_box::arrayback, this);
}

blue_box::~blue_box() = default;

void blue_box::onInitialize()
{
  connect(findChild<QPushButton*>(QString("up1")), SIGNAL(clicked()), this, SLOT(click1_up()));
  connect(findChild<QPushButton*>(QString("dn1")), SIGNAL(clicked()), this, SLOT(click1_dn()));
  connect(findChild<QPushButton*>(QString("up2")), SIGNAL(clicked()), this, SLOT(click2_up()));
  connect(findChild<QPushButton*>(QString("dn2")), SIGNAL(clicked()), this, SLOT(click2_dn()));
  connect(findChild<QPushButton*>(QString("up3")), SIGNAL(clicked()), this, SLOT(click3_up()));
  connect(findChild<QPushButton*>(QString("dn3")), SIGNAL(clicked()), this, SLOT(click3_dn()));
  connect(findChild<QPushButton*>(QString("up4")), SIGNAL(clicked()), this, SLOT(click4_up()));
  connect(findChild<QPushButton*>(QString("dn4")), SIGNAL(clicked()), this, SLOT(click4_dn()));
  connect(findChild<QPushButton*>(QString("up5")), SIGNAL(clicked()), this, SLOT(click5_up()));
  connect(findChild<QPushButton*>(QString("dn5")), SIGNAL(clicked()), this, SLOT(click5_dn()));
  connect(findChild<QPushButton*>(QString("up6")), SIGNAL(clicked()), this, SLOT(click6_up()));
  connect(findChild<QPushButton*>(QString("dn6")), SIGNAL(clicked()), this, SLOT(click6_dn()));
  //QTimer::singleShot(sendtime, this, SLOT(send_msg()));
}

void blue_box::check_goal()
{
  for(int i=0; i<7; i++){
    if(this->box[i]<0)box[i]=0;
    if(this->box[i]>4)box[i]=4;
  }

  ui->num1->display(this->box[0]);
  ui->num2->display(this->box[1]);
  ui->num3->display(this->box[2]);
  ui->num4->display(this->box[3]);
  ui->num5->display(this->box[4]);
  ui->num6->display(this->box[5]);

  std::string img_on_path = "/img/red_color_2.png";
  std::string img_off_path = "/img/red_bw_2.png";
  std::string icon_on_path_str = this->project_path+img_on_path;
  std::string icon_off_path_str = this->project_path+img_off_path;
  QString icon_on_path = QString::fromLocal8Bit(icon_on_path_str.c_str());
  QString icon_off_path = QString::fromLocal8Bit(icon_off_path_str.c_str());
  icon_on.addFile(icon_on_path, QSize(128,128), QIcon::Normal, QIcon::Off);
  icon_off.addFile(icon_off_path, QSize(128,128), QIcon::Normal, QIcon::Off);
  for(int i=1; i<7; i++){
    for(int j=1; j<5; j++){
      if(this->box[i-1]>j-1){
        findChild<QPushButton*>(QString("ob"+QString::number(i)+QString::number(j)))->setIcon(icon_on);
      }else{
        findChild<QPushButton*>(QString("ob"+QString::number(i)+QString::number(j)))->setIcon(icon_off);
      }
    }
  }

  std_msgs::Int32MultiArray array;
  array.data.resize(6);
  for(int i=0; i<6; i++){
    array.data[i] = this->box[i];
  }
  pub_.publish(array);
  //ROS_INFO("0:%i,1:%i,2:%i,3:%i,4:%i,5:%i",this->box[0],this->box[1],this->box[2],this->box[3],this->box[4],this->box[5]);
}

void blue_box::onEnable()
{
  show();
  parentWidget()->show();
}

void blue_box::onDisable()
{
  hide();
  parentWidget()->hide();
}

void blue_box::click1_up(){
  box[0] += 1;
  this->check_goal();
}
void blue_box::click1_dn(){
  box[0] -= 1;
  this->check_goal();
}
void blue_box::click2_up(){
  box[1] += 1;
  this->check_goal();
}
void blue_box::click2_dn(){
  box[1] -= 1;
  this->check_goal();
}
void blue_box::click3_up(){
  box[2] += 1;
  this->check_goal();
}
void blue_box::click3_dn(){
  box[2] -= 1;
  this->check_goal();
}
void blue_box::click4_up(){
  box[3] += 1;
  this->check_goal();
}
void blue_box::click4_dn(){
  box[3] -= 1;
  this->check_goal();
}
void blue_box::click5_up(){
  box[4] += 1;
  this->check_goal();
}
void blue_box::click5_dn(){
  box[4] -= 1;
  this->check_goal();
}
void blue_box::click6_up(){
  box[5] += 1;
  this->check_goal();
}
void blue_box::click6_dn(){
  box[5] -= 1;
  this->check_goal();
}

void blue_box::arrayback(const std_msgs::Int32MultiArray& msg){
  // print all the remaining numbers

  int num = msg.data.size();
  ROS_INFO("I susclibed [%i]", num);
  for (int i = 0; i < num; i++)
  {
    this->box[i] = msg.data[i];
    ROS_INFO("[%i]:%d", i, msg.data[i]);
  }
  this->check_goal();
}

}

PLUGINLIB_EXPORT_CLASS(shootbox::blue_box, rviz::Panel )
