#include "field/blue_field_2.h"
#include "ui_blue_2.h"

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


namespace field_2
{
Blue2::Blue2(QWidget *parent) :
    rviz::Panel(parent),
    ui(new Ui::Blue2)
{
    ui->setupUi(this);
    scene = new QGraphicsScene;
    std::string project_path = ros::package::getPath("catch_robo");
    
    std::string img_path = "/img/blue_field.png";
    std::string field_img_path_str = project_path+img_path;
    QString field_img_path = QString::fromLocal8Bit(field_img_path_str.c_str());
    pix = new QGraphicsPixmapItem(QPixmap::fromImage(QImage(field_img_path)));
    scene->addItem(pix);
    ui->graphicsView->setScene(scene);

    std::string img_on_path = "/img/red_color.png";
    std::string img_off_path = "/img/red_bw.png";
    std::string icon_on_path_str = project_path+img_on_path;
    std::string icon_off_path_str = project_path+img_off_path;
    QString icon_on_path = QString::fromLocal8Bit(icon_on_path_str.c_str());
    QString icon_off_path = QString::fromLocal8Bit(icon_off_path_str.c_str());
    QPixmap pixmap_on(icon_on_path);
    QPixmap pixmap_off(icon_off_path); 
    icon.addFile(icon_on_path, QSize(64,64), QIcon::Normal, QIcon::On);
    icon.addFile(icon_off_path, QSize(64,64), QIcon::Normal, QIcon::Off);
    for(int i=1; i<28; i++){
      findChild<QPushButton*>(QString("obj"+QString::number(i)))->setIcon(icon);
      findChild<QPushButton*>(QString("obj"+QString::number(i)))->setChecked(1);
    }
    pub_ = nh_.advertise<std_msgs::Int32MultiArray>("obj", 1);
    sub_ = n.subscribe("ob", sendtime, &Blue2::arrayback, this);
}

Blue2::~Blue2() = default;

void Blue2::onInitialize()
{
  for(int i=1; i<28; i++){
    connect(findChild<QPushButton*>(QString("obj"+QString::number(i))), SIGNAL(clicked()), this, SLOT(buttonClicked()));
  }

  //QTimer::singleShot(sendtime, this, SLOT(send_msg()));
}

void Blue2::onEnable()
{
  show();
  parentWidget()->show();
}

void Blue2::onDisable()
{
  hide();
  parentWidget()->hide();
}

void Blue2::buttonClicked()
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

void Blue2::send_msg()
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

void Blue2::arrayback(const std_msgs::Int32MultiArray& msg){
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

PLUGINLIB_EXPORT_CLASS(field_2::Blue2, rviz::Panel )
