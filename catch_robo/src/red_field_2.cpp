#include "field/red_field_2.h"
#include "ui_red_2.h"

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
Red2::Red2(QWidget *parent) :
    rviz::Panel(parent),
    ui(new Ui::Red2)
{
    ui->setupUi(this);
    scene = new QGraphicsScene;
    std::string pa = ros::package::getPath("catch_robo");
    std::string pat = "/img/red_field.png";
    std::string path = pa+pat;
    QString test = QString::fromLocal8Bit(path.c_str());
    //sstd::cout << pat+pa << std::endl;
    pix = new QGraphicsPixmapItem(QPixmap::fromImage(QImage(test)));
    scene->addItem(pix);
    ui->graphicsView->setScene(scene);
    pub_ = nh_.advertise<std_msgs::Int32MultiArray>("obj", 1);
    sub_ = n.subscribe("ob", sendtime, &Red2::arrayback, this);
}

Red2::~Red2() = default;

void Red2::onInitialize()
{
  connect(ui->obj1, SIGNAL(clicked()), this, SLOT(buttonClicked()));
  connect(ui->obj2, SIGNAL(clicked()), this, SLOT(buttonClicked()));
  connect(ui->obj3, SIGNAL(clicked()), this, SLOT(buttonClicked()));
  connect(ui->obj4, SIGNAL(clicked()), this, SLOT(buttonClicked()));
  connect(ui->obj5, SIGNAL(clicked()), this, SLOT(buttonClicked()));
  connect(ui->obj6, SIGNAL(clicked()), this, SLOT(buttonClicked()));
  connect(ui->obj7, SIGNAL(clicked()), this, SLOT(buttonClicked()));
  connect(ui->obj8, SIGNAL(clicked()), this, SLOT(buttonClicked()));
  connect(ui->obj9, SIGNAL(clicked()), this, SLOT(buttonClicked()));
  connect(ui->obj10, SIGNAL(clicked()), this, SLOT(buttonClicked()));
  connect(ui->obj11, SIGNAL(clicked()), this, SLOT(buttonClicked()));
  connect(ui->obj12, SIGNAL(clicked()), this, SLOT(buttonClicked()));
  connect(ui->obj13, SIGNAL(clicked()), this, SLOT(buttonClicked()));
  connect(ui->obj14, SIGNAL(clicked()), this, SLOT(buttonClicked()));
  connect(ui->obj15, SIGNAL(clicked()), this, SLOT(buttonClicked()));
  connect(ui->obj16, SIGNAL(clicked()), this, SLOT(buttonClicked()));
  connect(ui->obj17, SIGNAL(clicked()), this, SLOT(buttonClicked()));
  connect(ui->obj18, SIGNAL(clicked()), this, SLOT(buttonClicked()));
  connect(ui->obj19, SIGNAL(clicked()), this, SLOT(buttonClicked()));
  connect(ui->obj20, SIGNAL(clicked()), this, SLOT(buttonClicked()));
  connect(ui->obj21, SIGNAL(clicked()), this, SLOT(buttonClicked()));
  connect(ui->obj22, SIGNAL(clicked()), this, SLOT(buttonClicked()));
  connect(ui->obj23, SIGNAL(clicked()), this, SLOT(buttonClicked()));
  connect(ui->obj24, SIGNAL(clicked()), this, SLOT(buttonClicked()));
  connect(ui->obj25, SIGNAL(clicked()), this, SLOT(buttonClicked()));
  connect(ui->obj26, SIGNAL(clicked()), this, SLOT(buttonClicked()));
  connect(ui->obj27, SIGNAL(clicked()), this, SLOT(buttonClicked()));
}

void Red2::onEnable()
{
  show();
  parentWidget()->show();
}

void Red2::onDisable()
{
  hide();
  parentWidget()->hide();
}

void Red2::buttonClicked()
{
  std_msgs::Int32MultiArray array;
  array.data.resize(27);
  array.data[0] = ui->obj1->isChecked();
  array.data[1] = ui->obj2->isChecked();
  array.data[2] = ui->obj3->isChecked();
  array.data[3] = ui->obj4->isChecked();
  array.data[4] = ui->obj5->isChecked();
  array.data[5] = ui->obj6->isChecked();
  array.data[6] = ui->obj7->isChecked();
  array.data[7] = ui->obj8->isChecked();
  array.data[8] = ui->obj9->isChecked();
  array.data[9] = ui->obj10->isChecked();
  array.data[10] = ui->obj11->isChecked();
  array.data[11] = ui->obj12->isChecked();
  array.data[12] = ui->obj13->isChecked();
  array.data[13] = ui->obj14->isChecked();
  array.data[14] = ui->obj15->isChecked();
  array.data[15] = ui->obj16->isChecked();
  array.data[16] = ui->obj17->isChecked();
  array.data[17] = ui->obj18->isChecked();
  array.data[18] = ui->obj19->isChecked();
  array.data[19] = ui->obj20->isChecked();
  array.data[20] = ui->obj21->isChecked();
  array.data[21] = ui->obj22->isChecked();
  array.data[22] = ui->obj23->isChecked();
  array.data[23] = ui->obj24->isChecked();
  array.data[24] = ui->obj25->isChecked();
  array.data[25] = ui->obj26->isChecked();
  array.data[26] = ui->obj27->isChecked();
  //std_msgs::Float64 msg;
  //msg.data = static_cast<double>(count);
  pub_.publish(array);
  ROS_INFO("You pushed the button.");
  //ROS_INFO_STREAM(array);
}

void Red2::send_msg()
{
  std_msgs::Int32MultiArray array;
  array.data.resize(27);
  array.data[0] = ui->obj1->isChecked();
  array.data[1] = ui->obj2->isChecked();
  array.data[2] = ui->obj3->isChecked();
  array.data[3] = ui->obj4->isChecked();
  array.data[4] = ui->obj5->isChecked();
  array.data[5] = ui->obj6->isChecked();
  array.data[6] = ui->obj7->isChecked();
  array.data[7] = ui->obj8->isChecked();
  array.data[8] = ui->obj9->isChecked();
  array.data[9] = ui->obj10->isChecked();
  array.data[10] = ui->obj11->isChecked();
  array.data[11] = ui->obj12->isChecked();
  array.data[12] = ui->obj13->isChecked();
  array.data[13] = ui->obj14->isChecked();
  array.data[14] = ui->obj15->isChecked();
  array.data[15] = ui->obj16->isChecked();
  array.data[16] = ui->obj17->isChecked();
  array.data[17] = ui->obj18->isChecked();
  array.data[18] = ui->obj19->isChecked();
  array.data[19] = ui->obj20->isChecked();
  array.data[20] = ui->obj21->isChecked();
  array.data[21] = ui->obj22->isChecked();
  array.data[22] = ui->obj23->isChecked();
  array.data[23] = ui->obj24->isChecked();
  array.data[24] = ui->obj25->isChecked();
  array.data[25] = ui->obj26->isChecked();
  array.data[26] = ui->obj27->isChecked();
  pub_.publish(array);
  ROS_INFO("You pushed.");

  QTimer::singleShot(sendtime, this, SLOT(send_msg()));
}

void Red2::arrayback(const std_msgs::Int32MultiArray& msg){
  // print all the remaining numbers

  int num = msg.data.size();
  ROS_INFO("I susclibed [%i]", num);
  for (int i = 0; i < num; i++)
  {
    ROS_INFO("[%i]:%d", i, msg.data[i]);
  }
  ROS_INFO("You get msg.");
  ui->obj1->setChecked(msg.data[0]);
  ui->obj2->setChecked(msg.data[1]);
  ui->obj3->setChecked(msg.data[2]);
  ui->obj4->setChecked(msg.data[3]);
  ui->obj5->setChecked(msg.data[4]);
  ui->obj6->setChecked(msg.data[5]);
  ui->obj7->setChecked(msg.data[6]);
  ui->obj8->setChecked(msg.data[7]);
  ui->obj9->setChecked(msg.data[8]);
  ui->obj10->setChecked(msg.data[9]);
  ui->obj11->setChecked(msg.data[10]);
  ui->obj12->setChecked(msg.data[11]);
  ui->obj13->setChecked(msg.data[12]);
  ui->obj14->setChecked(msg.data[13]);
  ui->obj15->setChecked(msg.data[14]);
  ui->obj16->setChecked(msg.data[15]);
  ui->obj17->setChecked(msg.data[16]);
  ui->obj18->setChecked(msg.data[17]);
  ui->obj19->setChecked(msg.data[18]);
  ui->obj20->setChecked(msg.data[19]);
  ui->obj21->setChecked(msg.data[20]);
  ui->obj22->setChecked(msg.data[21]);
  ui->obj23->setChecked(msg.data[22]);
  ui->obj24->setChecked(msg.data[23]);
  ui->obj25->setChecked(msg.data[24]);
  ui->obj26->setChecked(msg.data[25]);
  ui->obj27->setChecked(msg.data[26]);
}

}

PLUGINLIB_EXPORT_CLASS(field_2::Red2, rviz::Panel )
