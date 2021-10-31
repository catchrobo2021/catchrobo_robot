#include <rviz_plugin_examples/dial_panel.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64.h>

#include "ui_red.h"
//#include "ui_dial_panel.h"

namespace rviz_plugin_examples
{
DialPanel::DialPanel(QWidget* parent) : Panel(parent),  ui_(new Ui::DialUI())
{
  ui_->setupUi(this);
}

DialPanel::~DialPanel() = default;

void DialPanel::onInitialize()
{
  connect(ui_->pushButton, SIGNAL(clicked()), this, SLOT(buttonClicked()));

  pub_ = nh_.advertise<std_msgs::Float64>("dial", 1);
  parentWidget()->setVisible(true);
}

void DialPanel::onEnable()
{
  show();
  parentWidget()->show();
}

void DialPanel::onDisable()
{
  hide();
  parentWidget()->hide();
}

void DialPanel::buttonClicked()
{
  std_msgs::Float64 msg;
  msg.data = static_cast<double>(value_);
  pub_.publish(msg);
  ROS_INFO("You pushed the button.");
}

}  // namespace rviz_plugin_examples

PLUGINLIB_EXPORT_CLASS(rviz_plugin_examples::DialPanel, rviz::Panel )

/*
#include <rviz_plugin_examples/mainwindow.h>
//#include <rviz_plugin_examples/dial_panel.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64.h>

#include "ui_mainwindow.h"

namespace rviz_plugin_examples
{
ObjPanel::ObjPanel(QWidget* parent) : Panel(parent),  ui_(new Ui::Blue())
{
  ui_->setupUi(this);
}

ObjPanel::~ObjPanel() = default;

void ObjPanel::onInitialize()
{
  connect(ui_->b1, SIGNAL(clicked()), this, SLOT(buttonClicked()));

  pub_ = nh_.advertise<std_msgs::Float64>("object", 1);
  parentWidget()->setVisible(true);
}

void ObjPanel::onEnable()
{
  show();
  parentWidget()->show();
}

void ObjPanel::onDisable()
{
  hide();
  parentWidget()->hide();
}

void ObjPanel::buttonClicked()
{
  std_msgs::Float64 msg;
  msg.data = static_cast<double>(1);
  pub_.publish(msg);
  ROS_INFO("You pushed the button.");
}

}  // namespace rviz_plugin_examples

PLUGINLIB_EXPORT_CLASS(rviz_plugin_examples::ObjPanel, rviz::Panel )
*/