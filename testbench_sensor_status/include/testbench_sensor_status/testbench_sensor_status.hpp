#pragma once

#ifndef Q_MOC_RUN

#include <ros/ros.h>
#include <ros/console.h>

#include <rqt_gui_cpp/plugin.h>
#include <testbench_sensor_status/ui_testbench_sensor_status.h>
#include <roboy_communication_middleware/MotorStatus.h>
#include <roboy_communication_middleware/MotorAngle.h>
#include <QWidget>
#include <QPushButton>
#include <map>
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <common_utilities/CommonDefinitions.h>
#include <opencv2/opencv.hpp>
#endif

class TestbenchSensorStatus
        : public rqt_gui_cpp::Plugin {
    Q_OBJECT
public:
    TestbenchSensorStatus();

    virtual void initPlugin(qt_gui_cpp::PluginContext &context);

    virtual void shutdownPlugin();

    virtual void saveSettings(qt_gui_cpp::Settings &plugin_settings,
                              qt_gui_cpp::Settings &instance_settings) const;

    virtual void restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                 const qt_gui_cpp::Settings &instance_settings);


public Q_SLOTS:
    void plotData();
    void rescale();
    void AngleCalibration();

private:

    void SensorStatus(const roboy_communication_middleware::MotorAngle::ConstPtr &msg);

Q_SIGNALS:

    void newData();
private:
    Ui::TestbenchSensorStatus ui;
    QWidget *widget_;


    double ground_angle =0.0;
    int angle_overflow_counter = 5;
    double current_angle;
    QVector<double> time;
    int counter = 0;
    QVector<double> testBenchData[2];
    double new_entry_temp;

    QVector<double> motorData[NUMBER_OF_FPGAS][NUMBER_OF_MOTORS_PER_FPGA][4];
    bool motorConnected[NUMBER_OF_FPGAS][NUMBER_OF_MOTORS_PER_FPGA];



    int samples_per_plot = 5000;
    QColor color_pallette[14] = {Qt::blue, Qt::red, Qt::green, Qt::cyan, Qt::magenta, Qt::darkGray, Qt::darkRed, Qt::darkGreen,
                                 Qt::darkBlue, Qt::darkCyan, Qt::darkMagenta, Qt::darkYellow, Qt::black, Qt::gray};
    ros::NodeHandlePtr nh;
    ros::Subscriber benchStatus;
    ros::Time start_time;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
private:
    std::map<std::string, QPushButton*> button;
};
