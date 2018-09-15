#include <testbench_sensor_status/testbench_sensor_status.hpp>

TestbenchSensorStatus::TestbenchSensorStatus()
        : rqt_gui_cpp::Plugin(), widget_(0) {
    setObjectName("TestbenchSensorStatus");
}
//testbench_sensor_status
//TestbenchSensorStatus
void TestbenchSensorStatus::initPlugin(qt_gui_cpp::PluginContext &context) {
    qDebug() << "HEY IM BEING MADE";
    ROS_DEBUG_THROTTLE(5, "HEY IM BEING MADE");

    ROS_DEBUG("Hello %s", "World");
    ROS_DEBUG_STREAM("Hello " << "World");



    // access standalone command line arguments
    QStringList argv = context.argv();
    // create QWidget
    widget_ = new QWidget();
    // extend the widget with all attributes and children from UI file
    ui.setupUi(widget_);
    // add widget to the user interface

    context.addWidget(widget_);
    button["calibrate"] = widget_->findChild<QPushButton *>("calibrate");

    ui.position_plot->addGraph();
    ui.position_plot->graph(0)->setPen(QPen(color_pallette[0]));
    ui.temperature_plot->addGraph();
    ui.temperature_plot->graph(0)->setPen(QPen(color_pallette[0]));

    ui.position_plot->xAxis->setLabel("time[s]");
    ui.position_plot->yAxis->setRange(-10,200);
    ui.position_plot->yAxis->setLabel("mm");
    ui.position_plot->replot();

    ui.temperature_plot->xAxis->setLabel("time[s]");
    ui.temperature_plot->yAxis->setRange(0,100);
    ui.temperature_plot->yAxis->setLabel("degrees");
    ui.temperature_plot->replot();



        nh = ros::NodeHandlePtr(new ros::NodeHandle);
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "motor_status_rqt_plugin");
    }

    benchStatus = nh->subscribe("/roboy/middleware/MotorAngle", 1, &TestbenchSensorStatus::SensorStatus, this);

    QObject::connect(this, SIGNAL(newData()), this, SLOT(plotData()));
    QObject::connect(button["calibrate"], SIGNAL(clicked()), this, SLOT(AngleCalibration()));
    spinner.reset(new ros::AsyncSpinner(2));
    spinner->start();

    start_time = ros::Time::now();
}

void TestbenchSensorStatus::shutdownPlugin() {
    benchStatus.shutdown();
}

void TestbenchSensorStatus::saveSettings(qt_gui_cpp::Settings &plugin_settings,
                                    qt_gui_cpp::Settings &instance_settings) const {
    // instance_settings.setValue(k, v)
}

void TestbenchSensorStatus::restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                       const qt_gui_cpp::Settings &instance_settings) {
    // v = instance_settings.value(k)
}

void TestbenchSensorStatus::SensorStatus(const roboy_communication_middleware::MotorAngle::ConstPtr &msg) {
    ROS_DEBUG_THROTTLE(5, "receiving testbench status");
    if(msg->id == 5) {
        ros::Duration delta = (ros::Time::now() - start_time);
        time.push_back(delta.toSec());
//                testBenchData[0].push_back(msg->angles.front());



        if (current_angle - msg->angles.front() > 200)
            angle_overflow_counter ++;
        if (current_angle - msg-> angles.front() < -200)
            angle_overflow_counter  --;
         current_angle =msg->angles.front();
        //(current_angle - ground_angle) * 20/360 + 20* angle_overflow_counter;



        int n = 10;


        new_entry_temp = std::accumulate(testBenchData[1].end()- std::min(testBenchData[1].size(), n), testBenchData[1].end(), 0);
        new_entry_temp += msg->temperature.front() - 10;
        new_entry_temp /= std::min(testBenchData[1].size(), n)+1;

                testBenchData[0].push_back( (current_angle - ground_angle) * 20/360 + 20* angle_overflow_counter);
                testBenchData[1].push_back(new_entry_temp);

                if (testBenchData[0].size() > samples_per_plot) {
                    testBenchData[0].pop_front();
                    testBenchData[1].pop_front();

                }

        if (time.size() > samples_per_plot)
            time.pop_front();

        if ((counter++) % 20 == 0) {
            Q_EMIT newData();
        }

//        if (counter % 1000 == 0) {
//            if (msg->power_sense)
//                ui.power_sense->setStyleSheet("background-color:green;");
//            else
//                ui.power_sense->setStyleSheet("background-color:red;");
//            rescale();
//        }
    }
}
void TestbenchSensorStatus::AngleCalibration(){
    angle_overflow_counter = 0.0;
    ground_angle = current_angle;
}
void TestbenchSensorStatus::plotData() {
    //ui.label->setText( QString::number( testBenchData[0].back()));





        ui.position_plot->graph(0)->setData(time, testBenchData[0]);
        ui.temperature_plot->graph(0)->setData(time, testBenchData[1]);
//    ui.position_plot->graph(0)->setData(time, time);
//    ui.temperature_plot->graph(0)->setData(time, time);

    ui.position_plot->xAxis->rescale();
    ui.temperature_plot->xAxis->rescale();
//    ui.position_plot->yAxis->rescale();
//    ui.temperature_plot->yAxis->rescale();
    ui.position_plot->replot();
    ui.temperature_plot->replot();
}

void TestbenchSensorStatus::rescale(){
    double minima[2], maxima[2];
    uint minimal_motor[4] = {0,0,0,0}, maximal_motor[4] = {0,0,0,0};
    for(uint type=0;type<2;type++) {
            minima[type] = 0;
            maxima[type] = 0;
            for (auto val:testBenchData[type]) {
                if (val < minima[type])
                    minima[type] = val;
                if (val > maxima[type])
                    maxima[type] = val;
            }
        }

//        for (uint motor = 1; motor < NUMBER_OF_MOTORS_PER_FPGA; motor++) {
//            if (minima[motor][type] < minima[minimal_motor[type]][type])
//                minimal_motor[type] = motor;
//            if (maxima[motor][type] < maxima[maximal_motor[type]][type])
//                maximal_motor[type] = motor;
//        }



            ui.position_plot->graph(0)->rescaleAxes();

            ui.temperature_plot->graph(0)->rescaleAxes();


            ui.position_plot->graph(0)->rescaleAxes(true);

            ui.temperature_plot->graph(0)->rescaleAxes(true);


}

PLUGINLIB_DECLARE_CLASS(testbench_sensor_status, TestbenchSensorStatus, TestbenchSensorStatus, rqt_gui_cpp::Plugin);
