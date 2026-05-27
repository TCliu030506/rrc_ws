#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <string>
#include <QApplication>
#include <QWidget>
#include <QLabel>
#include <QKeyEvent>
#include <QPixmap>
#include <QPushButton>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QPlainTextEdit>

#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <thread>
#include <chrono>

#include "sigema7_msg/msg/sigema7_position.hpp"
#include "ui_control_msg/msg/ui_control.hpp"
#include "std_msgs/msg/float64.hpp"
#include "ui_control_msg/msg/slaver_position.hpp"

const char *keyboard_input;

// 自定义 QApplication 子类
class CustomQApplication : public QApplication {
public:
    using QApplication::QApplication;

    bool notify(QObject *receiver, QEvent *event) override {
        try {
            return QApplication::notify(receiver, event);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(rclcpp::get_logger("ui_node"), "Exception in event handler: %s", e.what());
            return false;
        } catch (...) {
            RCLCPP_ERROR(rclcpp::get_logger("ui_node"), "Unknown exception in event handler");
            return false;
        }
    }
};


// 自定义窗口类，重写 keyPressEvent 方法
class FullScreenWidget : public QWidget {
public:
    using QWidget::QWidget;

protected:
    void keyPressEvent(QKeyEvent *event) override {
        if (event->key() == Qt::Key_Escape) {
            if (isFullScreen()) {
                showNormal(); // 退出全屏
            }
        } else if (event->key() == Qt::Key_F1) {
            if (!isFullScreen()) {
                showFullScreen(); // 进入全屏
            }
        }else if (event->key() == Qt::Key_Z) {
            keyboard_input = "z";
        }else if (event->key() == Qt::Key_X) {
            keyboard_input = "x";
        }else if (event->key() == Qt::Key_1) {
            keyboard_input = "1";
        }else if (event->key() == Qt::Key_2) {
            keyboard_input = "2";
        }else if (event->key() == Qt::Key_3) {
            keyboard_input = "3";
        }else if (event->key() == Qt::Key_S) {
            keyboard_input = "s";
        }
        else {
            QWidget::keyPressEvent(event);
        }
    }
};

class UINode : public rclcpp::Node
{
public:
    UINode() : Node("ui_node")
    {
        // 初始化自定义 Qt 应用
        int argc = 0;
        char **argv = nullptr;
        qt_app_ = new CustomQApplication(argc, argv);

        // 创建自定义窗口
        window_ = new FullScreenWidget();
        window_->setWindowTitle("Teleopration UI");

        // 加载 logo 图片
        QPixmap logo("/home/liutiancheng/Lab_WS/zzrobot_ws/src/teleoperation_system/ui_test/resource/logo.jpg");
        if (!logo.isNull()) {
            // 创建用于显示图片的 QLabel
            logoLabel_ = new QLabel(window_);
            // 设置图片到 QLabel
            logoLabel_->setPixmap(logo.scaled(250, 200, Qt::KeepAspectRatio));
            // 将 logo 放置在窗口左上角
            logoLabel_->move(10, -25);
            // 显示 QLabel
            logoLabel_->show();
        } else {
            RCLCPP_ERROR(get_logger(), "Failed to load logo image.");
        }

        // 创建标签
        // label_ = new QLabel("面向脑出血精准治疗的内镜手术机器人遥操作系统", window_);
        label_ = new QLabel(" 面向脑出血精准治疗的机器人辅助微创手术系统 ", window_);
        // 创建 QFont 对象并设置字体大小和样式
        QFont font;
        font.setPointSize(50); // 设置字体大小为 16 磅
        font.setBold(true);    // 设置字体为粗体
        font.setItalic(false); // 设置字体不倾斜
        // 将设置好的字体应用到标签上
        label_->setFont(font);
        // 使用样式表设置底纹和边框
        label_->setStyleSheet(
            "QLabel {"
            "    /* 设置底纹颜色 */"
            "     background-color: rgb(86, 200, 229);"
            "    /* 设置边框样式，宽度为 2 像素，实线，颜色为红色 */"
            "    border: 4px solid rgb(0, 81, 255);"
            "    /* 设置内边距，让文字与边框有间距 */"
            "    padding: 5px;"
            "    /* 设置边框圆角，使边框角变圆润 */"
            "    border-radius: 10px;"
            "}");
        // 将标签放置在窗口中央
        label_->move(280, 12);

        // 创建设置区
        settingsGroup = new QGroupBox("操作区", window_);
        // 设置设置区样式表（美化）
        settingsGroup->setStyleSheet(
            "QGroupBox {"
            "    background-color: rgb(240, 240, 240);"  // 设置背景颜色
            "    border: 2px solid rgb(128, 128, 128);"  // 设置边框宽度和颜色
            "    border-radius: 0px;"  // 设置边框圆角
            "    margin-top: 1ex;"  // 设置标题与边框的间距
            "}"
            "QGroupBox::title {"
            "    subcontrol-origin: margin;"
            "    left: 10px;"  // 设置标题左偏移
            "    padding: 0 3px;"
            "    font-size: 30px;"  // 设置标题字体大小
            "    font-weight: bold;"  // 设置标题字体加粗
            "    color: rgb(64, 64, 64);"  // 设置标题字体颜色
            "}"
        );
        settingsLayout = new QVBoxLayout(settingsGroup);
        // 添加五个按钮
        for (int i = 0; i < 5; ++i) {
            QPushButton *buttonq;
            auto msg = ui_control_msg::msg::UiControl();
            msg.control_flag = i;
            // buttonq->setEnabled(true);
            if(i == 0){
                buttonq = new QPushButton(QString("从端机器人初始化"), settingsGroup);
            }
            else if(i == 1){
                buttonq = new QPushButton(QString("自动穿刺导航模式"), settingsGroup);
            }
            else if(i == 2){
                buttonq = new QPushButton(QString("开启系统遥操作"), settingsGroup);
            }
            else if(i == 3){
                buttonq = new QPushButton(QString("停止系统遥操作"), settingsGroup);
            }
            else if(i == 4){
                buttonq = new QPushButton(QString("退出系统"), settingsGroup);
                // 连接退出系统按钮的信号和槽
                QObject::connect(buttonq, &QPushButton::clicked, [this, msg]() {
                    // 发布退出系统消息
                    ui_control_publisher_->publish(msg);
                    RCLCPP_INFO(this->get_logger(), "UI published control command: %d", msg.control_flag);
                    // 关闭UI
                    if (ros_spin_thread_.joinable()) {
                        rclcpp::shutdown();
                        ros_spin_thread_.join();
                    }
                    qt_app_->quit();
                });
            }
            // 为按钮连接点击信号和发布消息的槽函数
            QObject::connect(buttonq, &QPushButton::clicked, [this, msg]() {
                ui_control_publisher_->publish(msg);
                RCLCPP_INFO(this->get_logger(), "UI published control command: %d", msg.control_flag);
                // 改变机器人电源状态显示
                if (msg.control_flag == 0){
                    powerStatusLabel->setText(QString("电源状态: 上电"));
                }
                // 改变机器人运行状态显示
                if (msg.control_flag == 2){
                    if(!start_flag){
                        currentModeLabel->setText(QString("当前模式: 体外作业阶段"));
                        start_flag = true;
                    }
                    else if (mode_flag==0) currentModeLabel->setText(QString("当前模式: 体外作业阶段")); 
                    else if (mode_flag==1) currentModeLabel->setText(QString("当前模式: 通道穿刺阶段"));
                    else if (mode_flag==2) currentModeLabel->setText(QString("当前模式: 颅内作业阶段"));
                }
                if (msg.control_flag == 3){
                    currentModeLabel->setText(QString("当前模式: 遥操作暂停"));
                }
            });
            // 设置按钮样式表
            buttonq->setStyleSheet(
                "QPushButton {"
                "    background-color: rgb(78, 109, 148);"
                "    color: white;"
                "    border: 1px solid rgb(78, 109, 148);"
                "    border-radius: 5px;"
                "    padding: 8px 16px;"
                "    font-size: 18px;"
                "}"
                "QPushButton:hover {"
                "    background-color: rgb(40, 62, 101);"
                "}"
                "QPushButton:pressed {"
                "    background-color: rgb(38, 46, 59);"
                "}"
            );
            settingsLayout->addWidget(buttonq);
            buttonq->setFixedSize(275, 40); // 设置按钮大小

        }
        settingsGroup->setLayout(settingsLayout);
        // 设置设置区位置
        int setting_x = 940;
        int setting_y = 155+540+20;
        int setting_width = 300;
        int setting_hight = 340;
        settingsGroup->move(setting_x, setting_y); // 设置设置区的位置
        settingsGroup->resize(setting_width, setting_hight); // 设置设置区的大小
        settingsGroup->show();
        

        // 创建功能区
        functionGroup = new QGroupBox("映射模式选择", window_);
        // 设置功能区样式表（美化），可复用设置区样式
        functionGroup->setStyleSheet(
            "QGroupBox {"
            "    background-color: rgb(240, 240, 240);"
            "    border: 2px solid rgb(128, 128, 128);"
            "    border-radius: 0px;"
            "    margin-top: 1ex;"
            "}"
            "QGroupBox::title {"
            "    subcontrol-origin: margin;"
            "    left: 10px;"
            "    padding: 0 3px;"
            "    font-size: 30px;"
            "    font-weight: bold;"
            "    color: rgb(64, 64, 64);"
            "}"
        );
        functionLayout = new QVBoxLayout(functionGroup);
        // 添加三个按钮
        for (int i = 0; i < 3; ++i) {
            QPushButton *button;
            auto msg = ui_control_msg::msg::UiControl();
            msg.teleoperation_mode = i;
            if (i==0){
                button = new QPushButton(QString("体外作业阶段"), functionGroup);
            }
            if (i==1){
                button = new QPushButton(QString("通道穿刺阶段"), functionGroup);
            }
            if (i==2){
                button = new QPushButton(QString("颅内作业阶段"), functionGroup);
            }
            // 为按钮连接点击信号和发布消息的槽函数
            QObject::connect(button, &QPushButton::clicked, [this, msg]() {
                ui_mode_publisher_->publish(msg);
                RCLCPP_INFO(this->get_logger(), "UI published mode command: %d", msg.teleoperation_mode);
                // 改变机器人运行状态显示
                if (msg.teleoperation_mode == 0){
                    mode_flag = 0;
                    if (start_flag) currentModeLabel->setText(QString("当前模式: 体外作业阶段"));
                }
                if (msg.teleoperation_mode == 1){
                    mode_flag = 1;
                    if (start_flag) currentModeLabel->setText(QString("当前模式: 通道穿刺阶段"));
                }
                if (msg.teleoperation_mode == 2){
                    mode_flag = 2;
                    if (start_flag) currentModeLabel->setText(QString("当前模式: 颅内作业阶段"));
                }
            });
            // 设置按钮样式表，可复用设置区按钮样式
            button->setStyleSheet(
                "QPushButton {"
                "    background-color: rgb(109, 119, 150);"
                "    color: white;"
                "    border: 1px solid rgb(109, 119, 150);"
                "    border-radius: 5px;"
                "    padding: 8px 16px;"
                "    font-size: 18px;"
                "}"
                "QPushButton:hover {"
                "    background-color: rgb(46, 64, 99);"
                "}"
                "QPushButton:pressed {"
                "    background-color: rgb(16, 36, 68);"
                "}"
            );
            functionLayout->addWidget(button);
            button->setFixedSize(275, 60); // 设置按钮大小
        }
        functionGroup->setLayout(functionLayout);
        // 调整功能区位置和大小
        int function_x = setting_x + setting_width + 28;
        int function_y = setting_y;
        int function_width = setting_width;
        int function_hight = setting_hight;
        // int settingsBottom = 150 + 480 + 20 + 400; // 设置区底部的 y 坐标
        // functionGroup->move(1240 + 320, settingsBottom - 190); // 使功能区右侧与工业摄像头右侧对齐，底部与设置区底部对齐
        // functionGroup->resize(320, 190); // 宽度与工业摄像头一致，降低高度
        int settingsBottom = 150 + 480 + 20 + 400; // 设置区底部的 y 坐标
        functionGroup->move(function_x, function_y); // 使功能区右侧与工业摄像头右侧对齐，底部与设置区底部对齐
        functionGroup->resize(function_width, function_hight); // 宽度与工业摄像头一致，降低高度
        functionGroup->show();


        // 创建用于显示从端机器人末端位置信息的区域
        robotPositionGroup = new QGroupBox("实时信息", window_);
        robotPositionGroup->setStyleSheet(
            "QGroupBox {"
            "    background-color: rgb(240, 240, 240);"
            "    border: 2px solid rgb(128, 128, 128);"
            "    border-radius: 0px;"
            "    margin-top: 1ex;"
            "}"
            "QGroupBox::title {"
            "    subcontrol-origin: margin;"
            "    left: 10px;"
            "    padding: 0 3px;"
            "    font-size: 30px;"
            "    font-weight: bold;"
            "    color: rgb(64, 64, 64);"
            "}"
        );
        positionLayout = new QVBoxLayout(robotPositionGroup);

        // 创建显示位置信息的标签
        bloodEditText = "最大可视血肿面积: ";
        powerStatusLabelText = "电源状态: 断电";
        currentModeLabelText = "当前模式: 非遥操作控制模式";
        positionLabelText = "当前位置: ";
        bloodLabel = new QLabel(bloodEditText, robotPositionGroup);
        powerStatusLabel = new QLabel(powerStatusLabelText, robotPositionGroup);
        currentModeLabel = new QLabel(currentModeLabelText, robotPositionGroup);
        positionLabel = new QLabel(positionLabelText, robotPositionGroup);

        xmate_positionLabel = new QLabel("       机械臂: ", robotPositionGroup);
        pf_positionLabel =    new QLabel("  内镜平台: ", robotPositionGroup);

        // 设置标签字体
        positionFont.setPointSize(16);
        positionFont.setBold(true); // 设置字体为粗体
        bloodLabel->setFont(positionFont);
        positionFont.setBold(false); // 设置字体为非粗体
        powerStatusLabel->setFont(positionFont);
        currentModeLabel->setFont(positionFont);
        positionLabel->setFont(positionFont);
        positionFont.setPointSize(12);
        xmate_positionLabel->setFont(positionFont);
        pf_positionLabel->setFont(positionFont);

        // 将标签添加到布局中
        positionLayout->addWidget(bloodLabel);
        positionLayout->addWidget(powerStatusLabel);
        positionLayout->addWidget(currentModeLabel);
        positionLayout->addWidget(positionLabel);
        positionLayout->addWidget(xmate_positionLabel);
        positionLayout->addWidget(pf_positionLabel);

        robotPositionGroup->setLayout(positionLayout);

        // 调整信息显示区的位置和大小
        int inf_x = function_x + function_width + 28;
        int inf_y = function_y;
        int inf_width = function_width;
        int inf_hight = function_hight;
        // robotPositionGroup->move(1240 + 320, settingsBottom - 400); // 放置在功能区上方
        // robotPositionGroup->resize(320, 190); // 设置大小
        robotPositionGroup->move(inf_x, inf_y); // 放置在功能区上方
        robotPositionGroup->resize(inf_width, inf_hight); // 设置大小
        robotPositionGroup->show();

        
        // 创建用于显示内窥镜视频的 QLabel
        videoLabel_in = new QLabel(window_);
        videoLabel_in->move(20, 150); // 设置位置
        videoLabel_in->resize(1600, 900); // 设置大小
        videoLabel_in->show();

        // 创建用于显示工业摄像头视频的 QLabel
        videoLabel_out = new QLabel(window_);
        // videoLabel_out->move(1240, 150); // 设置位置
        // videoLabel_out->resize(640, 480); // 设置大小
        videoLabel_out->move(940, 155); // 设置位置
        videoLabel_out->resize(960, 540); // 设置大小-改
        videoLabel_out->show();

        // 将设置区和功能区置于顶层
        settingsGroup->raise();
        functionGroup->raise();
        robotPositionGroup->raise();

        // 创建内窥镜画面说明标签
        inCameraLabel = new QLabel("颅内画面", window_);
        labelFont.setPointSize(20); // 设置字体大小
        labelFont.setBold(true);    // 设置字体为粗体
        inCameraLabel->setFont(labelFont);
        inCameraLabel->setStyleSheet(
            "QLabel {"
            "    background-color: rgba(255, 255, 255, 150);" // 半透明白色背景
            "    color: black;"
            "    padding: 5px;"
            "    border-radius: 5px;"
            "}"
        );
        inCameraLabel->move(30, 165); // 放置在内窥镜画面左上方
        inCameraLabel->show();

        // 创建工业摄像头画面说明标签
        outCameraLabel = new QLabel("体外画面", window_);
        outCameraLabel->setFont(labelFont);
        outCameraLabel->setStyleSheet(
            "QLabel {"
            "    background-color: rgba(255, 255, 255, 150);" // 半透明白色背景
            "    color: black;"
            "    padding: 5px;"
            "    border-radius: 5px;"
            "}"
        );
        outCameraLabel->move(950, 165); // 放置在工业摄像头画面左上方
        outCameraLabel->show();

        // 以全屏模式显示窗口
        window_->showFullScreen();

        // 启动键盘读取线程
        keyboard_thread_ = std::thread(&UINode::keyboard_read_thread, this);

        // 初始化订阅者对象（消息类型、话题名、订阅者回调函数、队列长度）
        video_subscriber = this->create_subscription<sensor_msgs::msg::Image>(
            "image_detected", 5, std::bind(&UINode::video_listener_callback, this, std::placeholders::_1));
        
        // 初始化订阅者对象，用于接收工业摄像头图像消息
        usb_video_subscriber = this->create_subscription<sensor_msgs::msg::Image>(
            "camera2/image_raw", 5, std::bind(&UINode::usb_video_listener_callback, this, std::placeholders::_1));

        // 初始化订阅者对象，用于接收血肿面积
        area_subscriber = this->create_subscription<std_msgs::msg::Float64>(
            "area", 5, std::bind(&UINode::area_listener_callback, this, std::placeholders::_1));

        // 初始化订阅者对象，用于接收从端机器人末端位置信息
        slaver_position_subscriber = this->create_subscription<ui_control_msg::msg::SlaverPosition>(
            "slave_position_topic", 5, std::bind(&UINode::slaver_position_listener_callback, this, std::placeholders::_1));
        
        // 初始化发布者，话题名为 "ui_control_topic"，队列长度为 10
        ui_control_publisher_ = this->create_publisher<ui_control_msg::msg::UiControl>("ui_control_topic", 10);

        // 初始化发布者，话题名为 "ui_mode_topic"，队列长度为 10
        ui_mode_publisher_ = this->create_publisher<ui_control_msg::msg::UiControl>("ui_mode_topic", 10);

        // 初始化发布者，话题名为"ui_record_topic"，队列长度为 10
        ui_record_publisher_ = this->create_publisher<ui_control_msg::msg::UiControl>("ui_record_topic", 10);

    }

    ~UINode()
    {
        if (ros_spin_thread_.joinable()) {
            ros_spin_thread_.join();
        }
        if (label_) {
            delete label_;
            label_ = nullptr;
        }
        if (logoLabel_) {
            delete logoLabel_;
            logoLabel_ = nullptr;
        }
        if (window_) {
            delete window_;
            window_ = nullptr;
        }
        if (qt_app_) {
            delete qt_app_;
            qt_app_ = nullptr;
        }
    }

    int run()
    {
        // 启动 ROS 2 自旋线程
        ros_spin_thread_ = std::thread([this]() {
            rclcpp::spin(this->shared_from_this());
        });
        // 启动 Qt 应用程序
        return qt_app_->exec();
    }


private:
    CustomQApplication *qt_app_; //qt应用
    FullScreenWidget *window_; //窗口
    QLabel *label_;     //显示标题
    QLabel *logoLabel_; //显示 logo
    QLabel *videoLabel_in; // 用于显示内窥镜视频的 QLabel
    QLabel *videoLabel_out; // 用于显示客观视角视频的 QLabel
    QLabel *inCameraLabel; // 内窥镜画面说明标签
    QLabel *outCameraLabel; // 工业摄像头画面说明标签
    QFont labelFont;        // 字体对象
    QFont positionFont;     // 字体对象

    QGroupBox *settingsGroup; // 设置区
    QVBoxLayout *settingsLayout; // 设置区布局

    QGroupBox *functionGroup; // 功能区
    QVBoxLayout *functionLayout; // 功能区布局

    QGroupBox *robotPositionGroup; // 用于显示从端机器人末端位置信息的区域
    QVBoxLayout *positionLayout; // 用于布局位置信息的布局

    QLabel *bloodLabel; // 显示 x 坐标的标签
    QLabel *powerStatusLabel;   // 显示是否上电
    QLabel *currentModeLabel;   // 显示当前模式
    QLabel *positionLabel; // 显示当前位置
    QLabel *xmate_positionLabel; // 显示从端机器人的机械臂末端位置
    QLabel *pf_positionLabel; // 显示从端机器人的内镜运动平台末端位置

    QString bloodEditText; // bloodEdit标签内的内容
    QString powerStatusLabelText;   // powerStatusLabel标签内的内容
    QString currentModeLabelText;   // currentModeLabel标签内的内容
    QString positionLabelText;      // positionLabel标签内的内容

    cv_bridge::CvImagePtr cv_ptr;// 用于转换 ROS 图像消息到 OpenCV 图像的指针
    cv::Mat image; // 用于存储 OpenCV 图像的 Mat 对象

    // ROS2 线程(用于接收图像消息)
    std::thread ros_spin_thread_;
    // 键盘读取线程(用于操作时接收键盘输入)
    std::thread keyboard_thread_;

    // 订阅者对象，订阅内窥镜视频消息
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr video_subscriber;
    // 订阅者对象，订阅工业摄像头视频消息
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr usb_video_subscriber;
    // 订阅者对象，订阅血肿面积
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr area_subscriber;
    // 操作区按钮指令发布者对象
    rclcpp::Publisher<ui_control_msg::msg::UiControl>::SharedPtr ui_control_publisher_;
    // 要操作模式按钮指令发布者对象
    rclcpp::Publisher<ui_control_msg::msg::UiControl>::SharedPtr ui_mode_publisher_;
    // 录制指令发布者对象
    rclcpp::Publisher<ui_control_msg::msg::UiControl>::SharedPtr ui_record_publisher_;
    // 订阅者对象，订阅从端机器人位置消息
    rclcpp::Subscription<ui_control_msg::msg::SlaverPosition>::SharedPtr slaver_position_subscriber;

    int area_refresh_count = 10;
    int slaver_position_refresh_count = 100;

    bool start_flag = false;
    int mode_flag = 0;
 
    // 键盘读取线程函数
    void keyboard_read_thread()
    {
        while (rclcpp::ok()) {
            // 检查是否有键盘输入
            if (keyboard_input == "z"){
                auto msg = ui_control_msg::msg::UiControl();
                msg.teleoperation_mode = 3;
                ui_mode_publisher_->publish(msg);
                RCLCPP_INFO(this->get_logger(), "(KeyBoard)UI published mode command: %d", msg.teleoperation_mode);
                keyboard_input = " ";
                if (start_flag) currentModeLabel->setText(QString("当前模式: 颅内作业阶段"));
                mode_flag = 2;
            }
            else if (keyboard_input == "x"){
                auto msg = ui_control_msg::msg::UiControl();
                msg.teleoperation_mode = 2;
                ui_mode_publisher_->publish(msg);
                RCLCPP_INFO(this->get_logger(), "(KeyBoard)UI published mode command: %d", msg.teleoperation_mode);
                keyboard_input = " ";
                if (start_flag) currentModeLabel->setText(QString("当前模式: 颅内作业阶段"));
                mode_flag = 2;
            }
            else if (keyboard_input == "1"){
                auto msg = ui_control_msg::msg::UiControl();
                msg.teleoperation_mode = 0;
                ui_mode_publisher_->publish(msg);
                RCLCPP_INFO(this->get_logger(), "(KeyBoard)UI published mode command: %d", msg.teleoperation_mode);
                keyboard_input = " ";
                if (start_flag) currentModeLabel->setText(QString("当前模式: 体外作业阶段"));
                mode_flag = 0;
            }
            else if (keyboard_input == "2"){
                auto msg = ui_control_msg::msg::UiControl();
                msg.teleoperation_mode = 1;
                ui_mode_publisher_->publish(msg);
                RCLCPP_INFO(this->get_logger(), "(KeyBoard)UI published mode command: %d", msg.teleoperation_mode);
                keyboard_input = " ";
                if (start_flag) currentModeLabel->setText(QString("当前模式: 通道穿刺阶段"));
                mode_flag = 1;
            }
            else if (keyboard_input == "3"){
                auto msg = ui_control_msg::msg::UiControl();
                msg.teleoperation_mode = 2;
                ui_mode_publisher_->publish(msg);
                RCLCPP_INFO(this->get_logger(), "(KeyBoard)UI published mode command: %d", msg.teleoperation_mode);
                keyboard_input = " ";
                if (start_flag) currentModeLabel->setText(QString("当前模式: 颅内作业阶段"));
                mode_flag = 2;
            }
            else if (keyboard_input == "s"){
                auto msg = ui_control_msg::msg::UiControl();
                msg.record_flag = 1;
                ui_record_publisher_->publish(msg);
                RCLCPP_INFO(this->get_logger(), "(KeyBoard)UI published record command: %d", msg.record_flag);
                keyboard_input = " ";
            }
                
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 控制循环频率
        }
    }

    // 内窥镜视频回调函数
    void video_listener_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // // 调试用
        // RCLCPP_INFO(this->get_logger(), "Received an image");
        // 将 ROS 的图像消息转化成 OpenCV 图像
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        image = cv_ptr->image;
        // 将 OpenCV 图像转换为 QImage
        QImage qImage(image.data, image.cols, image.rows, image.step, QImage::Format_BGR888);
        QPixmap pixmap = QPixmap::fromImage(qImage);
        // 在 QLabel 中显示图像
        if (videoLabel_in) {
            videoLabel_in->setPixmap(pixmap.scaled(videoLabel_in->size(), Qt::KeepAspectRatio));
        }
    }

    // 内窥镜视频回调函数
    void usb_video_listener_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // // 调试用
        // RCLCPP_INFO(this->get_logger(), "Received an image");
        // 将 ROS 的图像消息转化成 OpenCV 图像
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        image = cv_ptr->image;
        // 将 OpenCV 图像转换为 QImage
        QImage qImage(image.data, image.cols, image.rows, image.step, QImage::Format_BGR888);
        QPixmap pixmap = QPixmap::fromImage(qImage);
        // 在 QLabel 中显示图像
        if (videoLabel_out) {
            videoLabel_out->setPixmap(pixmap.scaled(videoLabel_out->size(), Qt::KeepAspectRatio));
        }
    }
    
    // 血肿面积回调函数
    void area_listener_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        if (area_refresh_count > 0) {
            area_refresh_count--;
            return;
        }
        else {
            area_refresh_count = 10;
            double area = msg->data; // 获取 area 的值
            area = area / 80000.0 * 10; // 单位转换为 mm²
            bloodLabel->setText(QString("最大可视血肿面积: %1 mm²").arg(QString::number(area, 'f', 3)));
        }
    }
    // 从端机器人位置回调函数
    void slaver_position_listener_callback(const ui_control_msg::msg::SlaverPosition::SharedPtr msg)
    {
        if (slaver_position_refresh_count > 0) {
            slaver_position_refresh_count--;
            return;
        }
        else {
            slaver_position_refresh_count = 100;
            double xmate_x = msg->xmate_position[0]*1000;
            double xmate_y = msg->xmate_position[1]*1000;
            double xmate_z = msg->xmate_position[2]*1000;
            // 使用 QString::number 格式化数值，保留两位小数
            xmate_positionLabel->setText(QString("       机械臂:(%1, %2, %3)")
                .arg(QString::number(xmate_x, 'f', 2))
                .arg(QString::number(xmate_y, 'f', 2))
                .arg(QString::number(xmate_z, 'f', 2)));
            double pf_x = msg->pf_position[0];
            double pf_y = msg->pf_position[1];
            double pf_z = msg->pf_position[2];
            // 使用 QString::number 格式化数值，保留两位小数
            pf_positionLabel->setText(QString("  内镜平台:(%1, %2, %3)")
                .arg(QString::number(pf_x, 'f', 2))
                .arg(QString::number(pf_y, 'f', 2))
                .arg(QString::number(pf_z, 'f', 2)));
        }
    }
};


int main(int argc, char **argv)
{
    // cpu_set_t cpuset;
    // CPU_ZERO(&cpuset);
    // // 将任务绑定到 CPU 核心 0
    // CPU_SET(3, &cpuset); 

    rclcpp::init(argc, argv);
    auto node = std::make_shared<UINode>();

    try
    {
        int result = node->run();
        rclcpp::shutdown();
        return result;
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
}
