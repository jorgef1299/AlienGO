#ifndef SRC_ALIENGO_GUI_H
#define SRC_ALIENGO_GUI_H

// QT includes
#include <QMainWindow>
#include <QWidget>
#include <QVBoxLayout>
#include <QLabel>
#include <QRadioButton>

// RVIZ includes
#include <rviz/visualization_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>

// ROS includes
#include <ros/ros.h>
#include "ros_qt_sensor_msgs_image.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>

namespace Ui {
    class MainWindow;
}

namespace Aliengo {

    enum class CameraState {
        Disabled,
        RGB,
        Depth
    };

    enum class MapState {
        Disabled,
        Two_D,
        Three_D
    };

    class MainWindow: public QMainWindow
    {
    Q_OBJECT
    public:
        MainWindow(QWidget* parent=0);
        virtual ~MainWindow();
    private Q_SLOTS:
        void RadioButtonTopCameraPressed(int button_id);
        void RadioButtonBottomCameraPressed(int button_id);
        void RadioButtonMapPressed(int button_id);
        void SLOT_ROS_NewTopCameraImage();
        void SLOT_ROS_NewBottomCameraImage();
    private:
        rviz::VisualizationManager* FManager;
        rviz::RenderPanel* FRender_panel;
        rviz::Display* FDisplayGrid;
        rviz::Display* FDisplayPointCloud;
        Ui::MainWindow *ui;
        CameraState FTopCameraState, FBottomCameraState;
        MapState FMapState;
        ros_qt_interface::TRosQtSensorMsgsImageSub* FTopCameraImageSub;
        ros_qt_interface::TRosQtSensorMsgsImageSub* FBottomCameraImageSub;
        void convert_depth_to_color(const sensor_msgs::Image &msg, QImage& qt_image, uint16_t min_depth, uint16_t max_depth);
        void setPixmapImage(const QImage& image, QLabel* img_label);
        ros::NodeHandle nh;
        ros::Subscriber sub_lidar_cloud;
        // ROS Parameters
        std::string FTopCameraRGBTopicName, FBottomCameraRGBTopicName;
        std::string FTopCameraDepthTopicName, FBottomCameraDepthTopicName;
        int FTopCameraMinDepth, FTopCameraMaxDepth;
        int FBottomCameraMinDepth, FBottomCameraMaxDepth;
        std::string FMap3DTopicName, FMap2DTopicName;
        std::string FFrameId;
        int FDisplayPointCloud2SizePixels, FDisplayPointCloud2DecayTime;
    };
}

#endif //SRC_ALIENGO_GUI_H
