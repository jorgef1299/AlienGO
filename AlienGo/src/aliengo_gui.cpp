#include "aliengo_gui.h"
#include "ui_main_window.h"

namespace Aliengo {
    MainWindow::MainWindow(QWidget *parent) :
            QMainWindow(parent), ui(new Ui::MainWindow)
    {
        // Load ROS Parameters
        ros::param::get("/top_camera/rgb_image_topic", FTopCameraRGBTopicName);
        ros::param::get("/top_camera/depth_image_topic", FTopCameraDepthTopicName);
        ros::param::get("/top_camera/colorize_min_depth", FTopCameraMinDepth);
        ros::param::get("/top_camera/colorize_max_depth", FTopCameraMaxDepth);
        ros::param::get("/bottom_camera/rgb_image_topic", FBottomCameraRGBTopicName);
        ros::param::get("/bottom_camera/depth_image_topic", FBottomCameraDepthTopicName);
        ros::param::get("/bottom_camera/colorize_min_depth", FBottomCameraMinDepth);
        ros::param::get("/bottom_camera/colorize_max_depth", FBottomCameraMaxDepth);
        ros::param::get("/map/3d_topic", FMap3DTopicName);
        ros::param::get("/map/2d_topic", FMap2DTopicName);
        ros::param::get("/frame_id", FFrameId);

        // Load UI
        ui->setupUi(this);

        // Group Radio buttons in groups
        QButtonGroup* button_group_top_camera = new QButtonGroup();
        button_group_top_camera->addButton(ui->radioButton_Top_Camera_1);
        button_group_top_camera->addButton(ui->radioButton_Top_Camera_2);
        button_group_top_camera->addButton(ui->radioButton_Top_Camera_3);
        QButtonGroup* button_group_bottom_camera = new QButtonGroup();
        button_group_bottom_camera->addButton(ui->radioButton_Bottom_Camera_1);
        button_group_bottom_camera->addButton(ui->radioButton_Bottom_Camera_2);
        button_group_bottom_camera->addButton(ui->radioButton_Bottom_Camera_3);
        QButtonGroup* button_group_map = new QButtonGroup();
        button_group_map->addButton(ui->radioButton_Map_1);
        button_group_map->addButton(ui->radioButton_Map_2);
        button_group_map->addButton(ui->radioButton_Map_3);

        QVBoxLayout* new_vbox = new QVBoxLayout();
        FRender_panel = new rviz::RenderPanel();
        new_vbox->addWidget(FRender_panel);
        ui->GroupBox_Map->layout()->addWidget(FRender_panel);

        FManager = new rviz::VisualizationManager(FRender_panel);
        FRender_panel->initialize(FManager->getSceneManager(), FManager);
        FManager->initialize();
        FManager->startUpdate();
        FManager->setFixedFrame(FFrameId.c_str());
        FDisplayGrid = FManager->createDisplay("rviz/Grid", "adjustable grid", true);
        FDisplayPointCloud = FManager->createDisplay("rviz/PointCloud2", "map_point_cloud", true);
        FDisplayPointCloud->subProp("Topic")->setValue(FMap3DTopicName.c_str());
        FDisplayPointCloud->subProp("Style")->setValue("Points");
        FDisplayPointCloud->subProp("Size (Pixels)")->setValue("2");
        FDisplayPointCloud->subProp("Decay Time")->setValue("1");
        FDisplayPointCloud->subProp("Color Transformer")->setValue("Intensity");

        connect(button_group_top_camera, SIGNAL(buttonPressed(int)), this, SLOT(RadioButtonTopCameraPressed(int)));
        connect(button_group_bottom_camera, SIGNAL(buttonPressed(int)), this, SLOT(RadioButtonBottomCameraPressed(int)));
        connect(button_group_map, SIGNAL(buttonPressed(int)), this, SLOT(RadioButtonMapPressed(int)));

        FTopCameraState = CameraState::Disabled;
        FBottomCameraState = CameraState::Disabled;
        FMapState = MapState::Disabled;
    }

    MainWindow::~MainWindow() {
        delete ui;
        delete FManager;
    }

    void MainWindow::RadioButtonTopCameraPressed(int button_id)
    {
        if(button_id == -2 && FTopCameraState != CameraState::Disabled) {
            // Stop image subscriber
            FTopCameraImageSub->StopSubscriber();
            // Remove pixmap
            ui->TopCameraImage->clear();
            // Set black background
            ui->TopCameraImage->setStyleSheet("background-color: black;");
            FTopCameraState = CameraState::Disabled;
        }
        else {
            // Stop last image subscriber
            if(FTopCameraState != CameraState::Disabled)
                FTopCameraImageSub->StopSubscriber();
            // Clear label
            ui->TopCameraImage->clear();
            if (button_id == -3 && FTopCameraState != CameraState::RGB) {
                FTopCameraImageSub = new ros_qt_interface::TRosQtSensorMsgsImageSub(FTopCameraRGBTopicName);
                FTopCameraState = CameraState::RGB;
            } else if (button_id == -4 && FTopCameraState != CameraState::Depth) {
                FTopCameraImageSub = new ros_qt_interface::TRosQtSensorMsgsImageSub(FTopCameraDepthTopicName);
                FTopCameraState = CameraState::Depth;
            }
            connect(FTopCameraImageSub, SIGNAL(DataReceived()), this, SLOT(SLOT_ROS_NewTopCameraImage()));
        }
    }

    void MainWindow::RadioButtonBottomCameraPressed(int button_id)
    {
        if(button_id == -2 && FBottomCameraState != CameraState::Disabled) {
            // Stop image subscriber
            FBottomCameraImageSub->StopSubscriber();
            // Remove pixmap
            ui->BottomCameraImage->clear();
            // Set black background
            ui->BottomCameraImage->setStyleSheet("background-color: black;");
            FBottomCameraState = CameraState::Disabled;
        }
        else {
            // Stop last image subscriber
            if(FBottomCameraState != CameraState::Disabled)
                FBottomCameraImageSub->StopSubscriber();
            // Clear label
            ui->BottomCameraImage->clear();
            if (button_id == -3 && FBottomCameraState != CameraState::RGB) {
                FBottomCameraImageSub = new ros_qt_interface::TRosQtSensorMsgsImageSub(FBottomCameraRGBTopicName);
                FBottomCameraState = CameraState::RGB;
            } else if (button_id == -4 && FBottomCameraState != CameraState::Depth) {
                FBottomCameraImageSub = new ros_qt_interface::TRosQtSensorMsgsImageSub(FBottomCameraDepthTopicName);
                FBottomCameraState = CameraState::Depth;
            }
            connect(FBottomCameraImageSub, SIGNAL(DataReceived()), this, SLOT(SLOT_ROS_NewBottomCameraImage()));
        }
    }

    void MainWindow::RadioButtonMapPressed(int button_id)
    {
        if(button_id == -2 && FMapState != MapState::Disabled) {
            // Stop map related subscribers
            //TODO: Add subscribers
            FMapState = MapState::Disabled;
        }
        else if(button_id == -3 && FMapState != MapState::Two_D) {
            FMapState = MapState::Two_D;
        }
        else if(button_id == -4 && FMapState != MapState::Three_D) {
            FMapState = MapState::Three_D;
        }
    }

    void MainWindow::SLOT_ROS_NewTopCameraImage()
    {
        sensor_msgs::Image image_msg;
        FTopCameraImageSub->GetData(image_msg);
        QImage qt_image;
        if(image_msg.encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
            convert_depth_to_color(image_msg, qt_image, 500, 30000);
        }
        else {
            qt_image = QImage(&image_msg.data[0], image_msg.width, image_msg.height, image_msg.step, QImage::Format_RGB888);
        }
        setPixmapImage(qt_image, ui->TopCameraImage);
    }

    void MainWindow::SLOT_ROS_NewBottomCameraImage()
    {
        sensor_msgs::Image image_msg;
        FBottomCameraImageSub->GetData(image_msg);
        QImage qt_image;
        if(image_msg.encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
            convert_depth_to_color(image_msg, qt_image, 500, 20000);
        }
        else {
            qt_image = QImage(&image_msg.data[0], image_msg.width, image_msg.height, image_msg.step, QImage::Format_RGB888);
        }
        setPixmapImage(qt_image, ui->BottomCameraImage);
    }

    /********************************* Auxiliar functions *************************************/
    void MainWindow::setPixmapImage(const QImage& image, QLabel* img_label)
    {
        // Create pixmap
        QPixmap pix_image = QPixmap::fromImage(image);
        // Set pixmap
        img_label->setPixmap(pix_image);
    }

    void MainWindow::convert_depth_to_color(const sensor_msgs::Image &msg, QImage& qt_image, uint16_t min_depth, uint16_t max_depth)
    {
        uint16_t d_normal;
        uint8_t pixel_r, pixel_g, pixel_b;
        // Convert ROS image to OpenCV
        cv_bridge::CvImagePtr cv_depth = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        // Create QImage
        qt_image = QImage(cv_depth->image.cols, cv_depth->image.rows, QImage::Format_RGB888);
        for(int r=0; r < cv_depth->image.rows; r++) {
            uint16_t* ptr = cv_depth->image.ptr<uint16_t>(r);
            for(int c=0; c < cv_depth->image.cols; c++) {
                if(ptr[c] > max_depth) ptr[c] = max_depth;
                if(ptr[c] < min_depth) ptr[c] = min_depth;
                d_normal = (float)(ptr[c] - min_depth)/(max_depth - min_depth)*1529;
                // Red component
                if(d_normal <= 255 || (d_normal > 1275 && d_normal <= 1529)) pixel_r = 255;
                else if(d_normal > 255 && d_normal <= 510) pixel_r = 255 - d_normal;
                else if(d_normal > 510 && d_normal <= 1020) pixel_r = 0;
                else if(d_normal > 1020 && d_normal <= 1275) pixel_r = d_normal - 1020;
                // Green component
                if(d_normal <= 255) pixel_g = d_normal;
                else if(d_normal <= 510) pixel_g = 255;
                else if(d_normal <= 765) pixel_g = 765 - d_normal;
                else pixel_g = 0;
                // Blue component
                if(d_normal <= 765) pixel_b = d_normal;
                else if(d_normal <= 1020) pixel_b = d_normal - 765;
                else if(d_normal <= 1275) pixel_b = 255;
                else pixel_b = 1529 - d_normal;
                qt_image.setPixelColor(c, r, qRgb(pixel_r, pixel_g, pixel_b));
            }
        }
    }
    /****************************** ROS Functions *****************************/
    void cbLidarData(const sensor_msgs::PointCloud2ConstPtr &points) {
        ROS_INFO("Entrei!");
    }
}
