#include "aliengo_gui.h"
#include "ui_main_window.h"

namespace Aliengo {
    MainWindow::MainWindow(QWidget *parent) :
            QMainWindow(parent), ui(new Ui::MainWindow)
    {
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
        FGrid = FManager->createDisplay("rviz/Grid", "adjustable grid", true);

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
        else if(button_id == -3 && FTopCameraState != CameraState::RGB) {
            // Stop last image subscriber
            FTopCameraImageSub->StopSubscriber();
            // Init new image subscriber
            FTopCameraImageSub = new ros_qt_interface::TRosQtSensorMsgsImageSub("/top_camera/color/image_raw");
            connect(FTopCameraImageSub, SIGNAL(DataReceived()), this, SLOT(SLOT_ROS_NewTopCameraImage()));
            // Clear label
            ui->TopCameraImage->clear();
            FTopCameraState = CameraState::RGB;
        }
        else if(button_id == -4 && FTopCameraState != CameraState::Depth) {
            // Stop last image subscriber
            FTopCameraImageSub->StopSubscriber();
            // Init new image subscriber
            FTopCameraImageSub = new ros_qt_interface::TRosQtSensorMsgsImageSub("/top_camera/depth/image_rect_raw"); //TODO: Subscribe Depth
            connect(FTopCameraImageSub, SIGNAL(DataReceived()), this, SLOT(SLOT_ROS_NewTopCameraDepthImage()));
            // Clear label
            ui->TopCameraImage->clear();
            FTopCameraState = CameraState::Depth;
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
        else if(button_id == -3 && FBottomCameraState != CameraState::RGB) {
            // Init image subscriber
            FBottomCameraImageSub = new ros_qt_interface::TRosQtSensorMsgsImageSub("/camera/color/image_raw");
            connect(FTopCameraImageSub, SIGNAL(DataReceived()), this, SLOT(SLOT_ROS_NewBottomCameraImage()));
            // Clear label
            ui->BottomCameraImage->clear();
            FBottomCameraState = CameraState::RGB;
        }
        else if(button_id == -4 && FBottomCameraState != CameraState::Depth) {
            // Init image subscriber
            FBottomCameraImageSub = new ros_qt_interface::TRosQtSensorMsgsImageSub("/camera/depth/image_rect_raw");
            connect(FTopCameraImageSub, SIGNAL(DataReceived()), this, SLOT(SLOT_ROS_NewBottomCameraImage()));
            // Clear label
            ui->BottomCameraImage->clear();
            FBottomCameraState = CameraState::Depth;
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
        ROS_INFO("Recebi frame da câmara topo!");
        sensor_msgs::Image image;
        FTopCameraImageSub->GetData(image);
        QImage converted_image(&image.data[0], image.width, image.height, image.step, QImage::Format_RGB888);
        ui->TopCameraImage->setStyleSheet("");
        QPixmap pix_image = QPixmap::fromImage(converted_image);
        ui->TopCameraImage->setPixmap(pix_image);
    }

    void MainWindow::SLOT_ROS_NewTopCameraDepthImage()
    {
        ROS_INFO("Recebi depth frame da câmara topo!");
        sensor_msgs::Image image;
        FTopCameraImageSub->GetData(image);
        QImage qt_image;
        convert_depth_to_color(image, qt_image);
        ui->TopCameraImage->setStyleSheet("");
        QPixmap pix_image = QPixmap::fromImage(qt_image);
        ui->TopCameraImage->setPixmap(pix_image);
    }

    void MainWindow::SLOT_ROS_NewBottomCameraImage()
    {
        ROS_INFO("Recebi frame da câmara de baixo!");
        sensor_msgs::Image image;
        FBottomCameraImageSub->GetData(image);
        QImage converted_image(&image.data[0], image.width, image.height, image.step, QImage::Format_RGB888);
        ui->BottomCameraImage->setStyleSheet("");
        QPixmap pix_image = QPixmap::fromImage(converted_image);
        ui->BottomCameraImage->setPixmap(pix_image);
    }

    void MainWindow::convert_depth_to_color(const sensor_msgs::Image &msg, QImage& qt_image) {
        uint16_t min_depth = 0;
        uint16_t max_depth = 65535;
        uint16_t d_normal;
        uint8_t pixel_r, pixel_g, pixel_b;

        // Create QImage
        qt_image = QImage(msg.width, msg.height, QImage::Format_RGB888);

        // Convert ROS image to OpenCV
        cv_bridge::CvImagePtr cv_depth = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);

        for(int r=0; r < cv_depth->image.rows; r++) {
            uint16_t* ptr = cv_depth->image.ptr<uint16_t>(r);
            for(int c=0; c < cv_depth->image.cols; c++) {
                if(ptr[c] > max_depth) ptr[c] = max_depth;
                if(ptr[c] < min_depth) ptr[c] = min_depth;
                d_normal = (ptr[c] - min_depth)/(max_depth - min_depth)*1529;
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
                qt_image.setPixelColor(r, c, qRgb(pixel_r, pixel_g, pixel_b));
            }
        }
    }
}
