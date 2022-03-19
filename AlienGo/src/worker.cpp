#include "worker.h"

void TWorker::run()
{
    ros::MultiThreadedSpinner spinner2(2);
    spinner2.spin(&images_queue);
}

void TWorker::cbProcessTopCameraFrame(const sensor_msgs::ImageConstPtr& ros_image)
{
    QImage qt_image;
    if(ros_image->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
        convert_depth_to_color(ros_image, qt_image, 500, 30000);
    }
    else {
        qt_image = QImage(&ros_image->data[0], ros_image->width, ros_image->height, ros_image->step, QImage::Format_RGB888);
    }
    QPixmap qt_pixmap = QPixmap::fromImage(qt_image);
    emit topCameraFrameReadyToShow(qt_pixmap);
}

void TWorker::cbProcessBottomCameraFrame(const sensor_msgs::ImageConstPtr &ros_image)
{
    QImage qt_image;
    if(ros_image->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
        convert_depth_to_color(ros_image, qt_image, 500, 30000);
    }
    else {
        qt_image = QImage(&ros_image->data[0], ros_image->width, ros_image->height, ros_image->step, QImage::Format_RGB888);
    }
    QPixmap qt_pixmap = QPixmap::fromImage(qt_image);
    emit bottomCameraFrameReadyToShow(qt_pixmap);
}

void TWorker::convert_depth_to_color(const sensor_msgs::ImageConstPtr &msg, QImage& qt_image, uint16_t min_depth, uint16_t max_depth)
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

ros::CallbackQueue* TWorker::getCallbackQueue() {
    return &images_queue;
}