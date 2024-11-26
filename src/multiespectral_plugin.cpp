#include "multiespectral_plugin.h"
#include <ros/master.h>

MultiespectralPlugin::MultiespectralPlugin(QWidget* parent)
    : rviz::Panel(parent), it(nh),
      ac_lwir("/MultiespectralAcquire_lwir", true),
      ac_rgb("/MultiespectralAcquire_visible", true),
      lwir_frame_count(0), rgb_frame_count(0)
{
    // Leer los parámetros desde ROS
    nh.param<std::string>("lwir_action_name", action_name, "/MultiespectralAcquire_lwir");
    nh.param<std::string>("rgb_action_name", action_name, "/MultiespectralAcquire_visible");
    nh.param<std::string>("lwir_topic", lwir_topic, "/lwir_image");
    nh.param<std::string>("rgb_topic", rgb_topic, "/visible_image");

    // Inicializar los controles de configuración
    layout = new QVBoxLayout;
    controls_layout = new QHBoxLayout;
    info_layout = new QVBoxLayout;
    images_layout = new QHBoxLayout;

    start_button = new QPushButton("Start Action", this);
    connect(start_button, &QPushButton::clicked, this, &MultiespectralPlugin::sendGoal);
    controls_layout->addWidget(start_button);

    cancel_button = new QPushButton("Cancel Action", this);
    connect(cancel_button, &QPushButton::clicked, this, &MultiespectralPlugin::cancelGoal);
    controls_layout->addWidget(cancel_button);

    // start_button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
    // cancel_button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);

    info_layout->addLayout(controls_layout);

    frequency_label = new QLabel(this);
    frequency_label->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
    info_layout->addWidget(frequency_label);

    num_images_label = new QLabel(this);
    num_images_label->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);
    info_layout->addWidget(num_images_label);

    layout->addLayout(info_layout);

    // Agregar un espaciador flexible
    // layout->addItem(new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding));

    lwir_label = new QLabel(this);
    lwir_label->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    images_layout->addWidget(lwir_label);

    rgb_label = new QLabel(this);
    rgb_label->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    images_layout->addWidget(rgb_label);

    layout->addLayout(images_layout);

    setLayout(layout);

    // Inicializar suscriptores de topics
    lwir_sub = it.subscribe(lwir_topic, 1, &MultiespectralPlugin::imageCallbackLWIR, this);
    rgb_sub = it.subscribe(rgb_topic, 1, &MultiespectralPlugin::imageCallbackRGB, this);

    initializeImages();

    // Configurar el temporizador para actualizar las imágenes y calcular la frecuencia
    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &MultiespectralPlugin::updateImages);
    timer->start(1000 / 1); // Actualización a 30 FPS
    frame_timer.start();
}


void MultiespectralPlugin::sendGoal()
{
    multiespectral_fb::MultiespectralAcquisitionGoal goal;
    ac_lwir.sendGoal(goal,
                     boost::bind(&MultiespectralPlugin::doneCb, this, _1, _2),
                     boost::bind(&MultiespectralPlugin::activeCb, this),
                     boost::bind(&MultiespectralPlugin::feedbackCb, this, _1));

    ac_rgb.sendGoal(goal,
                    boost::bind(&MultiespectralPlugin::doneCb, this, _1, _2),
                    boost::bind(&MultiespectralPlugin::activeCb, this),
                    boost::bind(&MultiespectralPlugin::feedbackCb, this, _1));

    ROS_INFO("Goal sent to both AC.");
}

void MultiespectralPlugin::cancelGoal()
{
    ac_lwir.cancelGoal();
    ac_rgb.cancelGoal();
    ROS_INFO("Goal cancelled to both AC.");
}


void handleImageCallback(const sensor_msgs::ImageConstPtr& msg, QImage& image)
{
    try
    {
        // Manejar la imagen según su codificación
        cv_bridge::CvImagePtr cv_ptr;
        if (msg->encoding == sensor_msgs::image_encodings::BGR8)
        {
            // Verificar si la imagen está mal formada
            if (msg->step != msg->width * 3)
            {
                ROS_ERROR("Image is wrongly formed: step (%d) != calculated step (%d).", msg->step, msg->width * 3);
                return;
            }
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        else if (msg->encoding == sensor_msgs::image_encodings::RGB8)
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
            cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_RGB2BGR);
        }
        else if (msg->encoding == sensor_msgs::image_encodings::MONO8)
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
            cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_GRAY2BGR);
        }
        else
        {
            ROS_ERROR("Unsupported encoding: %s", msg->encoding.c_str());
            return;
        }

        // Convertir cv::Mat a QImage
        image = QImage(cv_ptr->image.data, cv_ptr->image.cols, cv_ptr->image.rows, QImage::Format_RGB888).rgbSwapped();
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void MultiespectralPlugin::imageCallbackLWIR(const sensor_msgs::ImageConstPtr &msg)
{
    lwir_frame_count++;
    lwir_total_frame_count++;
    handleImageCallback(msg, img_lwir);
}

void MultiespectralPlugin::imageCallbackRGB(const sensor_msgs::ImageConstPtr &msg)
{
    rgb_frame_count++;
    rgb_total_frame_count++;
    handleImageCallback(msg, img_rgb);
}



void MultiespectralPlugin::initializeImages()
{
    img_lwir = QImage(640, 480, QImage::Format_Grayscale8);
    img_lwir.fill(Qt::black);
    lwir_label->setPixmap(QPixmap::fromImage(img_lwir.scaled(lwir_label->size(), Qt::KeepAspectRatio)));

    img_rgb = QImage(640, 480, QImage::Format_RGB888);
    img_rgb.fill(Qt::black);
    rgb_label->setPixmap(QPixmap::fromImage(img_rgb.scaled(rgb_label->size(), Qt::KeepAspectRatio)));
}

void MultiespectralPlugin::updateImages()
{
    lwir_label->setPixmap(QPixmap::fromImage(img_lwir.scaled(lwir_label->size(), Qt::KeepAspectRatio)));
    rgb_label->setPixmap(QPixmap::fromImage(img_rgb.scaled(rgb_label->size(), Qt::KeepAspectRatio)));

    // Calcular la frecuencia de actualización
    double elapsed = frame_timer.elapsed() / 1000.0; // Tiempo en segundos
    double lwir_frequency = lwir_frame_count / elapsed;
    double rgb_frequency = rgb_frame_count / elapsed;
    frequency_label->setText(QString("LWIR freq.: %1 FPS \t| RGB freq.: %2 FPS")
                                 .arg(lwir_frequency, 0, 'f', 2)
                                 .arg(rgb_frequency, 0, 'f', 2));
    num_images_label->setText(QString("LWIR Images: %1  \t| RGB Images.: %2 FPS")
                                  .arg(lwir_total_frame_count, 0, 'f', 2)
                                  .arg(rgb_total_frame_count, 0, 'f', 2));

    // Reiniciar el contador de frames y el temporizador
    lwir_frame_count = 0;
    rgb_frame_count = 0;
    frame_timer.restart();
}

void MultiespectralPlugin::doneCb(const actionlib::SimpleClientGoalState &state, const multiespectral_fb::MultiespectralAcquisitionResultConstPtr &result)
{
    ROS_INFO("Finished in state [%s]", state.toString().c_str());
    ROS_INFO("Result: %d", result->images_acquired);
}

void MultiespectralPlugin::activeCb()
{
    ROS_INFO("Goal just went active");
}

void MultiespectralPlugin::feedbackCb(const multiespectral_fb::MultiespectralAcquisitionFeedbackConstPtr &feedback)
{
    ROS_INFO("Got Feedback: %d", feedback->images_acquired);
}

PLUGINLIB_EXPORT_CLASS(MultiespectralPlugin, rviz::Panel)
