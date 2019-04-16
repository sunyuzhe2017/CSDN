/**
 * @file /include/mul_t/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef mul_t_QNODE_HPP_
#define mul_t_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <std_msgs/String.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <QImage>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace mul_t {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();
  void myCallback(const std_msgs::String::ConstPtr &msg);//string callback function
  void myCallback_img(const sensor_msgs::ImageConstPtr& msg);//camera callback function

  QString str;
  cv::Mat img;
  QImage image;
	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
 // const QString* logstring(){return &str;}
	void log( const LogLevel &level, const std::string &msg);

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();
    //void stringimageSignal(QString);
    void loggingstring();
    void loggingCamera();
private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
  ros::Subscriber chatter_subscriber;
  QStringListModel logging_model;
  image_transport::Subscriber image_sub;

};

}  // namespace mul_t

#endif /* mul_t_QNODE_HPP_ */
