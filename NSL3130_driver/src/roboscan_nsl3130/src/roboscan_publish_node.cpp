#include <cstdio>
#include <chrono>
#include <functional>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>
//#include <pcl/conversions.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rcl_interfaces/msg/parameter_event.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/srv/set_camera_info.hpp>
#include <cstdio>
#include <sys/stat.h>
#include <cstdlib>
#include <unistd.h>

#include "roboscan_publish_node.hpp"

using namespace NslOption;
using namespace nanosys;
using namespace std::chrono_literals;
using namespace cv;
using namespace std;

#define WIN_NAME "NSL-3130AA IMAGE"
#define  MAX_LEVELS  9
#define NUM_COLORS     		30000

#define LEFTX_MAX	124	
#define RIGHTX_MIN	131
#define RIGHTX_MAX	319	
#define X_INTERVAL	4

#define LEFTY_MAX	116	
#define RIGHTY_MIN	123
#define RIGHTY_MAX	239	
#define Y_INTERVAL	2

#define DISTANCE_INFO_HEIGHT	80

std::atomic<int> x_start = -1, y_start = -1;
std::unique_ptr<NslPCD> latestFrame = std::make_unique<NslPCD>();
NslVec3b  rgbFrame[NSL_RGB_IMAGE_HEIGHT * NSL_RGB_IMAGE_WIDTH];


static void callback_mouse_click(int event, int x, int y, int flags, void* user_data)
{
	std::ignore = flags;
	std::ignore = user_data;
	
	if (event == cv::EVENT_LBUTTONDOWN)
	{
		x_start = x;
		y_start = y;
	}
	else if (event == cv::EVENT_LBUTTONUP)
	{
	}
	else if (event == cv::EVENT_MOUSEMOVE)
	{
	}
}

roboscanPublisher::roboscanPublisher() : 
	Node("roboscan_publish_node")
#ifdef image_transfer_function
	,nodeHandle(std::shared_ptr<roboscanPublisher>(this, [](auto *) {}))
	,imageTransport(nodeHandle)
	,imagePublisher(imageTransport.advertise("roboscanImage", 1000))
#endif	
{ 

    RCLCPP_INFO(this->get_logger(), "start roboscanPublisher...\n");
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    imgDistancePub = this->create_publisher<sensor_msgs::msg::Image>("roboscanDistance", qos_profile);
    imgAmplPub = this->create_publisher<sensor_msgs::msg::Image>("roboscanAmpl", qos_profile);
    imgGrayPub = this->create_publisher<sensor_msgs::msg::Image>("roboscanGray", qos_profile);
    pointcloudPub = this->create_publisher<sensor_msgs::msg::PointCloud2>("roboscanPointCloud", qos_profile);
    pointcloudRgbPub = this->create_publisher<sensor_msgs::msg::PointCloud2>("roboscanPointCloudRgb", qos_profile);

//	yaml_path_ = std::string(std::getenv("HOME")) + "/lidar_params.yaml";
	// NSL_PARAMS_FILE lets camera.launch.py pick a profile (general vs calibration).
	// Defaults to the installed general profile when the env var is unset/empty.
	{
		const char* env_params = std::getenv("NSL_PARAMS_FILE");
		yaml_path_ = (env_params && env_params[0] != '\0')
			? std::string(env_params)
			: ament_index_cpp::get_package_share_directory("roboscan_nsl3130") + "/lidar_params.yaml";
	}

    callback_handle_ = this->add_on_set_parameters_callback(std::bind(&roboscanPublisher::parametersCallback, this, std::placeholders::_1));

	reconfigure = false;
	mouseXpos = -1;
	mouseYpos = -1;
	runThread = true;
    publisherThread.reset(new boost::thread(boost::bind(&roboscanPublisher::threadCallback, this)));


    RCLCPP_INFO(this->get_logger(), "\nRun rqt to view the image!\n");
} 

roboscanPublisher::~roboscanPublisher()
{
	runThread = false;
	publisherThread->join();

	nsl_close();

    RCLCPP_INFO(this->get_logger(), "\nEnd roboscanPublisher()!\n");
}

void roboscanPublisher::initNslLibrary()
{
	nslConfig.lidarAngle = viewerParam.lidarAngle;
	nslConfig.lensType = static_cast<NslOption::LENS_TYPE>(viewerParam.lensType);

	nsl_handle = nsl_open(viewerParam.usbPath.c_str(), &nslConfig, FUNCTION_OPTIONS::FUNC_ON);
	if (nsl_handle >= 0) {
		RCLCPP_INFO(this->get_logger(), "USB(Vendor) connected. Updating camera IP: %s mask: %s gw: %s",
			viewerParam.ipAddr.c_str(), viewerParam.netMask.c_str(), viewerParam.gwAddr.c_str());
		nsl_setIpAddress(nsl_handle, viewerParam.ipAddr.c_str(), viewerParam.netMask.c_str(), viewerParam.gwAddr.c_str());
		nsl_saveConfiguration(nsl_handle);
		RCLCPP_INFO(this->get_logger(), "Streaming via USB.");

		// Auto-detect camera serial from sysfs (VID 1fc9 = NanoSystems)
		if (viewerParam.camera_id.empty() || viewerParam.camera_id == "nsl") {
			std::string ser = detectUsbSerial();
			if (!ser.empty()) {
				viewerParam.camera_id = ser;
				RCLCPP_INFO(this->get_logger(), "Camera ID auto-detected: %s", ser.c_str());
			}
		}
	} else {
		RCLCPP_INFO(this->get_logger(), "USB unavailable (code: %d), connecting via Ethernet %s ...",
			nsl_handle, viewerParam.ipAddr.c_str());
		nsl_handle = nsl_open(viewerParam.ipAddr.c_str(), &nslConfig, FUNCTION_OPTIONS::FUNC_ON);
		if( nsl_handle < 0 ){
			std::cout << "nsl_open::handle open error::" << nsl_handle << std::endl;
			return;
		}
		RCLCPP_INFO(this->get_logger(), "Streaming via Ethernet.");
	}

	load_sensor_tuning_params();

	nsl_setMinAmplitude(nsl_handle, nslConfig.minAmplitude);
	nsl_setIntegrationTime(nsl_handle, nslConfig.integrationTime3D, nslConfig.integrationTime3DHdr1, nslConfig.integrationTime3DHdr2, nslConfig.integrationTimeGrayScale);
	nsl_setHdrMode(nsl_handle, nslConfig.hdrOpt);
	nsl_setFilter(nsl_handle, nslConfig.medianOpt, nslConfig.gaussOpt, nslConfig.temporalFactorValue, nslConfig.temporalThresholdValue, nslConfig.edgeThresholdValue, nslConfig.interferenceDetectionLimitValue, nslConfig.interferenceDetectionLastValueOpt);
	nsl_set3DFilter(nsl_handle, viewerParam.pointCloudEdgeThreshold);
	nsl_setAdcOverflowSaturation(nsl_handle, nslConfig.overflowOpt, nslConfig.saturationOpt);
	nsl_setDualBeam(nsl_handle, nslConfig.dbModOpt, nslConfig.dbOpsOpt);
	nsl_setModulation(nsl_handle, nslConfig.mod_frequencyOpt, nslConfig.mod_channelOpt, nslConfig.mod_enabledAutoChannelOpt);
	nsl_setRoi(nsl_handle, nslConfig.roiXMin, nslConfig.roiYMin, nslConfig.roiXMax, nslConfig.roiYMax);
	nsl_setGrayscaleillumination(nsl_handle, nslConfig.grayscaleIlluminationOpt);

	startStreaming();
}

void roboscanPublisher::threadCallback()
{
	auto lastTime = chrono::steady_clock::now();
	int frameCount = 0;

	while(runThread){

		if( reconfigure ){
			reconfigure = false;
			setReconfigure();
		}

		
		if( viewerParam.imageType ==  static_cast<int>(OPERATION_MODE_OPTIONS::RGB_MODE)
			|| viewerParam.imageType == static_cast<int>(OPERATION_MODE_OPTIONS::RGB_DISTANCE_MODE)
			|| viewerParam.imageType == static_cast<int>(OPERATION_MODE_OPTIONS::RGB_DISTANCE_AMPLITUDE_MODE)
			|| viewerParam.imageType == static_cast<int>(OPERATION_MODE_OPTIONS::RGB_DISTANCE_GRAYSCALE_MODE) )
		{
			if( nsl_getPointCloudRgbData(nsl_handle, latestFrame.get(), rgbFrame, 0) == NSL_ERROR_TYPE::NSL_SUCCESS )
			{
				frameCount++;
				publishFrame(latestFrame.get(), rgbFrame);
			}
		}
		else{
		if( nsl_getPointCloudData(nsl_handle, latestFrame.get(), 0) == NSL_ERROR_TYPE::NSL_SUCCESS )
			{
				frameCount++;
				publishFrame(latestFrame.get(), NULL);
			}
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(10));

		auto now = chrono::steady_clock::now();
		auto elapsed = chrono::duration_cast<chrono::milliseconds>(now - lastTime).count();
		if( elapsed >= 1000 ){
			viewerParam.frameCount = frameCount;
			frameCount = 0;
			lastTime = now;
//			RCLCPP_INFO(this->get_logger(), "frame = %d fps\n", viewerParam.frameCount);
		}
		
	}

	cv::destroyAllWindows();
	RCLCPP_INFO(this->get_logger(), "end threadCallback\n");
}


rcl_interfaces::msg::SetParametersResult roboscanPublisher::parametersCallback( const std::vector<rclcpp::Parameter> &parameters)
{
	rcl_interfaces::msg::SetParametersResult result;
	result.successful = true;
	result.reason = "success";

	if (!parameters_ready_) {
		return result;
	}
	
	// Here update class attributes, do some actions, etc.
	for (const auto &param: parameters)
	{
		if (param.get_name() == "A. cvShow")
		{
			
			bool showCv = param.as_bool();
			if( viewerParam.cvShow != showCv ){
				viewerParam.cvShow = showCv;
				viewerParam.changedCvShow = true;
			}
			
		}
		else if (param.get_name() == "B. lensType")
		{
			string strLensType = param.as_string();
			auto itLens = lensStrMap.find(strLensType);
			int lensType = (itLens != lensStrMap.end()) ? itLens->second : 1; // defeault LENS_SF

			if( viewerParam.lensType != lensType && lensType >=0 && lensType <= 2){
				viewerParam.lensType = lensType;
				viewerParam.reOpenLidar = true;
				viewerParam.saveParam = true;
			}
		}
		else if (param.get_name() == "C. imageType")
		{
			string strImgType = param.as_string();
			auto itMode = modeStrMap.find(strImgType);
			int imgType = (itMode != modeStrMap.end()) ? itMode->second : 3; // defeault DISTANCE_AMPLITUDE

			if( viewerParam.imageType != imgType && imgType >= 1 && imgType <= 8 ){
				viewerParam.imageType = imgType;
				viewerParam.changedImageType = true;
				viewerParam.saveParam = true;
			}
		}
		else if (param.get_name() == "D. hdr_mode")
		{
			string strHdrType = param.as_string();
			auto itHdr = hdrStrMap.find(strHdrType);
			int hdr_opt = (itHdr != hdrStrMap.end()) ? itHdr->second : 0; // Hdr OFF

			nslConfig.hdrOpt = static_cast<NslOption::HDR_OPTIONS>(hdr_opt);
			viewerParam.saveParam = true;
		}
		else if (param.get_name() == "E. int0")
		{
			nslConfig.integrationTime3D = param.as_int();
			viewerParam.saveParam = true;
		}
		else if (param.get_name() == "F. int1")
		{
			nslConfig.integrationTime3DHdr1 = param.as_int();
			viewerParam.saveParam = true;
		}
		else if (param.get_name() == "G. int2")
		{
			nslConfig.integrationTime3DHdr2 = param.as_int();
			viewerParam.saveParam = true;
		}
		else if (param.get_name() == "H. intGr")
		{
			nslConfig.integrationTimeGrayScale = param.as_int();
			viewerParam.saveParam = true;
		}
		else if (param.get_name() == "I. minAmplitude")
		{
			nslConfig.minAmplitude = param.as_int();
			viewerParam.saveParam = true;
		}
		else if (param.get_name() == "J. modIndex")
		{
			string strFreqType = param.as_string();
			auto itFreq = modulationStrMap.find(strFreqType);
			int freq_opt = (itFreq != modulationStrMap.end()) ? itFreq->second : 0; // 12Mhz

			nslConfig.mod_frequencyOpt = static_cast<NslOption::MODULATION_OPTIONS>(freq_opt);
			viewerParam.saveParam = true;
		}
		else if (param.get_name() == "K. channel")
		{
			int ch_opt = param.as_int();
			if( ch_opt > 15 || ch_opt < 0 ) ch_opt = 0;
			nslConfig.mod_channelOpt = static_cast<NslOption::MODULATION_CH_OPTIONS>(ch_opt);
			viewerParam.saveParam = true;
		}
		else if (param.get_name() == "L. roi_leftX")
		{
			int x1_tmp = param.as_int();

			if(x1_tmp % X_INTERVAL ) x1_tmp+=X_INTERVAL-(x1_tmp % X_INTERVAL );
			if(x1_tmp > LEFTX_MAX ) x1_tmp = LEFTX_MAX;

			nslConfig.roiXMin = x1_tmp;

		}
		else if (param.get_name() == "N. roi_rightX")
		{
			int x2_tmp = param.as_int();
			
			if((x2_tmp-RIGHTX_MIN) % X_INTERVAL)	x2_tmp-=((x2_tmp-RIGHTX_MIN) % X_INTERVAL);
			if(x2_tmp < RIGHTX_MIN ) x2_tmp = RIGHTX_MIN;
			if(x2_tmp > RIGHTX_MAX ) x2_tmp = RIGHTX_MAX;
			
			nslConfig.roiXMax = x2_tmp;
		}
		else if (param.get_name() == "M. roi_topY")
		{
			int y1_tmp = param.as_int();
			
			if(y1_tmp % Y_INTERVAL )	y1_tmp++;
			if(y1_tmp > LEFTY_MAX ) y1_tmp = LEFTY_MAX;
			
			nslConfig.roiYMin = y1_tmp;
			
			int y2_tmp = RIGHTY_MAX - y1_tmp;
			nslConfig.roiYMax = y2_tmp;
		}
		else if (param.get_name() == "O. roi_bottomY")
		{
			int y2_tmp = param.as_int();
			
			if(y2_tmp % Y_INTERVAL == 0 )	y2_tmp++;
			if(y2_tmp < RIGHTY_MIN ) y2_tmp = RIGHTY_MIN;
			if(y2_tmp > RIGHTY_MAX ) y2_tmp = RIGHTY_MAX;
			
			nslConfig.roiYMax = y2_tmp;
			
			int y1_tmp = RIGHTY_MAX - y2_tmp;
			nslConfig.roiYMin = y1_tmp;
		}
		else if (param.get_name() == "P. transformAngle")
		{
			int lidarAngle = param.as_double();
			if( viewerParam.lidarAngle != lidarAngle ){
				viewerParam.lidarAngle = lidarAngle;
				viewerParam.reOpenLidar = true;
				viewerParam.saveParam = true;
			}
		}
		else if (param.get_name() == "Q. frameID")
		{
			RCLCPP_INFO(this->get_logger(), "changed frameID %s -> %s\n", viewerParam.frame_id.c_str(), param.as_string().c_str());
			string tmpId = param.as_string();
			if( tmpId != viewerParam.frame_id ) {
				viewerParam.frame_id = tmpId;
				viewerParam.saveParam = true;
			}
		}
		else if (param.get_name() == "R. medianFilter")
		{
			nslConfig.medianOpt = static_cast<NslOption::FUNCTION_OPTIONS>(param.as_bool());
			viewerParam.saveParam = true;
		}
		else if (param.get_name() == "S. gaussianFilter")
		{
			nslConfig.gaussOpt = static_cast<NslOption::FUNCTION_OPTIONS>(param.as_bool());
			viewerParam.saveParam = true;
		}
		else if (param.get_name() == "T. temporalFilterFactor")
		{
			nslConfig.temporalFactorValue = static_cast<int>(param.as_double()*1000);
			if( nslConfig.temporalFactorValue > 1000 ) nslConfig.temporalFactorValue = 1000;
			if( nslConfig.temporalFactorValue < 0 ) nslConfig.temporalFactorValue = 0;
			viewerParam.saveParam = true;
		}
		else if (param.get_name() == "T. temporalFilterFactorThreshold")
		{
			nslConfig.temporalThresholdValue = param.as_int();
			if( nslConfig.temporalThresholdValue < 0 ) nslConfig.temporalThresholdValue = 0;
			viewerParam.saveParam = true;
		}
		else if (param.get_name() == "U. edgeFilterThreshold")
		{
			nslConfig.edgeThresholdValue = param.as_int();
			if( nslConfig.edgeThresholdValue < 0 ) nslConfig.edgeThresholdValue = 0;
			viewerParam.saveParam = true;
		}
		/*
		else if (param.get_name() == "W. temporalEdgeThresholdLow")
		{
			lidarParam.temporalEdgeThresholdLow = param.as_int();
		}
		else if (param.get_name() == "X. temporalEdgeThresholdHigh")
		{
			lidarParam.temporalEdgeThresholdHigh = param.as_int();
		}
		*/
		else if (param.get_name() == "V. interferenceDetectionLimit")
		{
			nslConfig.interferenceDetectionLimitValue = param.as_int();
			if( nslConfig.interferenceDetectionLimitValue > 1000 ) nslConfig.interferenceDetectionLimitValue = 1000;
			if( nslConfig.interferenceDetectionLimitValue < 0 ) nslConfig.interferenceDetectionLimitValue = 0;
			viewerParam.saveParam = true;
		}
		else if (param.get_name() == "V. useLastValue")
		{
			nslConfig.interferenceDetectionLastValueOpt = static_cast<NslOption::FUNCTION_OPTIONS>(param.as_bool());
			viewerParam.saveParam = true;
		}
		else if (param.get_name() == "W. dualBeam")
		{
			string strDBType = param.as_string();
			auto itDb = DBStrMap.find(strDBType);
			int dualBeam = (itDb != DBStrMap.end()) ? itDb->second : 0; // DB OFF

			nslConfig.dbModOpt = static_cast<NslOption::DUALBEAM_MOD_OPTIONS>(dualBeam);
			viewerParam.saveParam = true;
		}
		else if (param.get_name() == "W. dualBeamOption")
		{
			string strDBOptType = param.as_string();
			auto itDbOpt = DBOptStrMap.find(strDBOptType);
			int dualBeamOpt = (itDbOpt != DBOptStrMap.end()) ? itDbOpt->second : 0; // DB_AVOIDANCE
			nslConfig.dbOpsOpt = static_cast<NslOption::DUALBEAM_OPERATION_OPTIONS>(dualBeamOpt);
			viewerParam.saveParam = true;
		}
		else if (param.get_name() == "X. grayscale LED")
		{
			nslConfig.grayscaleIlluminationOpt = static_cast<NslOption::FUNCTION_OPTIONS>(param.as_bool());
			viewerParam.saveParam = true;
		}
		else if (param.get_name() == "Y. PointColud EDGE")
		{
			int tmpThreshold = param.as_int();
			if( viewerParam.pointCloudEdgeThreshold != tmpThreshold ){
				viewerParam.pointCloudEdgeThreshold = tmpThreshold;
				viewerParam.saveParam = true;
			}
		}
		else if (param.get_name() == "Z. MaxDistance")
		{
			int tmpDistance = param.as_int();
			if( viewerParam.maxDistance != tmpDistance ){
				viewerParam.maxDistance = tmpDistance;
				viewerParam.saveParam = true;
			}
		}
		else if (param.get_name() == "0. IP Addr")
		{
			string tmpIp = param.as_string();
			if( tmpIp != viewerParam.ipAddr ) {
				RCLCPP_INFO(this->get_logger(), "changed IP addr %s -> %s\n", viewerParam.ipAddr.c_str(), tmpIp.c_str());

				viewerParam.saveParam = true;
				viewerParam.reOpenLidar = true;
				viewerParam.ipAddr = tmpIp;
			}
		}
		else if (param.get_name() == "1. Net Mask")
		{
			string tmpIp = param.as_string();
			if( tmpIp != viewerParam.netMask ) {
				RCLCPP_INFO(this->get_logger(), "changed Netmask addr %s -> %s\n", viewerParam.netMask.c_str(), tmpIp.c_str());
				viewerParam.saveParam = true;
				viewerParam.reOpenLidar = true;
				viewerParam.netMask= tmpIp;
			}
		}
		else if (param.get_name() == "2. GW Addr")
		{
			string tmpIp = param.as_string();
			if( tmpIp != viewerParam.gwAddr ) {
				RCLCPP_INFO(this->get_logger(), "changed Gw addr %s -> %s\n", viewerParam.gwAddr.c_str(), tmpIp.c_str());
				viewerParam.saveParam = true;
				viewerParam.reOpenLidar = true;
				viewerParam.gwAddr= tmpIp;
			}
		}
	}

	reconfigure = true;
	return result;
}

void roboscanPublisher::timeDelay(int milli)
{
	auto start = std::chrono::steady_clock::now();
	while ( runThread != 0 ) {
		auto now = std::chrono::steady_clock::now();
		if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() >= milli) {
			break;
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
}

std::string roboscanPublisher::detectUsbSerial()
{
    // Read USB serial from sysfs for VID 1fc9 (NanoSystems)
    // Works for Vendor mode (PID 0099) and original mode (PID 0094)
    const std::string USB_SYS = "/sys/bus/usb/devices/";
    try {
        for (const auto& entry : std::filesystem::directory_iterator(USB_SYS)) {
            auto vid_f = entry.path() / "idVendor";
            auto pid_f = entry.path() / "idProduct";
            auto ser_f = entry.path() / "serial";
            if (!std::filesystem::exists(vid_f) || !std::filesystem::exists(ser_f)) continue;

            std::string vid, pid, serial;
            { std::ifstream f(vid_f); f >> vid; }
            { std::ifstream f(pid_f); f >> pid; }
            { std::ifstream f(ser_f); f >> serial; }

            if (vid == "1fc9" && (pid == "0099" || pid == "0094") && !serial.empty())
                return serial;
        }
    } catch (...) {}
    return "";
}

void roboscanPublisher::publishWorldTf()
{
    geometry_msgs::msg::TransformStamped ts;
    ts.header.stamp    = this->get_clock()->now();
    ts.header.frame_id = "reference_lidar_frame";
    ts.child_frame_id  = viewerParam.frame_id;

    if (viewerParam.is_reference) {
        // Reference camera: identity transform (this camera IS the origin)
        ts.transform.rotation.w = 1.0;
        tf_static_broadcaster_->sendTransform(ts);
        RCLCPP_INFO(get_logger(),
            "[TF] is_reference=true: reference_lidar_frame → %s (identity)",
            viewerParam.frame_id.c_str());
    } else {
        // Non-reference camera: load R|t from {camera_id}/to_reference.yml
        const char* env = std::getenv("NSL_CALIB_DIR");
        const std::string dir = env
            ? std::string(env) + "/"
            : std::filesystem::current_path().string() + "/calib_output/";
        const std::string yml = dir + viewerParam.camera_id + "/to_reference.yml";

        cv::FileStorage fs(yml, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            RCLCPP_WARN(get_logger(),
                "[Need R|t] %s not found — TF for '%s' not published.\n"
                "  Create calib_output/%s/to_reference.yml with R|t to register this camera.",
                yml.c_str(), viewerParam.camera_id.c_str(), viewerParam.camera_id.c_str());
            return;
        }

        cv::Mat R, t;
        fs["R"] >> R;
        fs["t"] >> t;
        fs.release();
        R.convertTo(R, CV_64F);
        t.convertTo(t, CV_64F);

        tf2::Matrix3x3 mat(
            R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
            R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
            R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2));
        tf2::Quaternion q;
        mat.getRotation(q);

        ts.transform.rotation.x = q.x();
        ts.transform.rotation.y = q.y();
        ts.transform.rotation.z = q.z();
        ts.transform.rotation.w = q.w();
        ts.transform.translation.x = t.at<double>(0);
        ts.transform.translation.y = t.at<double>(1);
        ts.transform.translation.z = t.at<double>(2);
        tf_static_broadcaster_->sendTransform(ts);
        RCLCPP_INFO(get_logger(),
            "[TF] reference_lidar_frame → %s (from %s)",
            viewerParam.frame_id.c_str(), yml.c_str());
    }
}

void roboscanPublisher::tryLoadCalibParams()
{
    // Use NSL_CALIB_DIR env var if set, otherwise calib_output/ relative to CWD (repo root)
    const char* env_calib = std::getenv("NSL_CALIB_DIR");
    const std::string dir = env_calib
        ? std::string(env_calib) + "/"
        : std::filesystem::current_path().string() + "/calib_output/";
    const std::string intr = dir + viewerParam.camera_id + "/intrinsic.yml";
    const std::string extr = dir + viewerParam.camera_id + "/extrinsic.yml";

    if (!std::filesystem::exists(intr) || !std::filesystem::exists(extr)) {
        calib_.loaded = false;
        RCLCPP_INFO(get_logger(), "No calib files for '%s' → SDK homography",
            viewerParam.camera_id.c_str());
        return;
    }
    try {
        cv::FileStorage fi(intr, cv::FileStorage::READ);
        if (!fi.isOpened()) throw std::runtime_error("cannot open " + intr);
        fi["camera_matrix"]           >> calib_.K;
        fi["distortion_coefficients"] >> calib_.D;
        std::string dm;
        fi["distortion_model"] >> dm;
        calib_.fisheye = (dm == "equidistant" || dm == "fisheye");
        fi.release();

        cv::FileStorage fe(extr, cv::FileStorage::READ);
        if (!fe.isOpened()) throw std::runtime_error("cannot open " + extr);
        fe["R"] >> calib_.R;
        fe["t"] >> calib_.tvec;
        fe.release();

        if (calib_.K.empty() || calib_.R.empty() || calib_.tvec.empty())
            throw std::runtime_error("empty matrix in calib file");

        calib_.K.convertTo(calib_.K, CV_64F);
        calib_.D.convertTo(calib_.D, CV_64F);
        calib_.R.convertTo(calib_.R, CV_64F);
        calib_.tvec.convertTo(calib_.tvec, CV_64F);

        // fisheye::projectPoints requires exactly 4 distortion coefficients
        if (calib_.fisheye && calib_.D.cols > 4) {
            calib_.D = calib_.D.colRange(0, 4);
        }

        calib_.loaded = true;
        RCLCPP_INFO(get_logger(), "Calibration loaded for '%s' (fisheye=%s, D_cols=%d)",
            viewerParam.camera_id.c_str(), calib_.fisheye ? "yes" : "no", calib_.D.cols);
    } catch (const std::exception& e) {
        calib_.loaded = false;
        RCLCPP_WARN(get_logger(), "Calib load failed: %s → SDK homography", e.what());
    }
}

void roboscanPublisher::publishCalibratedRgbCloud(
    NslPCD* frame, NslOption::NslVec3b* rgbframe, const rclcpp::Time& stamp)
{
    auto t0 = std::chrono::steady_clock::now();

    // Zero-copy: NslVec3b {b,g,r} ≡ CV_8UC3 BGR
    cv::Mat rgb_img(NSL_RGB_IMAGE_HEIGHT, NSL_RGB_IMAGE_WIDTH, CV_8UC3, rgbframe);

    const int xMin = frame->roiXMin;
    const int yMin = frame->roiYMin;
    const double* Rd = calib_.R.ptr<double>();
    const double* td = calib_.tvec.ptr<double>();

    // Reused across frames to avoid per-frame heap churn (single publisher thread).
    static thread_local std::vector<cv::Point3d> cam_pts;   // camera frame — projection input
    static thread_local std::vector<cv::Point3f> lidar_pts; // lidar frame — published xyz
    const size_t cap = static_cast<size_t>(frame->width * frame->height);
    cam_pts.clear();   cam_pts.reserve(cap);
    lidar_pts.clear(); lidar_pts.reserve(cap);

    for (int y = 0; y < frame->height; ++y) {
        for (int x = 0; x < frame->width; ++x) {
            double zv = frame->distance3D[OUT_Z][y + yMin][x + xMin];
            if (zv >= NSL_LIMIT_FOR_VALID_DATA) continue;

            double lx =  zv / 1000.0;
            double ly = -frame->distance3D[OUT_X][y + yMin][x + xMin] / 1000.0;
            double lz = -frame->distance3D[OUT_Y][y + yMin][x + xMin] / 1000.0;

            double cx = Rd[0]*lx + Rd[1]*ly + Rd[2]*lz + td[0];
            double cy = Rd[3]*lx + Rd[4]*ly + Rd[5]*lz + td[1];
            double cz = Rd[6]*lx + Rd[7]*ly + Rd[8]*lz + td[2];
            if (cz <= 0.05) continue;

            cam_pts.emplace_back(cx, cy, cz);
            lidar_pts.emplace_back(static_cast<float>(lx),
                                   static_cast<float>(ly),
                                   static_cast<float>(lz));
        }
    }

    if (cam_pts.empty()) return;

    static thread_local std::vector<cv::Point2d> img_pts;
    cv::Mat zeros3 = cv::Mat::zeros(3, 1, CV_64F);
    if (calib_.fisheye) {
        // fisheye::projectPoints needs Nx1x3 Mat
        cv::Mat pts3d(static_cast<int>(cam_pts.size()), 1, CV_64FC3,
                      reinterpret_cast<void*>(cam_pts.data()));
        cv::fisheye::projectPoints(pts3d, img_pts, zeros3, zeros3, calib_.K, calib_.D);
    } else {
        cv::projectPoints(cam_pts, zeros3, zeros3, calib_.K, calib_.D, img_pts);
    }

    pcl::PointCloud<pcl::PointXYZRGB> cloudRgb;
    cloudRgb.points.reserve(img_pts.size());
    cloudRgb.header.frame_id = viewerParam.frame_id;
    cloudRgb.header.stamp    = pcl_conversions::toPCL(stamp);

    for (size_t i = 0; i < img_pts.size(); ++i) {
        int u = static_cast<int>(std::round(img_pts[i].x));
        int v = static_cast<int>(std::round(img_pts[i].y));
        if (u < 0 || u >= NSL_RGB_IMAGE_WIDTH || v < 0 || v >= NSL_RGB_IMAGE_HEIGHT) continue;

        const cv::Point3f& lp = lidar_pts[i];
        pcl::PointXYZRGB pt;
        pt.x = lp.x;
        pt.y = lp.y;
        pt.z = lp.z;
        const cv::Vec3b& bgr = rgb_img.at<cv::Vec3b>(v, u);
        pt.b = bgr[0]; pt.g = bgr[1]; pt.r = bgr[2];
        cloudRgb.points.push_back(pt);
    }

    auto t1 = std::chrono::steady_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    if (ms > 10.0) {
        // Informational only — publish anyway. Discarding an already-computed
        // cloud just because it ran long wastes the work and drops frames.
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "RGB projection slow: %.1f ms", ms);
    }

    cloudRgb.width    = static_cast<uint32_t>(cloudRgb.points.size());
    cloudRgb.height   = 1;
    cloudRgb.is_dense = false;

    sensor_msgs::msg::PointCloud2 msgRgb;
    pcl::toROSMsg(cloudRgb, msgRgb);
    msgRgb.header.stamp    = stamp;
    msgRgb.header.frame_id = viewerParam.frame_id;
    pointcloudRgbPub->publish(msgRgb);
}

void roboscanPublisher::renewParameter()
{
	this->set_parameter(rclcpp::Parameter("0. IP Addr", viewerParam.ipAddr));
	this->set_parameter(rclcpp::Parameter("B. lensType", lensIntMap.at(viewerParam.lensType)));
	this->set_parameter(rclcpp::Parameter("C. imageType", modeIntMap.at(viewerParam.imageType)));
	this->set_parameter(rclcpp::Parameter("D. hdr_mode", hdrIntMap.at(static_cast<int>(nslConfig.hdrOpt))));
	this->set_parameter(rclcpp::Parameter("E. int0", nslConfig.integrationTime3D));
	this->set_parameter(rclcpp::Parameter("F. int1", nslConfig.integrationTime3DHdr1));
	this->set_parameter(rclcpp::Parameter("G. int2", nslConfig.integrationTime3DHdr2));
	this->set_parameter(rclcpp::Parameter("H. intGr", nslConfig.integrationTimeGrayScale));
	this->set_parameter(rclcpp::Parameter("I. minAmplitude", nslConfig.minAmplitude));
	this->set_parameter(rclcpp::Parameter("J. modIndex", modulationIntMap.at(static_cast<int>(nslConfig.mod_frequencyOpt))));
	this->set_parameter(rclcpp::Parameter("K. channel", static_cast<int>(nslConfig.mod_channelOpt)));
	this->set_parameter(rclcpp::Parameter("L. roi_leftX", nslConfig.roiXMin));
	this->set_parameter(rclcpp::Parameter("M. roi_topY", nslConfig.roiYMin));
	this->set_parameter(rclcpp::Parameter("N. roi_rightX", nslConfig.roiXMax));
	this->set_parameter(rclcpp::Parameter("P. transformAngle", viewerParam.lidarAngle));
	this->set_parameter(rclcpp::Parameter("Q. frameID", viewerParam.frame_id));
	this->set_parameter(rclcpp::Parameter("R. medianFilter", static_cast<int>(nslConfig.medianOpt)));
	this->set_parameter(rclcpp::Parameter("S. gaussianFilter", static_cast<int>(nslConfig.gaussOpt)));
	this->set_parameter(rclcpp::Parameter("T. temporalFilterFactor", nslConfig.temporalFactorValue/1000.0));
	this->set_parameter(rclcpp::Parameter("T. temporalFilterFactorThreshold", nslConfig.temporalThresholdValue));
	this->set_parameter(rclcpp::Parameter("U. edgeFilterThreshold", nslConfig.edgeThresholdValue));
	
	this->set_parameter(rclcpp::Parameter("V. interferenceDetectionLimit", nslConfig.interferenceDetectionLimitValue));
	this->set_parameter(rclcpp::Parameter("V. useLastValue", static_cast<int>(nslConfig.interferenceDetectionLastValueOpt)));
	this->set_parameter(rclcpp::Parameter("W. dualBeam", DBIntMap.at(static_cast<int>(nslConfig.dbModOpt))));
	this->set_parameter(rclcpp::Parameter("W. dualBeamOption",DBOptIntMap.at(static_cast<int>(nslConfig.dbOpsOpt))));
	this->set_parameter(rclcpp::Parameter("X. grayscale LED", static_cast<int>(nslConfig.grayscaleIlluminationOpt)));
	this->set_parameter(rclcpp::Parameter("Y. PointColud EDGE", viewerParam.pointCloudEdgeThreshold));
	this->set_parameter(rclcpp::Parameter("Z. MaxDistance", viewerParam.maxDistance));
	
	

}

void roboscanPublisher::setReconfigure()
{	
	if( viewerParam.saveParam )
	{
		viewerParam.saveParam = false;
		save_params();
	}

	if( !viewerParam.changedCvShow )
	{
		nsl_streamingOff(nsl_handle);
		
		std::cout << " nsl_handle = "<< nsl_handle << "nsl_open :: reOpenLidar = "<< viewerParam.reOpenLidar << std::endl;
		
		if( nsl_handle < 0 && viewerParam.reOpenLidar ){

			nslConfig.lidarAngle = viewerParam.lidarAngle;
			nslConfig.lensType = static_cast<NslOption::LENS_TYPE>(viewerParam.lensType);
			nsl_handle = nsl_open(viewerParam.ipAddr.c_str(), &nslConfig, FUNCTION_OPTIONS::FUNC_ON);
			viewerParam.reOpenLidar = false;

			if( nsl_handle >= 0 ){
				renewParameter();
			}
		}
		
		
		nsl_setMinAmplitude(nsl_handle, nslConfig.minAmplitude);
		nsl_setIntegrationTime(nsl_handle, nslConfig.integrationTime3D, nslConfig.integrationTime3DHdr1, nslConfig.integrationTime3DHdr2, nslConfig.integrationTimeGrayScale);
		nsl_setHdrMode(nsl_handle, nslConfig.hdrOpt);
		nsl_setFilter(nsl_handle, nslConfig.medianOpt, nslConfig.gaussOpt, nslConfig.temporalFactorValue, nslConfig.temporalThresholdValue, nslConfig.edgeThresholdValue, nslConfig.interferenceDetectionLimitValue, nslConfig.interferenceDetectionLastValueOpt);
		nsl_set3DFilter(nsl_handle, viewerParam.pointCloudEdgeThreshold);
		nsl_setAdcOverflowSaturation(nsl_handle, nslConfig.overflowOpt, nslConfig.saturationOpt);
		nsl_setDualBeam(nsl_handle, nslConfig.dbModOpt, nslConfig.dbOpsOpt);
		nsl_setModulation(nsl_handle, nslConfig.mod_frequencyOpt, nslConfig.mod_channelOpt, nslConfig.mod_enabledAutoChannelOpt);
		nsl_setRoi(nsl_handle, nslConfig.roiXMin, nslConfig.roiYMin, nslConfig.roiXMax, nslConfig.roiYMax);
		nsl_setGrayscaleillumination(nsl_handle, nslConfig.grayscaleIlluminationOpt);
		
		nsl_saveConfiguration(nsl_handle);

		startStreaming();
	}

	setWinName();
	std::cout << "end setReconfigure"<< std::endl;

}

void roboscanPublisher::setWinName()
{
	bool changedCvShow = viewerParam.changedCvShow || viewerParam.changedImageType;
	viewerParam.changedCvShow = false;
	viewerParam.changedImageType = false;
	
	if( changedCvShow ){
		cv::destroyAllWindows();
	}
	
	if( viewerParam.cvShow == false || changedCvShow == false ) return;
	
	if( viewerParam.imageType == static_cast<int>(OPERATION_MODE_OPTIONS::DISTANCE_MODE)){
		sprintf(winName,"%s(Dist)", WIN_NAME);
	}
	else if( viewerParam.imageType == static_cast<int>(OPERATION_MODE_OPTIONS::GRAYSCALE_MODE)){
		sprintf(winName,"%s(Gray)", WIN_NAME);
	}
	else if( viewerParam.imageType == static_cast<int>(OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE)){
		sprintf(winName,"%s(Dist/Ampl)", WIN_NAME);
	}
	else if( viewerParam.imageType == static_cast<int>(OPERATION_MODE_OPTIONS::DISTANCE_GRAYSCALE_MODE)){
		sprintf(winName,"%s(Dist/Gray)", WIN_NAME);
	}
	else if( viewerParam.imageType == static_cast<int>(OPERATION_MODE_OPTIONS::RGB_MODE)){
		sprintf(winName,"%s(RGB)", WIN_NAME);
	}
	else if( viewerParam.imageType == static_cast<int>(OPERATION_MODE_OPTIONS::RGB_DISTANCE_MODE)){
		sprintf(winName,"%s(RGB/Dist)", WIN_NAME);
	}
	else if( viewerParam.imageType == static_cast<int>(OPERATION_MODE_OPTIONS::RGB_DISTANCE_AMPLITUDE_MODE)){
		sprintf(winName,"%s(RGB/Dist/Ampl)", WIN_NAME);
	}
	else if( viewerParam.imageType == static_cast<int>(OPERATION_MODE_OPTIONS::RGB_DISTANCE_GRAYSCALE_MODE)){
		sprintf(winName,"%s(RGB/Dist/Gray)", WIN_NAME);
	}
	else{
		sprintf(winName,"%s(READY)", WIN_NAME);
	}
	
	cv::namedWindow(winName, cv::WINDOW_AUTOSIZE);
	cv::setWindowProperty(winName, cv::WND_PROP_TOPMOST, 1);	
	cv::setMouseCallback(winName, callback_mouse_click, NULL);
}

rcl_interfaces::msg::ParameterDescriptor roboscanPublisher::create_Slider(const std::string &description, int from, int to, int step)
{
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description = description;

    rcl_interfaces::msg::IntegerRange range;
    range.from_value = from;
    range.to_value = to ;
    range.step = step;

    desc.integer_range.push_back(range);
    return desc;
}

rcl_interfaces::msg::ParameterDescriptor roboscanPublisher::create_Slider(const std::string &description, double from, double to, double step)
{
    rcl_interfaces::msg::ParameterDescriptor desc;
    desc.description = description;

    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = from;
    range.to_value = to;
    range.step = step;

    desc.floating_point_range.push_back(range);
    return desc;
}

void roboscanPublisher::initialise()
{
	std::cout << "Init roboscan_nsl3130 node\n"<< std::endl;

	viewerParam.saveParam = false;
	viewerParam.frameCount = 0;
	viewerParam.cvShow = false;
	viewerParam.changedCvShow = true;
	viewerParam.changedImageType = false;
	viewerParam.reOpenLidar = false;
	viewerParam.maxDistance = 12500;
	viewerParam.pointCloudEdgeThreshold = 200;
	viewerParam.imageType = 3;
	viewerParam.lensType = 1;
	viewerParam.lidarAngle = 0;

	viewerParam.frame_id = "lidar_frame";
	viewerParam.ipAddr   = "192.168.2.220";
	viewerParam.netMask  = "255.255.255.0";
	viewerParam.gwAddr   = "192.168.2.1";
	viewerParam.usbPath  = "";

	load_params();
	initNslLibrary();    // sets camera_id via USB serial auto-detect

	// frame_id derived from USB serial; NSL_FRAME_ID overrides for IP-based naming (e.g. cam_59_lidar_frame)
	viewerParam.frame_id = viewerParam.camera_id.empty()
	    ? "lidar_frame"
	    : viewerParam.camera_id + "_lidar_frame";
	{
		const char* env_frame = std::getenv("NSL_FRAME_ID");
		if (env_frame && env_frame[0] != '\0')
			viewerParam.frame_id = env_frame;
	}

	tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
	publishWorldTf();

	tryLoadCalibParams();
	setWinName();

	rclcpp::Parameter pIPAddr("0. IP Addr", viewerParam.ipAddr);
//	rclcpp::Parameter pNetMask("1. Net Mask", viewerParam.netMask);
//	rclcpp::Parameter pGWAddr("2. GW Addr", viewerParam.gwAddr);

	rclcpp::Parameter pCvShow("A. cvShow", viewerParam.cvShow);
	rclcpp::Parameter pLensType("B. lensType", lensIntMap.at(viewerParam.lensType));
	rclcpp::Parameter pImageType("C. imageType", modeIntMap.at(viewerParam.imageType));
	rclcpp::Parameter pHdr_mode("D. hdr_mode", hdrIntMap.at(static_cast<int>(nslConfig.hdrOpt)));
	rclcpp::Parameter pInt0("E. int0", nslConfig.integrationTime3D);
	rclcpp::Parameter pInt1("F. int1", nslConfig.integrationTime3DHdr1);
	rclcpp::Parameter pInt2("G. int2", nslConfig.integrationTime3DHdr2);
	rclcpp::Parameter pIntGr("H. intGr", nslConfig.integrationTimeGrayScale);
	rclcpp::Parameter pMinAmplitude("I. minAmplitude", nslConfig.minAmplitude);
	rclcpp::Parameter pModIndex("J. modIndex", modulationIntMap.at(static_cast<int>(nslConfig.mod_frequencyOpt)));
	rclcpp::Parameter pChannel("K. channel", static_cast<int>(nslConfig.mod_channelOpt));
	rclcpp::Parameter pRoi_leftX("L. roi_leftX", nslConfig.roiXMin);
	rclcpp::Parameter pRoi_topY("M. roi_topY", nslConfig.roiYMin);
	rclcpp::Parameter pRoi_rightX("N. roi_rightX", nslConfig.roiXMax);
	//rclcpp::Parameter pRoi_bottomY("O. roi_bottomY", nslConfig.roiYMax);
	
	rclcpp::Parameter pTransformAngle("P. transformAngle", viewerParam.lidarAngle);
	rclcpp::Parameter pFrameID("Q. frameID", viewerParam.frame_id);
	rclcpp::Parameter pMedianFilter("R. medianFilter", static_cast<int>(nslConfig.medianOpt));
	rclcpp::Parameter pAverageFilter("S. gaussianFilter", static_cast<int>(nslConfig.gaussOpt));
	rclcpp::Parameter pTemporalFilterFactor("T. temporalFilterFactor", nslConfig.temporalFactorValue/1000.0);
	rclcpp::Parameter pTemporalFilterThreshold("T. temporalFilterFactorThreshold", nslConfig.temporalThresholdValue);
	rclcpp::Parameter pEdgeFilterThreshold("U. edgeFilterThreshold", nslConfig.edgeThresholdValue);
	rclcpp::Parameter pInterferenceDetectionLimit("V. interferenceDetectionLimit", nslConfig.interferenceDetectionLimitValue);
	rclcpp::Parameter pUseLastValue("V. useLastValue", static_cast<int>(nslConfig.interferenceDetectionLastValueOpt));


	rclcpp::Parameter pDualBeam("W. dualBeam", DBIntMap.at(static_cast<int>(nslConfig.dbModOpt)));
	rclcpp::Parameter pDualBeamOpt("W. dualBeamOption", DBOptIntMap.at(static_cast<int>(nslConfig.dbOpsOpt)));	

	rclcpp::Parameter pGrayLED("X. grayscale LED", static_cast<int>(nslConfig.grayscaleIlluminationOpt));
	rclcpp::Parameter pPCEdgeFilter("Y. PointColud EDGE", viewerParam.pointCloudEdgeThreshold);
	rclcpp::Parameter pMaxDistance("Z. MaxDistance", viewerParam.maxDistance);

	this->declare_parameter<string>("0. IP Addr", viewerParam.ipAddr);
//	this->declare_parameter<string>("1. Net Mask", viewerParam.netMask);
//	this->declare_parameter<string>("2. GW Addr", viewerParam.gwAddr);
	this->declare_parameter<bool>("A. cvShow", viewerParam.cvShow);
	this->declare_parameter<string>("B. lensType", lensIntMap.at(viewerParam.lensType));
	this->declare_parameter<string>("C. imageType", modeIntMap.at(viewerParam.imageType));
	this->declare_parameter<string>("D. hdr_mode", hdrIntMap.at(static_cast<int>(nslConfig.hdrOpt)));

	auto int_0 = create_Slider("Defaut integration time", 0, 2000, 1);
	this->declare_parameter<int>("E. int0", nslConfig.integrationTime3D, int_0);

	auto int_1 = create_Slider("HDR integration time1", 0, 2000, 1);
	this->declare_parameter<int>("F. int1", nslConfig.integrationTime3DHdr1, int_1);

	auto int_2 = create_Slider("HDR integration time2", 0, 2000, 1);
	this->declare_parameter<int>("G. int2", nslConfig.integrationTime3DHdr2, int_2);

	auto int_Gr = create_Slider("Grayscale time", 0, 40000, 1);
	this->declare_parameter<int>("H. intGr",nslConfig.integrationTimeGrayScale, int_Gr);

	auto min_Amplitude = create_Slider("minimum Amplitude", 0, 1000, 1);
	this->declare_parameter<int>("I. minAmplitude", nslConfig.minAmplitude, min_Amplitude);

	this->declare_parameter<string>("J. modIndex", modulationIntMap.at(static_cast<int>(nslConfig.mod_frequencyOpt)));
	
	auto channelOpt = create_Slider("Channel", 0, 15, 1);
	this->declare_parameter<int>("K. channel", static_cast<int>(nslConfig.mod_channelOpt), channelOpt);

	auto roi_LeftX = create_Slider("roi LeftX", 0, 120, 8);
	this->declare_parameter<int>("L. roi_leftX", nslConfig.roiXMin, roi_LeftX);
 
	auto roi_TopY = create_Slider("roi TopY", 0, 116, 2);
	this->declare_parameter<int>("M. roi_topY",  nslConfig.roiYMin, roi_TopY);

	auto roi_RightX = create_Slider("roi rightX", 127, 319, 8);
	this->declare_parameter<int>("N. roi_rightX", (nslConfig.roiXMax == 0) ? 319 : nslConfig.roiXMax, roi_RightX);
//	this->declare_parameter<int>("O. roi_bottomY", nslConfig.roiYMax);

	auto transform_Angle = create_Slider("Angle", -90.0, 90.0, 9.0);
	this->declare_parameter<double>("P. transformAngle", viewerParam.lidarAngle, transform_Angle);

	this->declare_parameter<string>("Q. frameID", viewerParam.frame_id);

	this->declare_parameter<bool>("R. medianFilter", static_cast<int>(nslConfig.medianOpt));
	this->declare_parameter<bool>("S. gaussianFilter", static_cast<int>(nslConfig.gaussOpt));

	auto temporal_FactorValue = create_Slider("temporal FactorValue", 0.0, 1.0, 0.01);
	this->declare_parameter<double>("T. temporalFilterFactor", nslConfig.temporalFactorValue/1000.0, temporal_FactorValue);

	auto temporal_Threshold = create_Slider("temporal Threshold", 0, 1000, 1);
	this->declare_parameter<int>("T. temporalFilterFactorThreshold", nslConfig.temporalThresholdValue, temporal_Threshold);

	auto edge_Threshold = create_Slider("edge Threshold", 0, 5000, 1);
	this->declare_parameter<int>("U. edgeFilterThreshold", nslConfig.edgeThresholdValue, edge_Threshold);

	auto interference_DetectionLimit = create_Slider("interference DetectionLimit", 0, 10000, 1);
	this->declare_parameter<int>("V. interferenceDetectionLimit", nslConfig.interferenceDetectionLimitValue,interference_DetectionLimit);

	this->declare_parameter<bool>("V. useLastValue", static_cast<int>(nslConfig.interferenceDetectionLastValueOpt));

	this->declare_parameter<string>("W. dualBeam", DBIntMap.at(static_cast<int>(nslConfig.dbModOpt)));
	this->declare_parameter<string>("W. dualBeamOption", DBOptIntMap.at(static_cast<int>(nslConfig.dbOpsOpt)));
	this->declare_parameter<bool>("X. grayscale LED", static_cast<int>(nslConfig.grayscaleIlluminationOpt));

	auto pointCloud_EdgeThreshold = create_Slider("pointCloud EdgeThreshold", 0, 10000, 1);
	this->declare_parameter<int>("Y. PointColud EDGE", viewerParam.pointCloudEdgeThreshold, pointCloud_EdgeThreshold);

	auto max_Distance = create_Slider("max Distance", 0, 50000, 1);
	this->declare_parameter<int>("Z. MaxDistance", viewerParam.maxDistance, max_Distance);



	this->set_parameter(pFrameID);
	this->set_parameter(pIPAddr);
//	this->set_parameter(pNetMask);
//	this->set_parameter(pGWAddr);

	this->set_parameter(pLensType);
	this->set_parameter(pImageType);
	this->set_parameter(pHdr_mode);
	this->set_parameter(pInt0);
	this->set_parameter(pInt1);
	this->set_parameter(pInt2);
	this->set_parameter(pIntGr);
	this->set_parameter(pMinAmplitude);
	this->set_parameter(pModIndex);
	this->set_parameter(pChannel);
	this->set_parameter(pRoi_leftX);
	this->set_parameter(pRoi_topY);
	this->set_parameter(pRoi_rightX);
//	this->set_parameter(pRoi_bottomY);
	this->set_parameter(pTransformAngle);
	this->set_parameter(pMedianFilter);
	this->set_parameter(pAverageFilter);
	this->set_parameter(pTemporalFilterFactor);
	this->set_parameter(pTemporalFilterThreshold);
	this->set_parameter(pEdgeFilterThreshold);
	//this->set_parameter(pTemporalEdgeThresholdLow);
	//this->set_parameter(pTemporalEdgeThresholdHigh);
	this->set_parameter(pInterferenceDetectionLimit);
	this->set_parameter(pUseLastValue);

	this->set_parameter(pCvShow);
	this->set_parameter(pDualBeam);
	this->set_parameter(pDualBeamOpt);	
	this->set_parameter(pGrayLED);
	this->set_parameter(pPCEdgeFilter);
	this->set_parameter(pMaxDistance);

	viewerParam.saveParam = false;
	reconfigure = false;
	parameters_ready_ = true;
	
	RCLCPP_INFO(this->get_logger(),"end initialise()\n");
}


void roboscanPublisher::startStreaming()
{	
	if( viewerParam.imageType == static_cast<int>(OPERATION_MODE_OPTIONS::DISTANCE_MODE)){
		nsl_streamingOn(nsl_handle, OPERATION_MODE_OPTIONS::DISTANCE_MODE);
	}
	else if( viewerParam.imageType == static_cast<int>(OPERATION_MODE_OPTIONS::GRAYSCALE_MODE)){
		nsl_setColorRange(viewerParam.maxDistance, MAX_GRAYSCALE_VALUE, NslOption::FUNCTION_OPTIONS::FUNC_ON);
		nsl_streamingOn(nsl_handle, OPERATION_MODE_OPTIONS::GRAYSCALE_MODE);
	}
	else if( viewerParam.imageType == static_cast<int>(OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE)){
		nsl_setColorRange(viewerParam.maxDistance, MAX_GRAYSCALE_VALUE, NslOption::FUNCTION_OPTIONS::FUNC_OFF);
		nsl_streamingOn(nsl_handle, OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE);
	}
	else if( viewerParam.imageType == static_cast<int>(OPERATION_MODE_OPTIONS::DISTANCE_GRAYSCALE_MODE)){
		nsl_setColorRange(viewerParam.maxDistance, MAX_GRAYSCALE_VALUE, NslOption::FUNCTION_OPTIONS::FUNC_ON);
		nsl_streamingOn(nsl_handle, OPERATION_MODE_OPTIONS::DISTANCE_GRAYSCALE_MODE);
	}
	else if( viewerParam.imageType == static_cast<int>(OPERATION_MODE_OPTIONS::RGB_MODE)){
		nsl_streamingOn(nsl_handle, OPERATION_MODE_OPTIONS::RGB_MODE);
	}
	else if( viewerParam.imageType == static_cast<int>(OPERATION_MODE_OPTIONS::RGB_DISTANCE_MODE)){
		nsl_streamingOn(nsl_handle, OPERATION_MODE_OPTIONS::RGB_DISTANCE_MODE);
	}
	else if( viewerParam.imageType == static_cast<int>(OPERATION_MODE_OPTIONS::RGB_DISTANCE_AMPLITUDE_MODE)){
		nsl_setColorRange(viewerParam.maxDistance, MAX_GRAYSCALE_VALUE, NslOption::FUNCTION_OPTIONS::FUNC_OFF);
		nsl_streamingOn(nsl_handle, OPERATION_MODE_OPTIONS::RGB_DISTANCE_AMPLITUDE_MODE);
	}
	else if( viewerParam.imageType == static_cast<int>(OPERATION_MODE_OPTIONS::RGB_DISTANCE_GRAYSCALE_MODE)){
		nsl_setColorRange(viewerParam.maxDistance, MAX_GRAYSCALE_VALUE, NslOption::FUNCTION_OPTIONS::FUNC_ON);
		nsl_streamingOn(nsl_handle, OPERATION_MODE_OPTIONS::RGB_DISTANCE_GRAYSCALE_MODE);
	}
	else{
		std::cout << "operation mode NONE~~~"<< std::endl;
	}
}


cv::Mat roboscanPublisher::addDistanceInfo(cv::Mat distMat, NslPCD *frame)
{
	int xpos = mouseXpos;
	int ypos = mouseYpos;
	
	if( (ypos > 0 && ypos < frame->height)){
		// mouseXpos, mouseYpos
//		int origin_xpos = xpos;
		Mat infoImage(DISTANCE_INFO_HEIGHT, distMat.cols, CV_8UC3, Scalar(255, 255, 255));

		line(distMat, Point(xpos-10, ypos), Point(xpos+10, ypos), Scalar(255, 255, 0), 2);
		line(distMat, Point(xpos, ypos-10), Point(xpos, ypos+10), Scalar(255, 255, 0), 2);

		if( xpos >= frame->width ){ 
			xpos -= frame->width;
		}

		string dist2D_caption;
		string dist3D_caption;
		string info_caption;

		int distance2D = frame->distance2D[ypos][xpos];
		if( distance2D > NSL_LIMIT_FOR_VALID_DATA ){
			if( distance2D == NSL_ADC_OVERFLOW )
				dist2D_caption = format("X:%d,Y:%d ADC_OVERFLOW", xpos, ypos);
			else if( distance2D == NSL_SATURATION )
				dist2D_caption = format("X:%d,Y:%d SATURATION", xpos, ypos);
			else if( distance2D == NSL_BAD_PIXEL )
				dist2D_caption = format("X:%d,Y:%d BAD_PIXEL", xpos, ypos);
			else if( distance2D == NSL_INTERFERENCE )
				dist2D_caption = format("X:%d,Y:%d INTERFERENCE", xpos, ypos);
			else if( distance2D == NSL_EDGE_DETECTED )
				dist2D_caption = format("X:%d,Y:%d EDGE_FILTERED", xpos, ypos);
			else
				dist2D_caption = format("X:%d,Y:%d LOW_AMPLITUDE", xpos, ypos);
		}
		else{
			if( frame->operationMode == OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE || frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_AMPLITUDE_MODE ) {
				dist2D_caption = format("2D X:%d Y:%d %dmm/%dlsb", xpos, ypos, frame->distance2D[ypos][xpos], frame->amplitude[ypos][xpos]);
				dist3D_caption = format("3D X:%.1fmm Y:%.1fmm Z:%.1fmm", frame->distance3D[OUT_X][ypos][xpos], frame->distance3D[OUT_Y][ypos][xpos], frame->distance3D[OUT_Z][ypos][xpos]);
			}
			else{
				dist2D_caption = format("2D X:%d Y:%d <%d>mm", xpos, ypos, frame->distance2D[ypos][xpos]);
				dist3D_caption = format("3D X:%.1fmm Y:%.1fmm Z:%.1fmm", frame->distance3D[OUT_X][ypos][xpos], frame->distance3D[OUT_Y][ypos][xpos], frame->distance3D[OUT_Z][ypos][xpos]);
			}
		}
		
		info_caption = format("%s:%dx%d %.2f'C, %d fps", toString(frame->operationMode), frame->width, frame->height, frame->temperature, viewerParam.frameCount);

		putText(infoImage, info_caption.c_str(), Point(10, 23), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 0), 1, cv::LINE_AA);
		putText(infoImage, dist2D_caption.c_str(), Point(10, 46), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 0), 1, cv::LINE_AA);
		putText(infoImage, dist3D_caption.c_str(), Point(10, 70), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 0), 1, cv::LINE_AA);
		vconcat(distMat, infoImage, distMat);
	}
	else{
		Mat infoImage(DISTANCE_INFO_HEIGHT, distMat.cols, CV_8UC3, Scalar(255, 255, 255));

		string info_caption = format("%s:%dx%d %.2f'C, %d fps", toString(frame->operationMode), frame->width, frame->height, frame->temperature, viewerParam.frameCount);
		putText(infoImage, info_caption.c_str(), Point(10, 23), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 0), 1, cv::LINE_AA);		
		vconcat(distMat, infoImage, distMat);
	}

	return distMat;
}

void roboscanPublisher::setMatrixColor(Mat image, int x, int y, NslVec3b color)
{
	image.at<Vec3b>(y,x)[0] = color.b;
	image.at<Vec3b>(y,x)[1] = color.g;
	image.at<Vec3b>(y,x)[2] = color.r;
}

void roboscanPublisher::publishFrame(NslPCD *frame, NslVec3b *rgbframe)
{
	static rclcpp::Clock s_rclcpp_clock;
	auto data_stamp = s_rclcpp_clock.now();

	cv::Mat distanceMat(frame->height, frame->width, CV_8UC3, Scalar(255, 255, 255));	// distance
	cv::Mat amplitudeMat(frame->height, frame->width, CV_8UC3, Scalar(255, 255, 255));	// amplitude
#ifdef image_transfer_function
	cv::Mat rgbMat(NSL_RGB_IMAGE_HEIGHT, NSL_RGB_IMAGE_WIDTH, CV_8UC3, Scalar(255, 255, 255));
#endif


	if(frame->operationMode == OPERATION_MODE_OPTIONS::DISTANCE_MODE 
		|| frame->operationMode == OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE 
		|| frame->operationMode == OPERATION_MODE_OPTIONS::DISTANCE_GRAYSCALE_MODE 
		|| frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_MODE 
		|| frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_AMPLITUDE_MODE 
		|| frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_GRAYSCALE_MODE )
	{
		sensor_msgs::msg::Image imgDistance;

		std::vector<uint8_t> result;
		result.reserve(frame->height * frame->width * 2);

		int xMin = frame->roiXMin;
		int yMin = frame->roiYMin;
		
		for (int y = 0; y < frame->height; ++y) {
			for (int x = 0; x < frame->width; ++x) {
				result.push_back(static_cast<uint8_t>(frame->distance2D[y+yMin][x+xMin] & 0xFF));		 // LSB
				result.push_back(static_cast<uint8_t>((frame->distance2D[y+yMin][x+xMin] >> 8) & 0xFF)); // MSB

				setMatrixColor(distanceMat, x+xMin, y+yMin, nsl_getDistanceColor(frame->distance2D[y+yMin][x+xMin]));
			}
		}

		imgDistance.header.stamp = data_stamp;
		imgDistance.header.frame_id = viewerParam.frame_id;
		imgDistance.height = static_cast<uint32_t>(frame->height);
		imgDistance.width = static_cast<uint32_t>(frame->width);
		imgDistance.encoding = sensor_msgs::image_encodings::MONO16;
		imgDistance.step = imgDistance.width * 2;
		imgDistance.is_bigendian = 0;
		imgDistance.data = result;
		imgDistancePub->publish(imgDistance);
	}

	if(frame->operationMode == OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE 
		|| frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_AMPLITUDE_MODE)
	{
		sensor_msgs::msg::Image imgAmpl;

		std::vector<uint8_t> result;
		result.reserve(frame->height * frame->width * 2);

		int xMin = frame->roiXMin;
		int yMin = frame->roiYMin;

		for (int y = 0; y < frame->height; ++y) {
			for (int x = 0; x < frame->width; ++x) {
				result.push_back(static_cast<uint8_t>(frame->amplitude[y+yMin][x+xMin] & 0xFF));		// LSB
				result.push_back(static_cast<uint8_t>((frame->amplitude[y+yMin][x+xMin] >> 8) & 0xFF)); // MSB

				setMatrixColor(amplitudeMat, x+xMin, y+yMin, nsl_getAmplitudeColor(frame->amplitude[y+yMin][x+xMin]));
			}
		}

		imgAmpl.header.stamp = data_stamp;
		imgAmpl.header.frame_id = viewerParam.frame_id;
		imgAmpl.height = static_cast<uint32_t>(frame->height);
		imgAmpl.width = static_cast<uint32_t>(frame->width);
		imgAmpl.encoding = sensor_msgs::image_encodings::MONO16;
		imgAmpl.step = imgAmpl.width * 2;
		imgAmpl.is_bigendian = 0;
		imgAmpl.data = result;
		imgAmplPub->publish(imgAmpl);
	}	

	
	if(frame->operationMode == OPERATION_MODE_OPTIONS::GRAYSCALE_MODE
		|| frame->operationMode == OPERATION_MODE_OPTIONS::DISTANCE_GRAYSCALE_MODE 
		|| frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_GRAYSCALE_MODE)
	{
		sensor_msgs::msg::Image imgGray;

		std::vector<uint8_t> result;
		result.reserve(frame->height * frame->width * 2);

		int xMin = frame->roiXMin;
		int yMin = frame->roiYMin;
		
		for (int y = 0; y < frame->height; ++y) {
			for (int x = 0; x < frame->width; ++x) {
				result.push_back(static_cast<uint8_t>(frame->amplitude[y+yMin][x+xMin] & 0xFF));		// LSB
				result.push_back(static_cast<uint8_t>((frame->amplitude[y+yMin][x+xMin] >> 8) & 0xFF)); // MSB

				setMatrixColor(amplitudeMat, x+xMin, y+yMin, nsl_getAmplitudeColor(frame->amplitude[y+yMin][x+xMin]));
			}
		}

		imgGray.header.stamp = data_stamp;
		imgGray.header.frame_id = viewerParam.frame_id;
		imgGray.height = static_cast<uint32_t>(frame->height);
		imgGray.width = static_cast<uint32_t>(frame->width);
		imgGray.encoding = sensor_msgs::image_encodings::MONO16;
		imgGray.step = imgGray.width * 2;
		imgGray.is_bigendian = 0;
		imgGray.data = result;
		imgGrayPub->publish(imgGray);
	}		
	

#ifdef image_transfer_function
	if(frame->operationMode == OPERATION_MODE_OPTIONS::RGB_MODE 
		|| frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_MODE 
		|| frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_AMPLITUDE_MODE
		|| frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_GRAYSCALE_MODE)
	{
		if( rgbframe != NULL ){
			int totalPixels = NSL_RGB_IMAGE_HEIGHT * NSL_RGB_IMAGE_WIDTH;
			cv::Vec3b* dstPtr = rgbMat.ptr<cv::Vec3b>();
			NslOption::NslVec3b* srcPtr = &rgbframe[0];
			
			for (int i = 0; i < totalPixels; ++i) {
				dstPtr[i] = cv::Vec3b(
					srcPtr[i].b,  // blue
					srcPtr[i].g,  // green
					srcPtr[i].r   // red
				);
			}

			cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
			cv_ptr->header.stamp = data_stamp;
			cv_ptr->header.frame_id = viewerParam.frame_id;
			cv_ptr->image = rgbMat;
			cv_ptr->encoding = "bgr8";
		
			imagePublisher.publish(cv_ptr->toImageMsg());
		}
	}
#endif

	if( frame->operationMode != OPERATION_MODE_OPTIONS::RGB_MODE
		&& frame->operationMode != OPERATION_MODE_OPTIONS::GRAYSCALE_MODE )
	{
		const size_t nPixel = frame->width * frame->height;
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
		cloud->header.frame_id = viewerParam.frame_id;
		cloud->header.stamp = pcl_conversions::toPCL(data_stamp);
		//cloud->header.stamp = static_cast<uint64_t>(data_stamp.nanoseconds());
		cloud->width = static_cast<uint32_t>(frame->width);
		cloud->height = static_cast<uint32_t>(frame->height);
		cloud->is_dense = false;
		cloud->points.resize(nPixel);

		int xMin = frame->roiXMin;
		int yMin = frame->roiYMin;

		for(int y = 0, index = 0; y < frame->height; y++)
		{
			for(int x = 0; x < frame->width; x++, index++)
			{
				pcl::PointXYZI &point = cloud->points[index];

				if( frame->distance3D[OUT_Z][y+yMin][x+xMin] < NSL_LIMIT_FOR_VALID_DATA )
				{
					point.x = (double)(frame->distance3D[OUT_Z][y+yMin][x+xMin]/1000);
					point.y = (double)(-frame->distance3D[OUT_X][y+yMin][x+xMin]/1000);
					point.z = (double)(-frame->distance3D[OUT_Y][y+yMin][x+xMin]/1000);
					point.intensity = frame->amplitude[y+yMin][x+xMin];
				}
				else{
					point.x = std::numeric_limits<float>::quiet_NaN();
					point.y = std::numeric_limits<float>::quiet_NaN();
					point.z = std::numeric_limits<float>::quiet_NaN();
					point.intensity = std::numeric_limits<float>::quiet_NaN();
				}
			}
		}

		
		sensor_msgs::msg::PointCloud2 msg;
		pcl::toROSMsg(*cloud, msg);
		msg.header.stamp = data_stamp;
		msg.header.frame_id = viewerParam.frame_id;
		pointcloudPub->publish(msg);

		if( rgbframe != nullptr && (
			frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_MODE ||
			frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_AMPLITUDE_MODE ||
			frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_GRAYSCALE_MODE) )
		{
			if (calib_.loaded) {
				publishCalibratedRgbCloud(frame, rgbframe, data_stamp);
			} else {
				const float scaleX = static_cast<float>(NSL_RGB_IMAGE_WIDTH)  / frame->width;
				const float scaleY = static_cast<float>(NSL_RGB_IMAGE_HEIGHT) / frame->height;

				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRgb(new pcl::PointCloud<pcl::PointXYZRGB>());
				cloudRgb->header.frame_id = viewerParam.frame_id;
				cloudRgb->header.stamp = pcl_conversions::toPCL(data_stamp);
				cloudRgb->width  = static_cast<uint32_t>(frame->width);
				cloudRgb->height = static_cast<uint32_t>(frame->height);
				cloudRgb->is_dense = false;
				cloudRgb->points.resize(nPixel);

				for(int y = 0, index = 0; y < frame->height; y++)
				{
					for(int x = 0; x < frame->width; x++, index++)
					{
						pcl::PointXYZRGB &pt = cloudRgb->points[index];

						if( frame->distance3D[OUT_Z][y+yMin][x+xMin] < NSL_LIMIT_FOR_VALID_DATA )
						{
							pt.x = (double)(frame->distance3D[OUT_Z][y+yMin][x+xMin]/1000);
							pt.y = (double)(-frame->distance3D[OUT_X][y+yMin][x+xMin]/1000);
							pt.z = (double)(-frame->distance3D[OUT_Y][y+yMin][x+xMin]/1000);

							int rgbX = std::min(static_cast<int>(x * scaleX), NSL_RGB_IMAGE_WIDTH  - 1);
							int rgbY = std::min(static_cast<int>(y * scaleY), NSL_RGB_IMAGE_HEIGHT - 1);
							const NslVec3b &color = rgbframe[rgbY * NSL_RGB_IMAGE_WIDTH + rgbX];
							pt.r = color.r;
							pt.g = color.g;
							pt.b = color.b;
						}
						else
						{
							pt.x = std::numeric_limits<float>::quiet_NaN();
							pt.y = std::numeric_limits<float>::quiet_NaN();
							pt.z = std::numeric_limits<float>::quiet_NaN();
							pt.r = 0; pt.g = 0; pt.b = 0;
						}
					}
				}

				sensor_msgs::msg::PointCloud2 msgRgb;
				pcl::toROSMsg(*cloudRgb, msgRgb);
				msgRgb.header.stamp = data_stamp;
				msgRgb.header.frame_id = viewerParam.frame_id;
				pointcloudRgbPub->publish(msgRgb);
			}
		}
	}
	
	if(viewerParam.cvShow == true)
	{	
		getMouseEvent(mouseXpos, mouseYpos);
			
		if( frame->operationMode == OPERATION_MODE_OPTIONS::DISTANCE_MODE ){
			distanceMat = addDistanceInfo(distanceMat, frame);
		}
		else if( frame->operationMode == OPERATION_MODE_OPTIONS::GRAYSCALE_MODE ){
			distanceMat = addDistanceInfo(amplitudeMat, frame);
		}
		else if( frame->operationMode == OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE ){
			cv::hconcat(distanceMat, amplitudeMat, distanceMat);
			distanceMat = addDistanceInfo(distanceMat, frame);
		}
		else if( frame->operationMode == OPERATION_MODE_OPTIONS::DISTANCE_GRAYSCALE_MODE ){
			cv::hconcat(distanceMat, amplitudeMat, distanceMat);
			distanceMat = addDistanceInfo(distanceMat, frame);
		}
		else if( frame->operationMode == OPERATION_MODE_OPTIONS::RGB_MODE ){
			resize( rgbMat, rgbMat, Size( 640, 480 ), 0, 0);
			distanceMat = rgbMat;
		}
		else if( frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_MODE ){
			resize( rgbMat, rgbMat, Size( distanceMat.cols, distanceMat.rows ), 0, 0);
			hconcat( distanceMat, rgbMat, distanceMat );
			distanceMat = addDistanceInfo(distanceMat, frame);
		}
		else if( frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_AMPLITUDE_MODE ){
			cv::hconcat(distanceMat, amplitudeMat, distanceMat);
			resize( rgbMat, rgbMat, Size( distanceMat.cols, distanceMat.rows ), 0, 0);
			vconcat( distanceMat, rgbMat, distanceMat );
			distanceMat = addDistanceInfo(distanceMat, frame);
		}
		else if( frame->operationMode == OPERATION_MODE_OPTIONS::RGB_DISTANCE_GRAYSCALE_MODE ){
			cv::hconcat(distanceMat, amplitudeMat, distanceMat);
			resize( rgbMat, rgbMat, Size( distanceMat.cols, distanceMat.rows ), 0, 0);
			vconcat( distanceMat, rgbMat, distanceMat );
			distanceMat = addDistanceInfo(distanceMat, frame);
		}
		
		imshow(winName, distanceMat);
		waitKey(1);
	}

}


void roboscanPublisher::getMouseEvent( int &mouse_xpos, int &mouse_ypos )
{
	mouse_xpos = x_start;
	mouse_ypos = y_start;
}

/*
	ubuntu usb device
	
	sudo apt-get install libopencv-dev
	sudo apt-get install libpcl-dev(1.8.1)

	$ sudo vi /etc/udev/rules.d/defined_lidar.rules
	KERNEL=="ttyACM*", ATTRS{idVendor}=="1FC9", ATTRS{idProduct}=="0094", MODE:="0777",SYMLINK+="ttyNsl3140"

	$ service udev reload
	$ service udev restart

	ubuntu Network UDP speed up
	sudo sysctl -w net.core.rmem_max=22020096
	sudo sysctl -w net.core.rmem_default=22020096
*/

int main(int argc, char ** argv)
{
	(void) argc;
	(void) argv;
	
	rclcpp::init(argc, argv);

	auto node = std::make_shared<roboscanPublisher>();
	node->initialise();
	
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
