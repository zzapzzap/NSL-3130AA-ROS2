#include <cstdio>
#include <chrono>
#include <functional>
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
#include <cv_bridge/cv_bridge.h>
//#include <pcl/conversions.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <filesystem>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/srv/set_camera_info.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>
#include <cstdio>
#include <sys/stat.h>
#include <cstdlib>
#include <unistd.h>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "nanolib.h"

#define image_transfer_function

#ifdef image_transfer_function
#include <image_transport/image_transport.hpp>
#endif

namespace nanosys {

	struct CalibParams {
		bool    loaded  = false;
		bool    fisheye = false;
		cv::Mat K;      // 3×3 double  intrinsic
		cv::Mat D;      // 1×N double  distortion
		cv::Mat R;      // 3×3 double  lidar_ROS → camera rotation
		cv::Mat tvec;   // 3×1 double  lidar_ROS → camera translation
	};

	struct ViewerParameter {
		int	frameCount;
		int maxDistance;
		int pointCloudEdgeThreshold;
		int imageType;
		int lensType;
		double lidarAngle;

		bool cvShow;
		bool changedCvShow;
		bool changedImageType;
		bool reOpenLidar;
		bool saveParam;

		std::string	frame_id;
		std::string	ipAddr;
		std::string	netMask;
		std::string	gwAddr;
		std::string	usbPath;
		std::string camera_id;   // matches calib_output/{ID}/intrinsic.yml /extrinsic.yml
    };


	class roboscanPublisher : public rclcpp::Node { 

	public:
		roboscanPublisher();
		~roboscanPublisher();

		void initialise();
		void threadCallback();
		void setReconfigure();
		void publishFrame(NslPCD *frame, NslOption::NslVec3b *rgbframe);
		void startStreaming();

		//static rclcpp::Time timeNow;

		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imgDistancePub;
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imgAmplPub;
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imgGrayPub;
		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloudPub;
		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloudRgbPub;

#ifdef image_transfer_function
		rclcpp::Node::SharedPtr nodeHandle;
		image_transport::ImageTransport imageTransport;
		image_transport::Publisher imagePublisher;
#endif
		ViewerParameter viewerParam;

		boost::scoped_ptr<boost::thread> publisherThread;
		bool runThread;
		NslConfig 		nslConfig;
		int 			nsl_handle;
		CalibParams calib_;
	private:
		std::string yaml_path_;
	    const std::unordered_map<int, std::string> modeIntMap = {
	        {1, "DISTANCE"},
	        {2, "GRAYSCALE"},
			{3, "DISTANCE_AMPLITUDE"},
	        {4, "DISTANCE_GRAYSCALE"},
	        {5, "RGB"},
	        {6, "RGB_DISTANCE"},
	        {7, "RGB_DISTANCE_AMPLITUDE"},
	        {8, "RGB_DISTANCE_GRAYSCALE"}
	    };

		const std::unordered_map<int, std::string> lensIntMap = {
		    {0, "LENS_NF"},
		    {1, "LENS_SF"},
			{2, "LENS_WF"}
		};

		const std::unordered_map<int, std::string> hdrIntMap = {
		    {0, "HDR_None"},
		    {1, "HDR_Spatial"},
			{2, "HDR_Temporal"}
		};		

		const std::unordered_map<int, std::string> modulationIntMap = {
		    {0, "MOD_12Mhz"},
		    {1, "MOD_24Mhz"},
			{2, "MOD_6Mhz"},
			{3, "MOD_3Mhz"}
		};		

		const std::unordered_map<int, std::string> DBIntMap = {
		    {0, "DB_Off"},
		    {1, "DB_6Mhz"},
			{2, "DB_3Mhz"}
		};		

		const std::unordered_map<int, std::string> DBOptIntMap = {
		    {0, "DB_AVOIDANCE"},
		    {1, "DB_CORRECTION"},
			{2, "DB_FULL_CORRECTION"}
		};		


		const std::unordered_map<std::string, int> modeStrMap = {
			{"DISTANCE", 1},
			{"GRAYSCALE", 2},
			{"DISTANCE_AMPLITUDE", 3},
			{"DISTANCE_GRAYSCALE", 4},
			{"RGB", 5},
			{"RGB_DISTANCE", 6},
			{"RGB_DISTANCE_AMPLITUDE", 7},
			{"RGB_DISTANCE_GRAYSCALE", 8}
		};
		
		const std::unordered_map<std::string, int> lensStrMap = {
			{"LENS_NF", 0},
			{"LENS_SF", 1},
			{"LENS_WF", 2},
		};

		const std::unordered_map<std::string, int> hdrStrMap = {
		    {"HDR_None", 0},
		    {"HDR_Spatial", 1},
			{"HDR_Temporal", 2}
		};		

		const std::unordered_map<std::string, int> modulationStrMap = {
		    {"MOD_12Mhz", 0},
		    {"MOD_24Mhz", 1},
			{"MOD_6Mhz", 2},
			{"MOD_3Mhz", 3}
		};		

		const std::unordered_map<std::string, int> DBStrMap = {
		    {"DB_Off", 0},
		    {"DB_6Mhz", 1},
			{"DB_3Mhz", 2}
		};		

		const std::unordered_map<std::string, int> DBOptStrMap = {
		    {"DB_AVOIDANCE", 0},
		    {"DB_CORRECTION", 1},
			{"DB_FULL_CORRECTION", 2}
		};		

		
		// load yaml
		void load_params()
		{
			
			RCLCPP_INFO(this->get_logger(),"Loaded params: path=%s\n", yaml_path_.c_str());
			
			if (std::ifstream(yaml_path_))
			{
				YAML::Node config = YAML::LoadFile(yaml_path_);
				viewerParam.maxDistance = config["MaxDistance"] ? config["MaxDistance"].as<int>() : 12500;
				viewerParam.pointCloudEdgeThreshold = config["PointColud EDGE"] ? config["PointColud EDGE"].as<int>() : 200;
				std::string tmpModeStr = config["ImageType"] ? config["ImageType"].as<std::string>() : "DISTANCE_AMPLITUDE";
				auto itMode = modeStrMap.find(tmpModeStr);
				viewerParam.imageType = (itMode != modeStrMap.end()) ? itMode->second : 3; // defeault DISTANCE_AMPLITUDE

				std::string tmpLensStr = config["LensType"] ? config["LensType"].as<std::string>() : "LENS_SF";
				auto itLens = lensStrMap.find(tmpLensStr);
				viewerParam.lensType = (itLens != lensStrMap.end()) ? itLens->second : 1; // defeault LENS_SF

				viewerParam.lidarAngle   = config["LidarAngle"]   ? config["LidarAngle"].as<double>()   : 0;

				RCLCPP_INFO(this->get_logger(),"Loaded params: max=%d, edge=%d, imgType=%d, lensType=%d, angle=%.2f\n", viewerParam.maxDistance, viewerParam.pointCloudEdgeThreshold, viewerParam.imageType, viewerParam.lensType, viewerParam.lidarAngle);
			}
			else{
				RCLCPP_INFO(this->get_logger(),"Params file not found, using sensor defaults\n");
			}

			const char* env_ip = std::getenv("NSL_CAMERA_IP");
			const char* env_mask = std::getenv("NSL_CAMERA_NETMASK");
			const char* env_gw = std::getenv("NSL_CAMERA_GATEWAY");
			const char* env_usb = std::getenv("NSL_USB_ID");
			if (env_ip && env_ip[0] != '\0') viewerParam.ipAddr = env_ip;
			if (env_mask && env_mask[0] != '\0') viewerParam.netMask = env_mask;
			if (env_gw && env_gw[0] != '\0') viewerParam.gwAddr = env_gw;
			if (env_usb && env_usb[0] != '\0') viewerParam.usbPath = env_usb;
			RCLCPP_INFO(this->get_logger(),"Network target: ip=%s mask=%s gw=%s",
				viewerParam.ipAddr.c_str(), viewerParam.netMask.c_str(), viewerParam.gwAddr.c_str());
		}

		void load_sensor_tuning_params()
		{
			if (!std::ifstream(yaml_path_)) return;

			YAML::Node config = YAML::LoadFile(yaml_path_);

			auto read_int = [&](const char *key, int &target, int min_value, int max_value) {
				if (!config[key]) return;
				int value = config[key].as<int>();
				if (value < min_value) value = min_value;
				if (value > max_value) value = max_value;
				target = value;
			};

			auto read_bool = [&](const char *key, NslOption::FUNCTION_OPTIONS &target) {
				if (!config[key]) return;
				target = config[key].as<bool>()
					? NslOption::FUNCTION_OPTIONS::FUNC_ON
					: NslOption::FUNCTION_OPTIONS::FUNC_OFF;
			};

			if (config["HDRMode"]) {
				std::string hdr = config["HDRMode"].as<std::string>();
				auto it = hdrStrMap.find(hdr);
				if (it != hdrStrMap.end()) {
					nslConfig.hdrOpt = static_cast<NslOption::HDR_OPTIONS>(it->second);
				}
			}

			if (config["Modulation"]) {
				std::string mod = config["Modulation"].as<std::string>();
				auto it = modulationStrMap.find(mod);
				if (it != modulationStrMap.end()) {
					nslConfig.mod_frequencyOpt = static_cast<NslOption::MODULATION_OPTIONS>(it->second);
				}
			}

			if (config["DualBeam"]) {
				std::string dual = config["DualBeam"].as<std::string>();
				auto it = DBStrMap.find(dual);
				if (it != DBStrMap.end()) {
					nslConfig.dbModOpt = static_cast<NslOption::DUALBEAM_MOD_OPTIONS>(it->second);
				}
			}

			if (config["DualBeamOption"]) {
				std::string option = config["DualBeamOption"].as<std::string>();
				auto it = DBOptStrMap.find(option);
				if (it != DBOptStrMap.end()) {
					nslConfig.dbOpsOpt = static_cast<NslOption::DUALBEAM_OPERATION_OPTIONS>(it->second);
				}
			}

			read_int("IntegrationTime3D", nslConfig.integrationTime3D, 0, 2000);
			read_int("IntegrationTimeHdr1", nslConfig.integrationTime3DHdr1, 0, 2000);
			read_int("IntegrationTimeHdr2", nslConfig.integrationTime3DHdr2, 0, 2000);
			read_int("IntegrationTimeGray", nslConfig.integrationTimeGrayScale, 0, 40000);
			read_int("MinAmplitude", nslConfig.minAmplitude, 0, 500);
			if (config["Channel"]) {
				int channel = config["Channel"].as<int>();
				if (channel < 0) channel = 0;
				if (channel > 15) channel = 15;
				nslConfig.mod_channelOpt = static_cast<NslOption::MODULATION_CH_OPTIONS>(channel);
			}
			read_int("TemporalFilterFactor", nslConfig.temporalFactorValue, 0, 1000);
			read_int("TemporalFilterThreshold", nslConfig.temporalThresholdValue, 0, 1000);
			read_int("EdgeFilterThreshold", nslConfig.edgeThresholdValue, 0, 5000);
			read_int("InterferenceDetectionLimit", nslConfig.interferenceDetectionLimitValue, 0, 10000);

			read_bool("MedianFilter", nslConfig.medianOpt);
			read_bool("GaussianFilter", nslConfig.gaussOpt);
			read_bool("UseLastValue", nslConfig.interferenceDetectionLastValueOpt);
			read_bool("GrayscaleLED", nslConfig.grayscaleIlluminationOpt);

			RCLCPP_INFO(this->get_logger(),
				"Loaded sensor tuning: hdr=%d int0=%d int1=%d int2=%d minAmp=%d pcEdge=%d",
				static_cast<int>(nslConfig.hdrOpt),
				nslConfig.integrationTime3D,
				nslConfig.integrationTime3DHdr1,
				nslConfig.integrationTime3DHdr2,
				nslConfig.minAmplitude,
				viewerParam.pointCloudEdgeThreshold);
		}

	    // save yaml
	    void save_params()
	    {
	        std::ofstream fout(yaml_path_);
	        fout << "MaxDistance: " << this->get_parameter("Z. MaxDistance").as_int() << "\n";
	        fout << "PointColud EDGE: " << this->get_parameter("Y. PointColud EDGE").as_int() << "\n";
			fout << "ImageType: " << this->get_parameter("C. imageType").as_string() << "\n";
			fout << "LensType: " << this->get_parameter("B. lensType").as_string() << "\n";
	        fout << "LidarAngle: "   << this->get_parameter("P. transformAngle").as_double() << "\n";
	        fout << "HDRMode: " << this->get_parameter("D. hdr_mode").as_string() << "\n";
	        fout << "IntegrationTime3D: " << this->get_parameter("E. int0").as_int() << "\n";
	        fout << "IntegrationTimeHdr1: " << this->get_parameter("F. int1").as_int() << "\n";
	        fout << "IntegrationTimeHdr2: " << this->get_parameter("G. int2").as_int() << "\n";
	        fout << "IntegrationTimeGray: " << this->get_parameter("H. intGr").as_int() << "\n";
	        fout << "MinAmplitude: " << this->get_parameter("I. minAmplitude").as_int() << "\n";
	        fout << "Modulation: " << this->get_parameter("J. modIndex").as_string() << "\n";
	        fout << "Channel: " << this->get_parameter("K. channel").as_int() << "\n";
	        fout << "MedianFilter: " << (this->get_parameter("R. medianFilter").as_bool() ? "true" : "false") << "\n";
	        fout << "GaussianFilter: " << (this->get_parameter("S. gaussianFilter").as_bool() ? "true" : "false") << "\n";
	        fout << "TemporalFilterFactor: " << static_cast<int>(this->get_parameter("T. temporalFilterFactor").as_double() * 1000.0) << "\n";
	        fout << "TemporalFilterThreshold: " << this->get_parameter("T. temporalFilterFactorThreshold").as_int() << "\n";
	        fout << "EdgeFilterThreshold: " << this->get_parameter("U. edgeFilterThreshold").as_int() << "\n";
	        fout << "InterferenceDetectionLimit: " << this->get_parameter("V. interferenceDetectionLimit").as_int() << "\n";
	        fout << "UseLastValue: " << (this->get_parameter("V. useLastValue").as_bool() ? "true" : "false") << "\n";
	        fout << "DualBeam: " << this->get_parameter("W. dualBeam").as_string() << "\n";
	        fout << "DualBeamOption: " << this->get_parameter("W. dualBeamOption").as_string() << "\n";
	        fout << "GrayscaleLED: " << (this->get_parameter("X. grayscale LED").as_bool() ? "true" : "false") << "\n";

	        fout.close();
	        RCLCPP_INFO(this->get_logger(), "Params saved to %s", yaml_path_.c_str());
	    }
		
		void initNslLibrary();
		std::string resolveUsbOpenId() const;
		bool cameraNetworkReachable(const std::string &ip, int timeout_ms) const;
		void setMatrixColor(cv::Mat image, int x, int y, NslOption::NslVec3b color);
		void timeDelay(int milli);
		void renewParameter();
		void getMouseEvent( int &mouse_xpos, int &mouse_ypos );
		cv::Mat addDistanceInfo(cv::Mat distMat, NslPCD *frame);
		void setWinName();
		void paramDump(const std::string & filename);
		void paramLoad();
		rcl_interfaces::msg::ParameterDescriptor create_Slider(const std::string &description,int from, int to, int step);
		rcl_interfaces::msg::ParameterDescriptor create_Slider(const std::string &description, double from, double to, double step);

		OnSetParametersCallbackHandle::SharedPtr callback_handle_;
		rcl_interfaces::msg::SetParametersResult parametersCallback( const std::vector<rclcpp::Parameter> &parameters);
		bool parameters_ready_ = false;
		bool device_connected_ = false;
		std::string detectUsbSerial();
		void tryLoadCalibParams();
		void publishCalibratedRgbCloud(NslPCD* frame, NslOption::NslVec3b* rgbframe,
		                               const rclcpp::Time& stamp);
		int mouseXpos, mouseYpos;
		bool reconfigure;
		char winName[100];
	};


} //end namespace nanosys
