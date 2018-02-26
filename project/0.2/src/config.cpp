
#include "myslam/config.h"

namespace myslam
{
	void Config::setParameterFile( const std::string &filename )
	{
		if (config_ == nullptr) //check here, we can directly have access to ptr
			config_ = shared_ptr<Config>(new Config);
		config_->file_ = cv::FileStorage(filename.c_str(), cv::FileStorage::READ);

		if (config_->file_.isOpened() == false) {     // check if file exists
			std::cerr << "params file" << filename << "not exist!!!!" << std::endl;
			config_->file_.release();
			return;
		}
	}

	Config::~Config()
	{
		if (file_.isOpened())
			file_.release();
	}

	shared_ptr<Config> Config::config_ = nullptr;

}
