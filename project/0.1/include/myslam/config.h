#ifndef CONFIG_H
#define CONFIG_H

#include "myslam/common_include.h"


class Config // config file reading, and provide config accessible anywhere
// write Config as Singleton--only one objection, when seting config, create the object and read conifg
// when programs end, delete itself

{
private:  // avoid this object in class established somewhere else, only estabilished in setParameterFile
	// actually, object of construct is Cinfig's pointer: static shared_ptr<Config>config_
	// read file: use opencv filestorage class --can read a YAML file and visit any string
	static std::share_ptr<Config> config_;
	cv::FileStorage file_;
	
	Config (){} //private constructor makes a singleton
	
public:
       ~config(); //close the file when deconstrucing
	
	// set a new config file
	static void setParameterFile( const std::string& filename);
	
	// access the params values
	template < typename T>
	static T get( const std::string& key)
	{
		return T (Config::config_->file_[key]);
	}
};