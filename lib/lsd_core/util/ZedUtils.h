
#include <zed/Camera.hpp>

#ifndef USE_ZED
#error "This shouldn't be built unless USE_ZED is defined."
#endif

inline sl::zed::ZEDResolution_mode parseResolution( const std::string &arg )
{
	if( arg == "hd2k" )
		return sl::zed::HD2K;
	else if( arg == "hd1080" )
	 return sl::zed::HD1080;
	else if( arg == "hd720" )
		return sl::zed::HD720;
	else if( arg == "vga" )
		return sl::zed::VGA;
	else
		LOG(FATAL) << "Couldn't parse resolution \"" << arg << "\"";
}

inline std::string resolutionToString( sl::zed::ZEDResolution_mode arg )
{
	switch( arg ) {
		case sl::zed::HD2K:
				return "HD2K";
		case sl::zed::HD1080:
				return "HD1080";
		case sl::zed::HD720:
				return "HD720";
		case sl::zed::VGA:
				return "VGA";
		default:
				return "(unknown)";
	}
}
