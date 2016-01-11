#ifndef _BLUEJAY_LOG_H_
#define _BLUEJAY_LOG_H_

#include <iostream>
#include <cstdlib>
#include <cstdio>
#include <cstdarg>

#include <bjcomm/message.h>
#include <bjcomm/publisher.h>

/* 
 * Provides logging support while this has not yet been implemented
 */

//FIXME: add to BJOS namespace

class Log{
public:    
    //FIXME: fatal message is not really relevant here (should just be an error that is somewhere handled)
    static void fatal(const std::string origin, const std::string format, ... ) {
        va_list args;
        fprintf( stderr, "[FATAL] " );
        fprintf( stderr, "%s: ", origin.c_str());
        va_start( args, format );
        vfprintf( stderr, format.c_str(), args );
        va_end( args );
        fprintf( stderr, "\n" ); 
        _forward("fatal", origin, format);
    }
    
    static void error(const std::string origin, const std::string format, ... ) {
        va_list args;
        fprintf( stderr, "[ERROR] " );
        fprintf( stderr, "%s: ", origin.c_str());
        va_start( args, format);
        vfprintf( stderr, format.c_str(), args );
        va_end( args );
        fprintf( stderr, "\n" ); 
        _forward("error", origin, format);
    }
    
    static void warn(const std::string origin, const std::string format, ... ) {
        va_list args;
        fprintf( stderr, "[WARN] " );
        fprintf( stderr, "%s: ", origin.c_str());
        va_start( args, format);
        vfprintf( stderr, format.c_str(), args );
        va_end( args );
        fprintf( stderr, "\n" ); 
        _forward("warn", origin, format);
    }
    
    static void info(const std::string origin, const std::string format, ... ) {
        va_list args;
        fprintf( stderr, "[INFO] " );
        fprintf( stderr, "%s: ", origin.c_str());
        va_start( args, format);
        vfprintf( stderr, format.c_str(), args );
        va_end( args );
        fprintf( stderr, "\n" ); 
        _forward("info", origin, format);
    }
    
private:
    static void _forward(const std::string type, const std::string origin, const std::string format, ...){
        static bjcomm::Publisher pub("debug");
        if(!pub.isRunning()) pub.start();
        bjcomm::Message msg(type);
        
        std::stringstream sstr;
        sstr << origin << " " << format;
        msg.setData(sstr.str());
        
        pub.send(msg);
    }
};

#endif