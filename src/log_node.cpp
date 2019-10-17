#include "log_node.hpp"

void logLoop( FileNode log_config )
{
    int enable;
    string path;
    log_config["ENABLE"] >> enable;
    log_config["LOG_FILE_PATH"] >> path;
    if( enable == 0 )
    {
        cout << "[WARNING]: log node disabled" << endl;
        return;
    }

    LogStatus log_status;
    Video video( path );
    Mat image;
    while( true )
    {
        log_status_mutex.lock();
        log_status = log_status_topic;
        log_status_mutex.unlock();
        if( log_status.video )
        {
            image_mutex.lock();
            image = image_topic.clone();
            image_mutex.unlock();
        }
        video.run( log_status.video, image );
        this_thread::sleep_for( milliseconds(50) );
    }
    cout << "[WARNING]: log node shutdown" << endl;
    return;
}

Video::Video(  string _path  ){
    path = _path;
}

void Video::run( bool flag, Mat image ){
    if( ! flag && video.isOpened() )
    {
        video.release();
        return;
    }
    else if( ! flag && ! video.isOpened() )
    {
        return;
    }
    else if( flag && ! video.isOpened() )
    {
        if( image.empty() )
            return;
        width = image.cols;
        height = image.rows;
        if( ! video.open( path + getCurrentTime() + ".avi", VideoWriter::fourcc( 'D', 'I', 'V', 'X' ), 15.0, Size( width , height ) ) ){
            cout << "[ERROR]: fail to open videowriter" << endl;
        }
    }
    else
    {
        if( ! write_mutex.try_lock_for( milliseconds( WRITE_VIDEO_WAIT_FOR_MS ) ) ){
            cout << "[WARNING]: write frame timeout" << endl;
            return;
        }
        write_mutex.unlock();
        RECORDING = {false};
        thread t( &Video::tryWriteFrame, this, image );
        t.detach();
        while ( RECORDING == false) {};
    }
}

void Video::tryWriteFrame( Mat _image ){
    Mat image = _image.clone();
    write_mutex.lock();
    RECORDING = {true};
    video.write( image );
    write_mutex.unlock();
    return;
}