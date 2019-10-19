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
    ofstream position_log, velocity_log, attitude_log, u_log;
    position_log.open(path + getCurrentTime() + "_position.csv");
    velocity_log.open(path + getCurrentTime() + "_velocity.csv");
    attitude_log.open(path + getCurrentTime() + "_attitude.csv");
    u_log.open(path + getCurrentTime() + "_u.csv" );

    Mat image;
    vector<PositionNED> position_vec;
    vector<VelocityNED> velocity_vec;
    vector<EulerAngle> euler_angle_vec;
    vector<Input> u_vec;
    while( true )
    {
        log_status_mutex.lock();
        log_status = log_status_topic;
        log_status_mutex.unlock();
        /*
        if( log_status.video )
        {
            image_mutex.lock();
            image = image_topic.clone();
            image_mutex.unlock();
        }
        video.run( log_status.video, image );
        */
        if( log_status.log )
        {
            position_vec_mutex.lock();
            position_vec = position_vec_topic;
            position_vec_topic.clear();
            position_vec_mutex.unlock();
            for( int i=0; i < position_vec.size(); i++)
            {
                position_log << position_vec[i].north_m << ", "
                    << position_vec[i].east_m << ", "
                    << position_vec[i].down_m << ", "
                    << position_vec[i].time_ms << endl;
            }
            velocity_vec_mutex.lock();
            velocity_vec = velocity_vec_topic;
            velocity_vec_topic.clear();
            velocity_vec_mutex.unlock();
            for( int i=0; i < velocity_vec.size(); i++)
            {
                velocity_log << velocity_vec[i].north_m_s << ", "
                    << velocity_vec[i].east_m_s << ", "
                    << velocity_vec[i].down_m_s << ", "
                    << velocity_vec[i].time_ms << endl;
            }
            euler_angle_vec_mutex.lock();
            euler_angle_vec = euler_angle_vec_topic;
            euler_angle_vec_topic.clear();
            euler_angle_vec_mutex.unlock();
            for( int i=0; i < euler_angle_vec.size(); i++)
            {
                attitude_log << euler_angle_vec[i].roll_deg << ", "
                    << euler_angle_vec[i].pitch_deg << ", "
                    << euler_angle_vec[i].yaw_deg << ", "
                    << euler_angle_vec[i].time_ms << endl;
            }
            u_vec_mutex.lock();
            u_vec = u_vec_topic;
            u_vec_topic.clear();
            u_vec_mutex.unlock();
            for( int i=0; i < u_vec.size(); i++)
            {
                u_log << u_vec[i].forward_m_s << ", "
                    << u_vec[i].right_m_s << ", "
                    << u_vec[i].down_m_s << ", "
                    << u_vec[i].yaw_speed_deg_s << ", "
                    << u_vec[i].time_ms << endl;
            }
        }
        this_thread::sleep_for( milliseconds(30) );
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