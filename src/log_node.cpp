#include "log_node.hpp"

void logLoop( FileNode log_config )
{
    int enable;
    string path;
    int width, height;
    log_config["ENABLE"] >> enable;
    log_config["LOG_FILE_PATH"] >> path;
    log_config["WIDTH"] >> width;
    log_config["HEIGHT"] >> height;
    if( enable == 0 )
    {
        cout << "[WARNING]: log node disabled" << endl;
        return;
    }

    GCCommand command;
    Log log(path);
    Video video(path, width, height);
    while( true )
    {
        command_mutex.lock();
        command = command_topic;
        command_mutex.unlock();
        if( command.log )
        {           
            log.open();
            log.writePositionVec();
            log.writeVelocityVec();
            log.writeAttitudeVec();
            log.writeInputVec();
        }
        else{
            log.close();
        }
        if( command.video )
        {
            video.open();
            video.writeImage();
        }
        else{
            video.close();
        }
        this_thread::sleep_for( milliseconds(50) );
    }
    cout << "[WARNING]: log node shutdown" << endl;
    return;
}

Log::Log( string path )
{
    log_path = path;
}

void Log::open()
{
    if( ! log_file.is_open() )
    {
        log_file.open( log_path + getCurrentTime() + "_log.csv" );
    }
}

void Log::close()
{
    if( log_file.is_open() )
    {
        log_file.close();
    }
}

void Log::writePositionVec()
{
    vector<PositionNED> position_vec;
    position_vec_log_mutex.lock();
    position_vec = position_vec_log_topic;
    position_vec_log_topic.clear();
    position_vec_log_mutex.unlock();
    for( unsigned int i=0; i < position_vec.size(); i++)
    {
        log_file << "PositionNED" << ", "
            << position_vec[i].north_m << ", "
            << position_vec[i].east_m << ", "
            << position_vec[i].down_m << ", "
            << position_vec[i].time_ms << endl;
    }
}

void Log::writeVelocityVec()
{
    vector<VelocityNED> velocity_vec;
    velocity_vec_log_mutex.lock();
    velocity_vec = velocity_vec_log_topic;
    velocity_vec_log_topic.clear();
    velocity_vec_log_mutex.unlock();
    for( unsigned int i=0; i < velocity_vec.size(); i++)
    {
        log_file << "VelocityNED" << ", "
            << velocity_vec[i].north_m_s << ", "
            << velocity_vec[i].east_m_s << ", "
            << velocity_vec[i].down_m_s << ", "
            << velocity_vec[i].time_ms << endl;
    }
}

void Log::writeAttitudeVec()
{
    vector<EulerAngle> euler_angle_vec;
    euler_angle_vec_log_mutex.lock();
    euler_angle_vec = euler_angle_vec_log_topic;
    euler_angle_vec_log_topic.clear();
    euler_angle_vec_log_mutex.unlock();
    for( unsigned int i=0; i < euler_angle_vec.size(); i++)
    {
        log_file << "EulerAngle" << ", "
            << euler_angle_vec[i].roll_deg << ", "
            << euler_angle_vec[i].pitch_deg << ", "
            << euler_angle_vec[i].yaw_deg << ", "
            << euler_angle_vec[i].time_ms << endl;
    }
}

void Log::writeInputVec()
{
    vector<Input> input_vec;
    input_vec_log_mutex.lock();
    input_vec = input_vec_log_topic;
    input_vec_log_topic.clear();
    input_vec_log_mutex.unlock();
    for( unsigned int i=0; i < input_vec.size(); i++)
    {
        log_file << "Input" << ", "
            << input_vec[i].forward_m_s << ", "
            << input_vec[i].right_m_s << ", "
            << input_vec[i].down_m_s << ", "
            << input_vec[i].yawspeed_deg_s << ", "
            << input_vec[i].time_ms << endl;
    }
}

Video::Video( string path, int img_width, int img_height )
{
    video_path = path;
    width = img_width;
    height = img_height;
}

void Video::open()
{
    if( ! writer.isOpened() )
    {
        writer.open( video_path + getCurrentTime() + string(".avi"), VideoWriter::fourcc('D', 'I', 'V', 'X'), 30, Size(width, height) );
    }
}

void Video::close()
{
    if( writer.isOpened() )
    {
        writer.release();
    }
}

void Video::writeImage()
{
    Mat image;
    if( writer.isOpened() )
    {
        image_mutex.lock();
        image = image_topic.clone();
        image_mutex.unlock();
        if( image.empty() )
            return;
        resize(image, image, Size(width, height));
        writer.write( image );
    }
}