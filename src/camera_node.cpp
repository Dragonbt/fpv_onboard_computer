#include "camera_node.hpp"

void cameraLoop( FileNode camera_config )
{
    int enable;
    int id, width, height;
    camera_config["ENABLE"] >> enable;
    camera_config["ID"] >> id;
    camera_config["WIDTH"] >> width;
    camera_config["HEIGHT"] >> height;
    if( enable == 0 )
    {
        cout << "[WARNING]: camera node disabled" << endl;
        return;
    }

    Mat image;
    int camera_status;

    //VideoCapture cap( id );
    Camera cap;
    if ( ! cap.init( id, width, height) ){
        cout << "[ERROR]: fail to init camera" << endl;
        camera_exception_mutex.lock();
        camera_exception_topic = -1;
        camera_exception_mutex.unlock();
        cout << "[WARNING]: camera node shut down" << endl;
        return;
    }
    while( true )
    {
        camera_status_mutex.lock();
        camera_status = camera_status_topic;
        camera_status_mutex.unlock();

        switch( camera_status )
        {
        case 0:
            cap.read( image );
            image_mutex.lock();
            image_topic = image.clone();
            image_mutex.unlock();
            break;
        default:
            break;
        }
    }
    cout << "[WARNING]: camera node shut down" << endl;
    return;
}

void cameraLoopTest()
{
    Mat image;
    while( true )
    {
        image_mutex.lock();
        image = image_topic.clone();
        image_mutex.unlock();
        if( image.empty() ){
            continue;
        }
        imshow( "image", image );
        waitKey(30);
    }
    return;
}

bool Camera::init( int id, int _width, int _height ){
    if ( ! cap.open( id ) ){
        cout << "[ERROR]: fail to open camera" << endl;
        return false;
    }
    cap.set( CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
    //cap.set( CAP_PROP_FOURCC, VideoWriter::fourcc('Y', 'U', 'Y', '2'));
    cap.set( CAP_PROP_FRAME_WIDTH, _width);
    cap.set( CAP_PROP_FRAME_HEIGHT, _height);
    cap.set( CAP_PROP_BUFFERSIZE, 2);
    //cout << cap.get(CV_CAP_PROP_BUFFERSIZE) << endl;
    width = cap.get( CAP_PROP_FRAME_WIDTH );
    height = cap.get( CAP_PROP_FRAME_HEIGHT);
    fps = cap.get(CAP_PROP_FPS);
    if ( width != _width || height != _height ){
        cout << "[ERROR]: set camera resolution fail" << endl;
        return false;
    }
    interval = 1000.0 / fps;
    for ( int i = 5; i > 0; i--)
        cap.grab();
    new_frame = true;
    last_grab = high_resolution_clock::now();
    thread updating_thread( &Camera::updating, this );
    updating_thread.detach();
    return true;
}

bool Camera::isOpened( void ){
    cap_mutex.lock();
    bool flag = cap.isOpened();
    cap_mutex.unlock();
    return flag;
}

bool Camera::read( Mat& image ){
    cap_mutex.lock();
    if ( ! cap.isOpened() ){
        cap_mutex.unlock();
        cout << "[ERROR]: cap closed" << endl;
        return false;
    }
    if ( intervalMs( high_resolution_clock::now(), last_grab ) > interval * 0.3 || new_frame == false ){
        cap.grab();
        new_frame = true;
        last_grab = high_resolution_clock::now();
    }
    bool flag = cap.retrieve( image );
    new_frame = false;
    cap_mutex.unlock();
    return flag;
}

void Camera::release( void ){
    cap_mutex.lock();
    cap.release();
    cap_mutex.unlock();
    return;
}

void Camera::updating( void ){
    while( true ){
        cap_mutex.lock();
        if ( ! cap.isOpened() ){
            cap_mutex.unlock();
            cout << "[ERROR]: cap closed" << endl;
            break;
        }
        if ( intervalMs(high_resolution_clock::now(), last_grab) > interval * 0.9 ){
            cap.grab();
            new_frame = true;
            last_grab = high_resolution_clock::now();
        }
        cap_mutex.unlock();
        this_thread::sleep_for(milliseconds( (int) interval ) );
    }
    return;
}