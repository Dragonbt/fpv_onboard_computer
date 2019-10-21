#include "socket_nodes.hpp"

void sendLoop( FileNode send_config )
{
    int enable;
    string host;
    int port;
    double img_msg_freq;
    double img_msg_resize;
    int img_msg_quality;
    send_config["ENABLE"] >> enable;
    send_config["HOST"] >> host;
    send_config["PORT"] >> port;
    send_config["IMG_MSG_FREQ"] >> img_msg_freq;
    send_config["IMG_MSG_RESIZE"] >> img_msg_resize;
    send_config["IMG_MSG_QUALITY"] >> img_msg_quality;
    if( enable == 0 )
    {
        cout << "[WARNING]: send node disabled" << endl;
        return;
    }
    if( ! socketInit() )
    {
        cout << "[WARNING]: send node shutdown" << endl;
        return;
    }

    bzero(&send_to_addr, sizeof(send_to_addr));
    if( inet_pton( AF_INET, host.c_str(), &send_to_addr.sin_addr ) <= 0 )
    {
        cout << "[WARNING]: " + string(strerror(errno)) << endl;
        return;
    }
    send_to_addr.sin_family = AF_INET; 
    send_to_addr.sin_port = htons( port );
    
    while( true )
    {
        sendHeartBeat();
        sendImg(img_msg_resize, img_msg_quality);
        sendPosition();
        sendVelocity();
        sendAttitude();
        this_thread::sleep_for( milliseconds( 50 ) );
    }
    cout << "[WARNING]: send node shutdown" << endl;
    return;
}

void recvLoop( FileNode recv_config )
{
    int enable;
    string host;
    int port;
    recv_config["ENABLE"] >> enable;
    recv_config["HOST"] >> host;
    recv_config["PORT"] >> port;
    if( enable == 0 )
    {
        cout << "[WARNING]: recv node disabled" << endl;
        return;
    }
    if( ! socketInit() )
    {
        cout << "[WARNING]: recv node shutdown" << endl;
        return;
    }

    bzero(&recv_from_addr, sizeof(recv_from_addr));
    if( inet_pton( AF_INET, host.c_str(), &recv_from_addr.sin_addr ) <= 0 )
    {
        cout << "[WARNING]: " + string(strerror(errno)) << endl;
        return;
    }
    recv_from_addr.sin_family = AF_INET; 
    recv_from_addr.sin_port = htons( port );

    char msg[MAX_MSG_LENGTH];
    int msg_length;
    struct sockaddr_in addr;
    socklen_t addr_len  = sizeof recv_from_addr;
    while (true)
    {
        msg_length = (int)recvfrom( fd, msg, sizeof msg, 0, (struct sockaddr *)&addr, &addr_len );
        if( msg_length < 0 && errno == EAGAIN)
        {
            this_thread::sleep_for( milliseconds( 30 ) );
            continue;
        }
        else if( msg_length < 0 )
        {
            cout << "[WARNING]: " + string(strerror(errno)) << endl;
            continue;
        }
        recvMsg( msg, msg_length, addr );
    }
    cout << "[WARNING]: recv node shutdown" << endl;
    return;
}

bool socketInit()
{
    fd_mutex.lock();
    if( fd < 0 )
    {
        if ( ( fd = socket(AF_INET, SOCK_DGRAM, 0 ) ) < 0 )
        {
            cout << "[WARNING]: " + string(strerror(errno)) << endl;
            fd_mutex.unlock();
            return false;
        }
        if ( fcntl( fd, F_SETFL, O_NONBLOCK ) < 0 )
        {
            cout << "[WARNING]: " + string(strerror(errno)) << endl;
            fd_mutex.unlock();
            return false;
        }
    }
    fd_mutex.unlock();
    return true;
}

void sendMsg( uint8_t msg_type, uint16_t length, void* buffer )
{
    if ( length > MAX_MSG_LENGTH + 7 ){
        cout << "[ERROR]: buffer size overflow" << endl;
        return;
    }
    uint16_t head = HEAD;
    uint16_t tail = TAIL;
    char msg[ sizeof head + sizeof msg_type + sizeof length + length + sizeof tail ];
    char* p = msg;
    memcpy(p, &head, sizeof head);
    p = p + sizeof head;
    memcpy(p, &msg_type, sizeof msg_type);
    p = p + sizeof msg_type;
    memcpy(p, &length, sizeof length);
    p = p + sizeof length;
    memcpy(p, buffer, length);
    p = p + length;
    memcpy(p, &tail, sizeof tail);
    if ( (int)sendto( fd, msg, sizeof msg, 0, (struct sockaddr *)&send_to_addr, sizeof send_to_addr ) < 0 )
    {
        cout << "[WARNING]: " + string(strerror(errno)) << endl;
    }
    return;
}

void recvMsg( char* msg, int msg_length, sockaddr_in addr )
{
    uint8_t msg_type;
    uint16_t head, tail, len;
    if ( addr.sin_addr.s_addr != recv_from_addr.sin_addr.s_addr || addr.sin_port != recv_from_addr.sin_port )
        return;
    if( msg_length < 8 || msg_length > MAX_MSG_LENGTH )
        return;
    char *p = msg;
    memcpy( &head, p, sizeof head );
    p = p + sizeof head;
    if( head != HEAD )
        return;
    memcpy( &msg_type, p, sizeof msg_type );
    p = p + sizeof msg_type;
    memcpy( &len, p, sizeof len );
    p = p + sizeof len;
    if( msg_length != 7 + (int)len )
        return;
    char buffer[len];
    memcpy( buffer, p, len );
    p = p + len;
    memcpy( &tail, p, sizeof tail );
    if( tail != TAIL )
        return;
    bool flag;
    switch(msg_type)
    {
        case 0:
            memcpy( &flag, buffer, sizeof flag );
            command_mutex.lock();
            command_topic.arm = flag;
            command_mutex.unlock();
            break;
        case 1:
            memcpy( &flag, buffer, sizeof flag );
            command_mutex.lock();
            command_topic.takeoff = flag;
            command_mutex.unlock();
            break;
        case 2:
            memcpy( &flag, buffer, sizeof flag );
            command_mutex.lock();
            command_topic.land = flag;
            command_mutex.unlock();
            break;
        case 3:
            memcpy( &flag, buffer, sizeof flag );
            command_mutex.lock();
            command_topic.up = flag;
            command_mutex.unlock();
            break;
        case 4:
            memcpy( &flag, buffer, sizeof flag );
            command_mutex.lock();
            command_topic.down = flag;
            command_mutex.unlock();
            break;
        case 5:
            memcpy( &flag, buffer, sizeof flag );
            command_mutex.lock();
            command_topic.video = flag;
            command_mutex.unlock();
            break;
        case 6:
            memcpy( &flag, buffer, sizeof flag );
            command_mutex.lock();
            command_topic.log = flag;
            command_mutex.unlock();
            break;
        case 7:
            memcpy( &flag, buffer, sizeof flag );
            command_mutex.lock();
            command_topic.forward = flag;
            command_mutex.unlock();
            break;
        case 8:
            memcpy( &flag, buffer, sizeof flag );
            command_mutex.lock();
            command_topic.backward = flag;
            command_mutex.unlock();
            break;
        case 9:
            memcpy( &flag, buffer, sizeof flag );
            command_mutex.lock();
            command_topic.left = flag;
            command_mutex.unlock();
            break;
        case 10:
            memcpy( &flag, buffer, sizeof flag );
            command_mutex.lock();
            command_topic.right = flag;
            command_mutex.unlock();
            break;
        case 11:
            memcpy( &flag, buffer, sizeof flag );
            command_mutex.lock();
            command_topic.yaw_pos = flag;
            command_mutex.unlock();
            break;
        case 12:
            memcpy( &flag, buffer, sizeof flag );
            command_mutex.lock();
            command_topic.yaw_neg = flag;
            command_mutex.unlock();
            break;
        default:
            break;
    }
}

bool compress( Mat image, double resize_k, int quality, vector<uchar>& img_buffer)
{
    vector< int > jpeg_quality{ IMWRITE_JPEG_QUALITY, quality };
    resize( image, image, Size(), resize_k, resize_k );
    //cvtColor( frame, gray_frame, COLOR_BGR2GRAY );
    return imencode(".jpeg", image, img_buffer, jpeg_quality);
}

void sendHeartBeat()
{
    bool heart_beat = true;
    sendMsg( HEART_BEAT_MSG, 1, &heart_beat);
    return;
}

void sendImg( double img_msg_resize, int img_msg_quality )
{
    Mat image;
    vector< uchar > img_buffer;
    image_mutex.lock();
    image = image_topic.clone();
    image_mutex.unlock();
    if( ! image.empty() && compress( image, img_msg_resize, img_msg_quality, img_buffer ) ){
        //cout << img_buffer.size() << endl;
        sendMsg( IMG_MSG, (uint16_t) img_buffer.size(), img_buffer.data() );
        img_buffer.clear();
    }
    return;
}

void sendPosition()
{
    vector<PositionNED> position_vec;
    position_vec_mutex.lock();
    position_vec = position_vec_topic;
    position_vec_topic.clear();
    position_vec_mutex.unlock();
    if( ! position_vec.empty() )
    {
        sendMsg( POSITION_MSG, (uint16_t) ( position_vec.size() * sizeof(PositionNED) ), position_vec.data() );
        position_vec.clear();
    }
    return;
}

void sendVelocity()
{
    vector<VelocityNED> velocity_vec;
    velocity_vec_mutex.lock();
    velocity_vec = velocity_vec_topic;
    velocity_vec_topic.clear();
    velocity_vec_mutex.unlock();
    if( ! velocity_vec.empty() )
    {
        sendMsg( VELOCITY_MSG, (uint16_t) ( velocity_vec.size() * sizeof(VelocityNED) ), velocity_vec.data() );
        velocity_vec.clear();
    }
    return;
}

void sendAttitude()
{
    vector<EulerAngle> euler_angle_vec;
    euler_angle_vec_mutex.lock();
    euler_angle_vec = euler_angle_vec_topic;
    euler_angle_vec_topic.clear();
    euler_angle_vec_mutex.unlock();
    if( ! euler_angle_vec.empty() )
    {
        sendMsg( ATTITUDE_MSG, (uint16_t) ( euler_angle_vec.size() * sizeof(EulerAngle) ), euler_angle_vec.data() );
        euler_angle_vec.clear();
    }
    return;
}