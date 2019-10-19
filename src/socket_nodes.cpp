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

    Mat image;
    vector< uchar > img_buffer;
    while( true )
    {
        image_mutex.lock();
        image = image_topic.clone();
        image_mutex.unlock();
        if( ! image.empty() && compress( image, img_msg_resize, img_msg_quality, img_buffer ) ){
            //cout << img_buffer.size() << endl;
            sendMsg( 1, (uint16_t) img_buffer.size(), img_buffer.data() );
            img_buffer.clear();
        }
        this_thread::sleep_for( milliseconds( 30 ) );
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
    bool arm, takeoff, land;
    bool video;
    bool log;
    bool up, down, forward, backward, left, right;
    switch(msg_type)
    {
        case 0:
            memcpy( &arm, buffer, sizeof arm );
            command_mutex.lock();
            command_topic.arm = arm;
            command_mutex.unlock();
            break;
        case 1:
            memcpy( &takeoff, buffer, sizeof takeoff );
            command_mutex.lock();
            command_topic.takeoff = takeoff;
            command_mutex.unlock();
            break;
        case 2:
            memcpy( &land, buffer, sizeof land );
            command_mutex.lock();
            command_topic.land = land;
            command_mutex.unlock();
            break;
        case 3:
            memcpy( &up, buffer, sizeof up );
            command_mutex.lock();
            command_topic.up = up;
            command_mutex.unlock();
            break;
        case 4:
            memcpy( &down, buffer, sizeof down );
            command_mutex.lock();
            command_topic.down = down;
            command_mutex.unlock();
            break;
        case 5:
            memcpy( &video, buffer, sizeof video );
            log_status_mutex.lock();
            log_status_topic.video = video;
            log_status_mutex.unlock();
            break;
        case 6:
            memcpy( &log, buffer, sizeof log );
            log_status_mutex.lock();
            log_status_topic.log = log;
            log_status_mutex.unlock();
            break;
        case 7:
            memcpy( &forward, buffer, sizeof forward );
            command_mutex.lock();
            command_topic.forward = forward;
            command_mutex.unlock();
            break;
        case 8:
            memcpy( &backward, buffer, sizeof backward );
            command_mutex.lock();
            command_topic.backward = backward;
            command_mutex.unlock();
            break;
        case 9:
            memcpy( &left, buffer, sizeof left );
            command_mutex.lock();
            command_topic.left = left;
            command_mutex.unlock();
            break;
        case 10:
            memcpy( &right, buffer, sizeof right );
            command_mutex.lock();
            command_topic.right = right;
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

bool udpClientInit( int& client_fd, const char* host, const int port ){
    //set to UDP
    if ( ( client_fd = socket(AF_INET, SOCK_DGRAM, 0 ) ) < 0 )
        return false;
    //set send buffer size
    int send_buffer_size = 32 * 1024 * 1024; //32MB
    if ( setsockopt( client_fd, SOL_SOCKET, SO_SNDBUF, &send_buffer_size, sizeof send_buffer_size ) < 0 )
        return false;
    
    //connect to server
    struct sockaddr_in server_address;
    if( inet_pton( AF_INET, host, &server_address.sin_addr ) <= 0 ) 
        return false;
    server_address.sin_family = AF_INET; 
    server_address.sin_port = htons( port );
    if ( connect( client_fd, (struct sockaddr*)& server_address, sizeof( server_address ) ) < 0 )
        return false;
    //set to non-blocking
    if ( fcntl( client_fd, F_SETFL, O_NONBLOCK | O_WRONLY ) < 0 )
        return false;
    return true;
}
