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
        cout << "[WARNING]: udp send node disabled" << endl;
        return;
    }

    int udp_fd;
    Mat image;
    vector< uchar > img_buffer;
    int64_t timepoint_ms;
    if ( ! udpClientInit( udp_fd, host.c_str(), port) ){
        socket_exception_mutex.lock();
        socket_exception_topic = -1;
        socket_exception_mutex.unlock();
        cout << "[WARNING]: " + string(strerror(errno)) << endl;
        cout << "[WARNING]: no udp connection to GC" << endl;
        cout << "[WARNING]: send node shutdown" << endl;
        return;
    }
    while( true )
    {
        image_mutex.lock();
        image = image_topic.clone();
        image_mutex.unlock();
        if( ! image.empty() && compress( image, img_msg_resize, img_msg_quality, img_buffer ) ){
            timepoint_ms = intervalMs( high_resolution_clock::now(), init_timepoint );
            //cout << img_buffer.size() << endl;
            sendMsg( udp_fd, timepoint_ms, 1, (uint16_t) img_buffer.size(), img_buffer.data() );
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
        cout << "[WARNING]: tcp send node disabled" << endl;
        return;
    }

    int tcp_fd;
    uint16_t head, tail;
    uint8_t msg_type;
    uint16_t length;
    int64_t timestamp_ms;
    bool video;
    if ( ! tcpClientInit(tcp_fd, host.c_str(), port) ){
        socket_exception_mutex.lock();
        socket_exception_topic = -2;
        socket_exception_mutex.unlock();
        cout << "[WARNING]: " + string(strerror(errno)) << endl;
        cout << "[WARNING]: no tcp connection to GC" << endl;
        cout << "[WARNING]: recv node shutdown" << endl;
        return;
    }
    while (true)
    {
        try{
            if (!recvAll(tcp_fd, &head, sizeof head))
            {
                cout << "[WARNING]: " + string(strerror(errno)) << endl;
                continue;
            }
            if (head != HEAD)
                continue;
            if ( recvAll(tcp_fd, &msg_type, sizeof msg_type) 
                && recvAll(tcp_fd, &length, sizeof length) == false)
            {
                cout << "[WARNING]: " + string(strerror(errno)) << endl;
                continue;
            }
            char buffer[length];
            if (recvAll(tcp_fd, buffer, length) && recvAll(tcp_fd, &timestamp_ms, sizeof timestamp_ms) && recvAll(tcp_fd, &tail, sizeof tail) == false)
            {
                cout << "[WARNING]: " + string(strerror(errno)) << endl;
                continue;
            }
            if (tail != TAIL)
                continue;
            switch (msg_type)
            {
            case 5:
                memcpy(&video, buffer, sizeof video);
                log_status_mutex.lock();
                log_status_topic.video = video;
                log_status_mutex.unlock();
                break;
            default:
                break;
            }
        }
        catch (int exception)
        {
            if (exception == PEER_SHUTDOWN)
            {
                socket_exception_mutex.lock();
                socket_exception_topic = -3;
                socket_exception_mutex.unlock();
                shutdown(tcp_fd, 2);
                close(tcp_fd);
                cout << "[WARNING]: try reconnect" << endl;
                cout << "[WARNING]: no tcp connection to GC" << endl;
                while ( ! tcpClientInit(tcp_fd, host.c_str(), port) ){
                    close(tcp_fd);
                }
                socket_exception_mutex.lock();
                socket_exception_topic = 0;
                socket_exception_mutex.unlock();
                cout << "[LOGGING]: new tcp connection established" << endl;
            }
        }
        this_thread::sleep_for( milliseconds( 30 ) );
    }
    cout << "[WARNING]: send node shutdown" << endl;
    return;
}

bool sendMsg( int fd, int64_t timepoint_ms, uint8_t msg_type, uint16_t length, void* buffer )
{
    if ( length > MAX_MSG_CONTENT_LENGTH ){
        cout << "[ERROR]: buffer size overflow" << endl;
        return false;
    }
    uint16_t head = HEAD;
    uint16_t tail = TAIL;
    if (  sendAll( fd, &head, sizeof head )
        && sendAll( fd, &msg_type, sizeof msg_type )
        && sendAll( fd, &length, sizeof length )
        && sendAll( fd, buffer, length )
        && sendAll( fd, &timepoint_ms, sizeof timepoint_ms )
        && sendAll( fd, &tail, sizeof tail ) == false )
    {
        cout << "[WARNING]: " + string(strerror(errno)) << endl;
        return false;
    }
    return true;
}

bool compress( Mat image, double resize_k, int quality, vector<uchar>& img_buffer)
{
    vector< int > jpeg_quality{ IMWRITE_JPEG_QUALITY, quality };
    resize( image, image, Size(), resize_k, resize_k );
    //cvtColor( frame, gray_frame, COLOR_BGR2GRAY );
    return imencode(".jpeg", image, img_buffer, jpeg_quality);
}

bool udpServerInit( int& server_fd, const char* host, const int port){
    if ( ( server_fd = socket(AF_INET, SOCK_DGRAM, 0 ) ) < 0 )
        return false;

    struct sockaddr_in server_address, client_address;
    if( inet_pton( AF_INET, host, &server_address.sin_addr ) <= 0 ) 
        return false;
    server_address.sin_family = AF_INET;
    server_address.sin_port = htons(port);

    if ( bind( server_fd, (struct sockaddr *)& server_address, sizeof( server_address ) ) < 0 )
        return false;
    return true;
}

bool udpClientInit( int& client_fd, const char* host, const int port ){
    //set to UDP
    if ( ( client_fd = socket(AF_INET, SOCK_DGRAM, 0 ) ) < 0 )
        return false;
    
    //set to non-blocking
    if ( fcntl( client_fd, F_SETFL, O_NONBLOCK | O_WRONLY ) < 0 )
        return false;
    //if ( fcntl( client_fd, F_SETFL, O_WRONLY ) < 0 )
    //    return false;

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
    return true;
}

bool tcpServerInit( int& server_fd, int& client_fd, const char* host, const int port){
    if ( ( server_fd = socket(AF_INET, SOCK_STREAM, 0 ) ) < 0 )
        return false;
    
    struct sockaddr_in server_address, client_address;
    if( inet_pton( AF_INET, host, &server_address.sin_addr ) <= 0 ) 
        return false;
    server_address.sin_family = AF_INET;
    server_address.sin_port = htons(port);

    if ( bind( server_fd, (struct sockaddr *)& server_address, sizeof( server_address ) ) < 0 )
        return false;
    if ( listen( server_fd, 3 ) < 0 )
        return false;
    
    int addrlen = sizeof( server_address );
    if ( ( client_fd = accept( server_fd, (struct sockaddr *)& client_address, (socklen_t *)& addrlen ) ) < 0 )
        return false;
    return true;
}

bool tcpClientInit( int& client_fd, const char* host, const int port ){   
    if ( ( client_fd = socket( AF_INET, SOCK_STREAM, 0 ) ) < 0 ) 
        return false;

    if ( fcntl( client_fd, F_SETFL, O_RDONLY ) < 0 )
        return false;
    
    struct sockaddr_in server_address;
    if( inet_pton( AF_INET, host, &server_address.sin_addr ) <= 0 ) 
        return false;
    server_address.sin_family = AF_INET; 
    server_address.sin_port = htons( port ); 
    
    int send_buffer_size = 32 * 1024 * 1024;
    if ( setsockopt( client_fd, SOL_SOCKET, SO_SNDBUF, &send_buffer_size, sizeof send_buffer_size ) < 0 )
        return false;
    
    if ( connect( client_fd, (struct sockaddr *)& server_address, sizeof( server_address ) ) < 0 ) 
        return false; 
    return true;
}

bool sendAll( int socket, void *buffer, size_t length )
{
    char * ptr = (char *) buffer;
    size_t offset = 0;
    size_t bytes_sent = 0;
    while (offset < length)
    {
        bytes_sent = send(socket, ptr + offset, length - offset, MSG_DONTWAIT );
        if (bytes_sent < 0){
            return false;
        }
        offset += bytes_sent;
    }
    return true;
}

bool recvAll( int socket, void *buffer, size_t length )
{
    char * ptr = (char *) buffer;
    size_t offset = 0;
    size_t bytes_recv = 0;
    while (offset < length)
    {
        bytes_recv = recv(socket, ptr + offset, length - offset, 0);
        if ( bytes_recv == 0){
            throw PEER_SHUTDOWN;
        }
        else if (bytes_recv < 0){
            return false;
        }
        offset += bytes_recv;
    }
    return true;
}