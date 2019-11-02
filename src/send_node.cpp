#include "send_node.hpp"

struct sockaddr_in send_to_addr;
int64_t sent_position_ms = 0, sent_velocity_ms = 0, sent_attitude_ms = 0, sent_target_ms = 0, sent_position_body_ms = 0,sent_status_ms=0;

void sendLoop( FileNode send_config )
{
    int enable;
    string host;
    int port;
    double img_msg_resize;
    int img_msg_quality;
    bool gray;
    send_config["ENABLE"] >> enable;
    send_config["HOST"] >> host;
    send_config["PORT"] >> port;
    send_config["GRAY_IMG"] >> gray;
    send_config["IMG_MSG_RESIZE"] >> img_msg_resize;
    send_config["IMG_MSG_QUALITY"] >> img_msg_quality;
    if( enable == 0 )
    {
        cout << "[WARNING]: send node disabled" << endl;
        return;
    }
    if( ! sendSocketInit() )
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
    
    high_resolution_clock::time_point t0 = high_resolution_clock::now();
    while( true )
    {
        if( intervalMs(high_resolution_clock::now(), t0) > 300 )
        {
            sendHeartBeat();
            //sendPosition();
            //sendPositionBody();
            //sendVelocity();
            sendAttitude();
            sendReference();
            sendInputAttitude();
            sendStatus();
            sendString();
            sendTarget();
            t0 = high_resolution_clock::now();
        }
        else{
            sendImg(gray, img_msg_resize, img_msg_quality);
        }
        this_thread::sleep_for( milliseconds( 60 ) );
    }
    cout << "[WARNING]: send node shutdown" << endl;
    return;
}

bool sendSocketInit()
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

bool compress( Mat image, bool gray, double resize_k, int quality, vector<uchar>& img_buffer)
{
    vector< int > jpeg_quality{ IMWRITE_JPEG_QUALITY, quality };
    if( gray )
    {
        cvtColor( image, image, COLOR_BGR2GRAY );
    }
    resize( image, image, Size(), resize_k, resize_k );
    return imencode(".jpeg", image, img_buffer, jpeg_quality);
}

void sendHeartBeat()
{
    bool heart_beat = true;
    sendMsg( HEART_BEAT_MSG, 1, &heart_beat);
    return;
}

void sendImg( bool gray, double img_msg_resize, int img_msg_quality )
{
    Mat image, empty;
    vector< uchar > img_buffer;
    image_mutex.lock();
    image = image_topic.clone();
    image_topic = empty;
    image_mutex.unlock();
    if( ! image.empty() && compress( image, gray, img_msg_resize, img_msg_quality, img_buffer ) ) {
        //cout << img_buffer.size() << endl;
        sendMsg( IMG_MSG, (uint16_t) img_buffer.size(), img_buffer.data() );
        img_buffer.clear();
    }
    return;
}

void sendPosition()
{
    vector<PositionNED> position;
    PositionNED pos;
    position_mutex.lock();
    for( size_t i=0; i < position_topic.size(); i++ )
    {
        pos = position_topic[i];
        if( pos.time_ms > sent_position_ms )
        {
            position.push_back( pos );
        }
    }
    position_mutex.unlock();
    if( ! position.empty() )
    {
        sendMsg( POSITION_MSG, (uint16_t) ( position.size() * sizeof(PositionNED) ), position.data() );
        sent_position_ms = position.back().time_ms;
        position.clear();
    }
    return;
}

void sendPositionBody()
{
    vector<PositionBody> position;
    PositionBody pos;
    position_body_mutex.lock();
    for( size_t i=0; i < position_body_topic.size(); i++ )
    {
        pos = position_body_topic[i];
        if( pos.time_ms > sent_position_body_ms )
        {
            position.push_back( pos );
        }
    }
    position_body_mutex.unlock();
    if( ! position.empty() )
    {
        sendMsg( POSITION_BODY_MSG, (uint16_t) ( position.size() * sizeof(PositionBody) ), position.data() );
        sent_position_ms = position.back().time_ms;
        position.clear();
    }
    return;
}
void sendVelocity()
{
    vector<VelocityNED> velocity;
    VelocityNED vel;
    velocity_mutex.lock();
    for( size_t i=0; i < velocity_topic.size(); i++ )
    {
        vel = velocity_topic[i];
        if( vel.time_ms > sent_velocity_ms )
        {
            velocity.push_back( vel );
        }
    }
    velocity_mutex.unlock();
    if( ! velocity.empty() )
    {
        sendMsg( VELOCITY_MSG, (uint16_t) ( velocity.size() * sizeof(VelocityNED) ), velocity.data() );
        sent_velocity_ms = velocity.back().time_ms;
        velocity.clear();
    }
    return;
}

void sendAttitude()
{
    vector<EulerAngle> attitude;
    EulerAngle att;
    attitude_mutex.lock();
    for( size_t i=0; i < attitude_topic.size(); i++ )
    {
        att = attitude_topic[i];
        if( att.time_ms > sent_attitude_ms )
        {
            attitude.push_back( att );
        }
    }
    attitude_mutex.unlock();
    if( ! attitude.empty() )
    {
        sendMsg( ATTITUDE_MSG, (uint16_t) ( attitude.size() * sizeof(EulerAngle) ), attitude.data() );
        sent_attitude_ms = attitude.back().time_ms;
        attitude.clear();
    }
    return;
}

void sendStatus()
{
    vector<Status> status;
    status_mutex.lock();
    status = status_topic;
    status_topic.clear();
    status_mutex.unlock();
    if( ! status.empty() )
    {
        sendMsg( STATUS_MSG, (uint16_t) sizeof(Status), &status.back() );
    }
    return;
}

void sendString()
{
    vector<string> string;
    string_mutex.lock();
    string = string_topic;
    string_topic.clear();
    string_mutex.unlock();
    if( ! string.empty() )
    {
        for( size_t i=0; i < string.size(); i++)
        {
            sendMsg( LOG_MSG, (uint16_t) string[i].size(), const_cast<char*>(string[i].data()) );
        }
    }
    return;
}

void sendInputAttitude()
{
    vector<InputAttitude> input_attitude;
    input_attitude_mutex.lock();
    input_attitude = input_attitude_topic;
    input_attitude_topic.clear();
    input_attitude_mutex.unlock();
    if( ! input_attitude.empty() )
    {
        sendMsg( INPUT_ATTITUDE_MSG, (uint16_t) ( input_attitude.size() * sizeof(InputAttitude) ), input_attitude.data() );
    }
    return;
}

void sendControlStatus()
{
    vector<ControlStatus> control_status;
    ControlStatus status;
    control_status_mutex.lock();
    for( size_t i=0; i < control_status_topic.size(); i++ )
    {
        status = control_status_topic[i];
        if( status.time_ms > sent_status_ms )
        {
            control_status.push_back( status );
        }
    }
    control_status_mutex.unlock();
    if( !control_status.empty() )
    {
        sendMsg(CONTROL_STATUS_MSG, (uint16_t) ( control_status.size() * sizeof(ControlStatus) ), control_status.data());
        sent_status_ms = control_status.back().time_ms;
        control_status.clear();
    }
    return;
}
void sendReference()
{
    vector<Reference> reference;
    reference_mutex.lock();
    reference = reference_topic;
    reference_topic.clear();
    reference_mutex.unlock();
    if( ! reference.empty() )
    {
        sendMsg( REFERENCE_MSG, (uint16_t) (reference.size() * sizeof(Reference) ), reference.data() );
    }
    return;
}

void sendTarget()
{
    vector<DetectionResult> target;
    DetectionResult tar;
    target_mutex.lock();
    for( size_t i=0; i < target_topic.size(); i++ )
    {
        tar = target_topic[i];
        if( tar.time_ms > sent_target_ms )
        {
            target.push_back( tar );
        }
    }
    target_mutex.unlock();
    if( ! target.empty() )
    {
        sendMsg( TARGET_MSG, (uint16_t) ( target.size() * sizeof(DetectionResult) ), target.data() );
        sent_target_ms = target.back().time_ms;
        target.clear();
    }
    return;
}