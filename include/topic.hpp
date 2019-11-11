#ifndef _TOPIC_
#define _TOPIC_

#include <deque>
#include <vector>
#include <chrono>
#include <iostream>
#include <utility>
#include <mutex> 

template<class Struct>
class Topic{
    public:
    Topic(std::mutex& mtx, std::chrono::high_resolution_clock::time_point init_timepoint=std::chrono::high_resolution_clock::now(), size_t max_size=1);
    ~Topic();
    void update(Struct content);
    bool latest(int64_t& timestamp, Struct& content);
    void recent(std::vector< std::pair<int64_t, Struct> >& topic_vector, int64_t& timestamp);
    void clear(void);

    private:
    std::mutex& topic_mutex; 
    std::chrono::high_resolution_clock::time_point init_time_point;
    size_t max_size;
    std::deque< std::pair<int64_t, Struct> > topic_deque;
    int64_t timestamp(void);
};

template<class Struct>
Topic<Struct>::Topic(std::mutex& mtx, std::chrono::high_resolution_clock::time_point init_timepoint, size_t max_size):
    topic_mutex(mtx)
{
    this->init_time_point = init_timepoint;
    this->max_size = std::max(max_size, (size_t)1);
    return;
}

template<class Struct>
Topic<Struct>::~Topic()
{
    return;
}

template<class Struct>
void Topic<Struct>::update(const Struct content)
{
    std::pair<int64_t, Struct> topic = std::make_pair(timestamp(), content);
    topic_mutex.lock();
    if( topic_deque.size() >= max_size )
    {
        for(size_t i = 0; i < 1 + topic_deque.size() - max_size; i++)
        {
            topic_deque.pop_front();
        }
    }
    topic_deque.push_back(topic);
    topic_mutex.unlock();
    return;
}

template<class Struct>
bool Topic<Struct>::latest(int64_t& timestamp, Struct& content)
{
    bool valid;
    std::pair<int64_t, Struct> topic;
    topic_mutex.lock();
    if( ! topic_deque.empty() )
    {
        topic = topic_deque.back();
        timestamp = topic.first;
        content = topic.second;
        valid = true;
    }
    else{
        valid = false;
    }
    topic_mutex.unlock();
    return valid;
}

template<class Struct>
void Topic<Struct>::recent(std::vector< std::pair<int64_t, Struct> >& topic_vector, int64_t& timestamp)
{
    std::pair<int64_t, Struct> topic;
    topic_mutex.lock();
    if( ! topic_deque.empty() )
    {
       for(size_t i = 0; i < topic_deque.size(); i++)
        {
            topic = topic_deque[i];
            if( topic.first > timestamp )
            {
                timestamp = topic.first;
                topic_vector.push_back(topic);
            }
        }
    }
    topic_mutex.unlock();
    return;
}

template<class Struct>
void Topic<Struct>::clear( void )
{
    topic_mutex.lock();
    topic_deque.clear();
    topic_mutex.unlock();
    return;
}

template<class Struct>
int64_t Topic<Struct>::timestamp(void)
{
    std::chrono::duration<double> time_span = std::chrono::high_resolution_clock::now() - init_time_point;
    std::chrono::milliseconds d = std::chrono::duration_cast<std::chrono::milliseconds>(time_span);
    return d.count();
}

#endif
