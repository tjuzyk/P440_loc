#ifndef THREAD_MAPPING_HPP
#define THREAD_MAPPING_HPP

#include "basic_function.hpp"
#include "mapping/mapping_abstract.hpp"

namespace uavos{

class Mapping_Controller{
public:
    explicit Mapping_Controller():m_mapping_thread(), m_nh("~mapping"){    //被explicit关键字修饰的类构造函数，不能进行自动地隐式类型转换，只能显式地进行类型转换
        m_stop_thread = false;
        m_nh.setCallbackQueue(&m_mapping_callback_queue);  //自定义了一个回调队列
    }
    ~Mapping_Controller(){
        m_stop_thread = true;
        if(m_mapping_thread.joinable()){
            m_mapping_thread.join();
        }
    }

    void start();

private:
    ros::NodeHandle m_nh;
    ros::CallbackQueue m_mapping_callback_queue;     // 自定义了一个回调队列 该类有两种方式调用回调函数callAvailable() and callOne()

    bool m_stop_thread;
    boost::thread m_mapping_thread;


    void thread_mapping();

};


}


#endif // THREAD_MAPPING_HPP
