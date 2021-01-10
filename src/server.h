#pragma once

#include <cstdlib>
#include <iostream>
#include <memory>
#include <utility>
#include <boost/asio.hpp>
#include <functional>

using namespace std::placeholders;
using boost::asio::ip::tcp;

const int test_port = 3000;
class session
    : public std::enable_shared_from_this<session>
{
public:
    session(tcp::socket socket);

    void start();

    void setCallback(std::function<void(char*, int)> cb);

private:
    std::function<void(char*, int)> callback = nullptr;
    void do_read();

    void decode();

    tcp::socket socket_;
    enum { max_length = 544 }; // 68keypoints * 2(x,y) * 4bytes
    char data_[max_length];
};

class server
{
public:
    server(boost::asio::io_service& io_service, short port, std::function<void(char*, int)> cb);
    
private:
    void do_accept();

    tcp::acceptor acceptor_;
    tcp::socket socket_;

    std::function<void(char*, int)> callback = nullptr;
};


class serverController{
public:
    server* m_server;
    std::thread* m_thread;

    boost::asio::io_service io_service;

    bool is_running = false;
    void Run(std::function<void(char*, int)> callback);
    void Stop();

    ~serverController();
};
