#include "server.h"

void serverController::Run(std::function<void(char*, int)> callback)
{
    if (!is_running) {

        try
        {
            is_running = true;
            std::cout << "Start Server at port :" << test_port << std::endl;
            m_server = new server(io_service, test_port, callback);
            m_thread = new std::thread([this]() { this->io_service.run(); });
        }
        catch (std::exception& e)
        {
            std::cerr << "Exception: " << e.what() << "\n";
        }
    }
}

void serverController::Stop()
{
    if (is_running) {
        this->io_service.stop();
        m_thread->join();
        std::cout << "DELETE THREAD...\n";
        if(m_thread != nullptr)
            delete m_thread;

        std::cout << "DELETE SERVER...\n";
        if(m_server != nullptr)
            delete m_server;

        std::cout << "Server stopped\n";
    }
    is_running = false;
}

serverController::~serverController()
{
    Stop();
}

server::server(boost::asio::io_service& io_service, short port, std::function<void(char*, int)> cb)
    : acceptor_(io_service, tcp::endpoint(tcp::v4(), port)),
    socket_(io_service)
{
    callback = cb;
    do_accept();
}

void server::do_accept()
{
    acceptor_.async_accept(socket_,
        [this](boost::system::error_code ec)
        {
            if (!ec)
            {
                auto p_session = std::make_shared<session>(std::move(socket_));
                p_session->setCallback(callback);
                p_session->start();
            }
            do_accept();
        });
}

session::session(tcp::socket socket)
    : socket_(std::move(socket))
{
}

void session::start()
{
    do_read();
}

void session::setCallback(std::function<void(char*, int)> cb)
{
    this->callback = cb;
}

void session::do_read()
{
    auto self(shared_from_this());
    boost::asio::async_read(socket_,
        boost::asio::buffer(data_, max_length),
        [this, self](boost::system::error_code ec, std::size_t length)
        {
            if (!ec)
            {
                decode();
                do_read();
            }
        });
}

void session::decode()
{
    if (callback != nullptr) {
        char buffer[max_length];
        std::memcpy(buffer, data_, max_length);
        callback(buffer, max_length);
    }
}
