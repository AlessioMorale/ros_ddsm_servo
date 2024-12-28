#ifndef SERIAL_PORT_HPP
#define SERIAL_PORT_HPP

#include <boost/asio.hpp>
#include <deque>
#include <iostream>
#include <vector>
#include "port.hpp"
namespace ddsm210_driver::comm
{

class SerialPort : public Port
{
public:

  SerialPort()
    : io_context_()
    , work_guard_(boost::asio::make_work_guard(io_context_))
    , serial_port_(io_context_)
    , read_buffer_(1024)
    , tx_strand_(boost::asio::make_strand(io_context_))
    , rx_strand_(boost::asio::make_strand(io_context_))
  {
    // Start the IO thread
    io_thread_ = std::thread([this]() {
        try {
            io_context_.run();
        }
        catch (const std::exception& e) {
            // Log error or handle exception
            std::cerr << "IO Context exception: " << e.what() << std::endl;
        }
    });
  }

  virtual ~SerialPort()
  {
    work_guard_.reset();  // Allow io_context to complete
    io_context_.stop();   // Stop io_context
    
    if (io_thread_.joinable()) {
        io_thread_.join();
    }
  }
  bool open(const std::string& port, unsigned int baud_rate)
  {
    try
    {
      serial_port_.open(port);
      serial_port_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
      serial_port_.set_option(boost::asio::serial_port_base::character_size(8));
      serial_port_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
      serial_port_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
      serial_port_.set_option(
          boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

      start_receive();
      return true;
    }
    catch (const boost::system::system_error& e)
    {
      std::cout << "Error" << e.code() << std::endl;
      return false;
    }
  }

  virtual void close() override
  {
    // Use both strands to ensure all operations complete before closing
    boost::asio::post(tx_strand_, [this]() {
      boost::asio::post(rx_strand_, [this]() {
        if (serial_port_.is_open())
        {
          boost::system::error_code ec;
          serial_port_.close(ec);
        }
      });
    });
  }

  virtual void set_receive_callback(receive_callback_t callback) override
  {
    boost::asio::post(rx_strand_,
                      [this, callback = std::move(callback)]() { receive_callback_ = std::move(callback); });
  }

  virtual void send(const std::vector<uint8_t>& data) override
  {
    boost::asio::post(tx_strand_, [this, data = std::move(data)]() mutable {
      bool write_in_progress = !write_queue_.empty();
      write_queue_.push_back(std::move(data));

      if (!write_in_progress)
      {
        start_send();
      }
    });
  }

private:
  void start_receive()
  {
    std::cout << "start_receive" << std::endl;
    serial_port_.async_read_some(
        boost::asio::buffer(read_buffer_),
        boost::asio::bind_executor(rx_strand_, [this](const boost::system::error_code& error,
                                                      std::size_t bytes_transferred) {
          std::cout << "async_read_some" << std::endl;
          if (!error)
          {
            std::cout << "error " << error << std::endl;

            if (receive_callback_)
            {
              receive_callback_(std::vector<uint8_t>(read_buffer_.begin(), read_buffer_.begin() + bytes_transferred));
            }
            start_receive();
          }
        }));
  }

  void start_send()
  {
    if (write_queue_.empty())
    {
      return;
    }

    boost::asio::async_write(serial_port_, boost::asio::buffer(write_queue_.front()),
                             boost::asio::bind_executor(tx_strand_, [this](const boost::system::error_code& error,
                                                                           std::size_t /*bytes_transferred*/) {
                               if (!error)
                               {
                                 write_queue_.pop_front();
                                 if (!write_queue_.empty())
                                 {
                                   start_send();
                                 }
                               }
                             }));
  }
  boost::asio::io_context io_context_;
  boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard_;
  boost::asio::serial_port serial_port_;
  std::vector<uint8_t> read_buffer_;
  boost::asio::strand<boost::asio::io_context::executor_type> tx_strand_;  // Transmit strand
  boost::asio::strand<boost::asio::io_context::executor_type> rx_strand_;  // Receive strand
  std::deque<std::vector<uint8_t>> write_queue_;
  receive_callback_t receive_callback_;
  std::thread io_thread_;
};
};  // namespace ddsm210_driver
#endif  // SERIAL_PORT_HPP
