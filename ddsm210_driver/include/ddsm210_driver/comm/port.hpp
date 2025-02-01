// Copyright 2025 Alessio Morale <alessiomorale-at-gmail.com>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef DDSM210_DRIVER__COMM__PORT_HPP_
#define DDSM210_DRIVER__COMM__PORT_HPP_
#include <cstdint>
#include <functional>
#include <vector>
namespace ddsm210_driver::comm
{
using receive_callback_t = std::function<void(const std::vector<uint8_t> &)>;
class Port
{
public:
  virtual ~Port() = default;
  virtual void close() = 0;
  virtual void set_receive_callback(receive_callback_t callback) = 0;
  virtual void send(const std::vector<uint8_t> & data) = 0;
};
};  // namespace ddsm210_driver::comm
#endif  // DDSM210_DRIVER__COMM__PORT_HPP_
