/*
 * Copyright (c) 2019 Roboception GmbH
 *
 * Author: Monika Florek-Jasinska
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef rc_rest_api_EXCEPTIONS_H
#define rc_rest_api_EXCEPTIONS_H

namespace rc_rest_api
{

class RestClientException : public std::runtime_error
{
  public:
    explicit RestClientException(const std::string &msg) :
        std::runtime_error(msg) {}
    virtual ~RestClientException() = default;
};

class NotAvailableInThisVersionException : public RestClientException
{
  public:
    explicit NotAvailableInThisVersionException(const std::string &msg) :
        RestClientException(msg) {}
    virtual ~NotAvailableInThisVersionException() = default;
};

class MiscException : public RestClientException
{
  public:
    explicit MiscException(const std::string &msg) :
        RestClientException(msg) {}
    virtual ~MiscException() = default;
};

class RequestException : public RestClientException
{
  public:
    RequestException(const std::string &msg) :
        RestClientException(msg) {}
    virtual ~RequestException() = default;
};

}

#endif //rc_rest_api_EXCEPTIONS_H
