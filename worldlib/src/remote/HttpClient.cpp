/*!
 * \file HttpClient.cpp
 * \brief The main HTTP client connection.
 *
 * The worldlib SQL HTTP can make HTTP get requests to remote web servers.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 25, 2015
 */

// worldlib
#include "worldlib/remote/HttpClient.h"

// ROS
#include <ros/ros.h>

using namespace std;
using namespace rail::spatial_temporal_learning::worldlib::remote;

HttpClient::HttpClient(const HttpClient &client) : Client(client.getHost(), client.getPort())
{
  // initialize the client
  this->init();
}

HttpClient::HttpClient(const string &host, const uint16_t port) : Client(host, port)
{
  // initialize the client
  this->init();
}

HttpClient::~HttpClient()
{
  // check for an existing connection
  if (curl_ != NULL)
  {
    curl_easy_cleanup(curl_);
  }
}

void HttpClient::init()
{

  // create the base URL
  stringstream ss;
  ss << "http://" << this->getHost() << ":" << this->getPort() << "/";
  base_ = ss.str();

  // initialize cURL
  curl_ = curl_easy_init();
  // follow redirects
  curl_easy_setopt(curl_, CURLOPT_FOLLOWLOCATION, 1L);
  // set up the read into the buffer
  curl_easy_setopt(curl_, CURLOPT_WRITEFUNCTION, curlWriteFunction);
}

string HttpClient::get(const std::string &url) const
{
  // create the URL
  string full_url = base_ + url;
  curl_easy_setopt(curl_, CURLOPT_URL, full_url.c_str());

  // create a buffer
  string buffer;
  curl_easy_setopt(curl_, CURLOPT_WRITEDATA, &buffer);

  // perform the request and check for errors
  CURLcode result = curl_easy_perform(curl_);
  if (result != CURLE_OK)
  {
    ROS_ERROR("HTTP Error: %s", curl_easy_strerror(result));
    buffer.clear();
  } else
  {
    // check if we had a valid response
    long code;
    curl_easy_getinfo(curl_, CURLINFO_RESPONSE_CODE, &code);
    if (code != 200)
    {
      ROS_ERROR("HTTP Error: %s returned Error Code %li.", full_url.c_str(), code);
      buffer.clear();
    }
  }

  return buffer;
}

static size_t rail::spatial_temporal_learning::worldlib::remote::curlWriteFunction(const void *contents,
    const size_t size, const size_t num, void *buffer)
{
  // get the size of contents
  size_t contents_size = size * num;
  // store the result in the buffer
  ((string *) buffer)->append((char *) contents, contents_size);
  return contents_size;
}
