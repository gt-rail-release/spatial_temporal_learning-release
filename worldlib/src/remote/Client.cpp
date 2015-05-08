/*!
 * \file Client.cpp
 * \brief The abstract remote client.
 *
 * The abstract remote client contains parameters for common remote connections.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 24, 2015
 */

// worldlib
#include "worldlib/remote/Client.h"

using namespace std;
using namespace rail::spatial_temporal_learning::worldlib::remote;

Client::Client(const string &host, const uint16_t port) : host_(host)
{
  port_ = port;
}

const string &Client::getHost() const
{
  return host_;
}

uint16_t Client::getPort() const
{
  return port_;
}
