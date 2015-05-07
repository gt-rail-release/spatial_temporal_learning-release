/*!
 * \file Client.h
 * \brief The abstract remote client.
 *
 * The abstract remote client contains parameters for common remote connections.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 24, 2015
 */

#ifndef SPATIAL_TEMPORAL_LEARNING_WORLDLIB_REMOTE_CLIENT_H_
#define SPATIAL_TEMPORAL_LEARNING_WORLDLIB_REMOTE_CLIENT_H_

// C++ Standard Library
#include <stdint.h>
#include <string>

namespace rail
{
namespace spatial_temporal_learning
{
namespace worldlib
{
namespace remote
{

/*!
 * \class Client
 * \brief The abstract remote client.
 *
 * The abstract remote client contains parameters for common remote connections.
 */
class Client
{
public:
  /*! The default client port. */
  static const unsigned int DEFAULT_PORT = 80;

  /*!
   * \brief Create a new Client.
   *
   * Creates a new Client with the given connection information.
   *
   * \param host The host of the remote connection (defaults to localhost).
   * \param port The host port of the remote connection (defaults to port 80).
   */
  Client(const std::string &host = "localhost", const uint16_t port = DEFAULT_PORT);

  /*!
   * \brief Host value accessor.
   *
   * Get the host value of this Client.
   *
   * \return The host value.
   */
  const std::string &getHost() const;

  /*!
   * \brief Port value accessor.
   *
   * Get the port value of this Client.
   *
   * \return The port value.
   */
  uint16_t getPort() const;

private:
  /*! Remote connection host. */
  const std::string host_;
  /*! Remote connection port. */
  uint16_t port_;
};

}
}
}
}

#endif
