/*!
 * \file HttpClient.h
 * \brief The main HTTP client connection.
 *
 * The worldlib SQL HTTP can make HTTP get requests to remote web servers.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 25, 2015
 */

#ifndef SPATIAL_TEMPORAL_LEARNING_WORLDLIB_REMOTE_HTTP_CLIENT_H_
#define SPATIAL_TEMPORAL_LEARNING_WORLDLIB_REMOTE_HTTP_CLIENT_H_

// worldlib
#include "Client.h"

// cURL
#include <curl/curl.h>

namespace rail
{
namespace spatial_temporal_learning
{
namespace worldlib
{
namespace remote
{

/*!
 * \class HttpClient
 * \brief The main HTTP client connection.
 *
 * The worldlib SQL HTTP can make HTTP get requests to remote web servers.
 */
class HttpClient : public Client
{
public:
  /*! The default HTTP port. */
  static const unsigned int DEFAULT_PORT = 80;

  /*!
   * \brief Create a new HttpClient.
   *
   * Creates a new HttpClient by copying the values from the given HttpClient.
   *
   * \param client The HttpClient to copy.
   */
  HttpClient(const HttpClient &client);

  /*!
   * \brief Create a new HttpClient.
   *
   * Creates a new HttpClient with the given connection information.
   *
   * \param host The host of the server (without a protocol specification; defaults to localhost).
   * \param port The host port of the server (defaults to port 80).
   */
  HttpClient(const std::string &host = "localhost", const uint16_t port = 80);

  /*!
   * \brief Cleans up a HttpClient.
   *
   * Cleans up any connections used by the HttpClient.
   */
  virtual ~HttpClient();

protected:
  /*!
   * \brief Execute a HTTP GET request to the server.
   *
   * Attempt to make an HTTP GET request to the server. The request is made to http://<host>:<port>/<url>. If the
   * request fails, an empty string is returned.
   *
   * \param url The URL (minus the host information) to make the request to.
   * \return The resulting contents of the HTTP request or the empty string if an error occured.
   */
  std::string get(const std::string &url) const;

private:
  /*! The main HTTP connection. */
  CURL *curl_;
  /*! The base URL. */
  std::string base_;

  /*!
   * \brief Initialize the cURL client.
   *
   * Initialize the cURL client paramters.
   */
  void init();
};

/*!
 * \brief Callback for a cURL read.
 *
 * Read the contents from a cURL request into the given buffer.
 *
 * \param contents The buffer containing the contents to read.
 * \param size The size of each element to read (buffer size is size*num).
 * \param num The number of elements to read (buffer size is size*num).
 * \param buffer The buffer to read elements into.
 * \return The number of bytes read.
 */
static size_t curlWriteFunction(const void *contents, const size_t size, const size_t num, void *buffer);

}
}
}
}

#endif
